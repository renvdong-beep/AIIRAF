"""OpenAI-compatible Model Provider，仅负责受约束意图解析。

2026-10-19（E300 跨机实测）：不少 OpenAI 兼容网关**不支持** `response_format.type=json_schema`
（实测 New API 代理 DeepSeek 直接返回 400 `This response_format type is unavailable now`，
而同一请求去掉 response_format 就 200）。因此请求层做成**降级链**：
`json_schema → json_object → 不带 response_format`，并且**降级不等于放宽校验**——
拿到文本后仍按同一份 JSON Schema 用 Draft202012Validator 校验，不合法照样失败。
"""
from dataclasses import dataclass
import json
import ssl
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen
from jsonschema import Draft202012Validator


class IntentProviderError(RuntimeError):
    pass


@dataclass(frozen=True)
class IntentProviderConfig:
    endpoint: str
    model: str
    token: str
    timeout_seconds: float = 10.0
    verify_tls: bool = True
    # auto = 依次尝试 json_schema → json_object → none（默认）；也可强制某一档，便于复现问题
    response_format_mode: str = "auto"


class OpenAICompatibleIntentProvider:
    name = "openai-compatible-intent"
    version = "1.2.0"

    def __init__(self, config):
        self.config = config
        self.model_id = config.model

    def parse(self, text, skill_contracts):
        if not isinstance(text, str) or not text.strip():
            raise IntentProviderError("意图文本不能为空")
        contracts = skill_contracts if isinstance(skill_contracts, dict) else {name: {"inputSchema": {"type": "object"}} for name in skill_contracts}
        allowed = tuple(sorted(contracts))
        if not allowed:
            raise IntentProviderError("没有可用 Skill")
        variants = []
        for name in allowed:
            variants.append({"type": "object", "properties": {"skill": {"const": name}, "parameters": contracts[name]["inputSchema"]}, "required": ["skill", "parameters"], "additionalProperties": False})
        response_schema = {"oneOf": variants}
        system = "你是机器人意图解析器。根据 Skill 契约和示例选择一个 Skill，并生成严格符合 schema 的 parameters。禁止解释、Markdown、电机命令或未声明动作。对于 positions 对象，只能使用 inputSchema.properties 中列出的真实关节名，禁止把字段名 positions 当成关节名。Skill契约：" + json.dumps(contracts, ensure_ascii=False, separators=(",", ":"))
        content = self._chat(system, text, response_schema)
        try:
            intent = json.loads(content.strip())
        except ValueError:
            intent = _extract_json_object(content)
        errors = sorted(Draft202012Validator(response_schema).iter_errors(intent), key=lambda e: list(e.path))
        if errors:
            raise IntentProviderError("模型意图不符合 Skill schema: " + errors[0].message)
        self._validate_profile_semantics(intent, contracts)
        return intent

    # ---------- 传输层 ----------
    def _modes(self):
        mode = str(self.config.response_format_mode or "auto").strip().lower()
        if mode == "auto":
            return ("json_schema", "json_object", "none")
        if mode not in {"json_schema", "json_object", "none"}:
            raise IntentProviderError("未知的 IRAF_AGENTOS_RESPONSE_FORMAT：%s" % mode)
        return (mode,)

    def _body(self, system, text, schema, mode):
        body = {"model": self.config.model,
                "messages": [{"role": "system", "content": system},
                             {"role": "user", "content": text}],
                "temperature": 0, "max_tokens": 256,
                "chat_template_kwargs": {"enable_thinking": False}}
        if mode == "json_schema":
            body["response_format"] = {"type": "json_schema",
                                       "json_schema": {"name": "robot_intent", "schema": schema}}
        elif mode == "json_object":
            body["response_format"] = {"type": "json_object"}
        return body

    def _post(self, body):
        headers = {"Content-Type": "application/json"}
        if self.config.token:
            headers["Authorization"] = "Bearer " + self.config.token
        request = Request(self.config.endpoint.rstrip("/") + "/chat/completions",
                          data=json.dumps(body, ensure_ascii=False).encode(),
                          headers=headers, method="POST")
        context = None if self.config.verify_tls else ssl._create_unverified_context()
        with urlopen(request, timeout=self.config.timeout_seconds, context=context) as response:
            return json.loads(response.read().decode())

    def _chat(self, system, text, schema):
        """按降级链请求；只有"网关不支持该 response_format"这一类 400 才降级。"""
        modes = self._modes()
        last_error = None
        for index, mode in enumerate(modes):
            try:
                raw = self._post(self._body(system, text, schema, mode))
            except HTTPError as exc:
                detail = ""
                try:
                    detail = exc.read().decode()[:300]
                except Exception:  # noqa: BLE001 - 读错误体失败不应掩盖原始错误
                    pass
                unsupported = (exc.code == 400 and "response_format" in detail.lower())
                if unsupported and index + 1 < len(modes):
                    last_error = "%s(HTTP %s)" % (detail or exc, exc.code)
                    continue
                raise IntentProviderError(
                    "意图 Provider 调用失败: HTTP %s %s" % (exc.code, detail)) from exc
            except (URLError, TimeoutError, ValueError) as exc:
                raise IntentProviderError("意图 Provider 调用失败: " + str(exc)) from exc
            try:
                return raw["choices"][0]["message"]["content"]
            except (KeyError, IndexError, TypeError) as exc:
                raise IntentProviderError("模型响应缺少 choices[0].message.content") from exc
        raise IntentProviderError("意图 Provider 调用失败（所有 response_format 档位均被拒绝）：%s"
                                  % last_error)


def _extract_json_object(text):
    """从可能带 Markdown 围栏/前后缀的回复里抠出第一个完整 JSON 对象。

    降级到 json_object / none 时模型可能包 ```json 围栏；这里只做**定位**，
    不做字段补全——补全会让 schema 校验失去意义。
    """
    if not isinstance(text, str):
        raise IntentProviderError("模型返回内容不是文本")
    start = text.find("{")
    end = text.rfind("}")
    if start < 0 or end <= start:
        raise IntentProviderError("模型未返回严格 JSON 意图")
    try:
        return json.loads(text[start:end + 1])
    except ValueError as exc:
        raise IntentProviderError("模型返回的 JSON 无法解析: %s" % exc) from exc

    @staticmethod
    def _validate_profile_semantics(intent, contracts):
        contract = contracts.get(intent.get("skill"), {})
        joints = set(contract.get("robotJoints") or ())
        positions = (intent.get("parameters") or {}).get("positions")
        if joints and isinstance(positions, dict):
            unknown = set(positions) - joints
            if unknown:
                raise IntentProviderError("模型意图包含未声明关节: " + str(sorted(unknown)))
