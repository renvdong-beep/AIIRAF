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


class IntentNoMatch(IntentProviderError):
    """这句话**不是机器人任务**（模型判定与所有 Skill 契约都不匹配）。

    单独成类而不是复用 IntentProviderError：调用方需要能把"过滤掉了"与"解析坏了"分开。
    平台把整段对话都转发给 IRAF 时（机器人开发模式），这就是过滤器输出的那个信号。
    """

    def __init__(self, reason, matches=()):
        self.reason = reason or "与已声明的机器人技能都不匹配"
        self.matches = list(matches)
        super().__init__("IRAF-NOT-ROBOT-TASK: " + self.reason)


@dataclass(frozen=True)
class IntentProviderConfig:
    endpoint: str
    model: str
    token: str
    timeout_seconds: float = 10.0
    verify_tls: bool = True
    # auto = 依次尝试 json_schema → json_object → none（默认）；也可强制某一档，便于复现问题
    response_format_mode: str = "auto"
    # 2026-10-19 板上实测：写死 256 会被推理型模型吃光（finish_reason=length、
    # completion_tokens=256、content 为空 ⇒ "模型未返回严格 JSON 意图"）。默认 1024，可部署期覆盖。
    max_tokens: int = 1024


class OpenAICompatibleIntentProvider:
    name = "openai-compatible-intent"
    version = "1.2.0"

    def __init__(self, config):
        self.config = config
        self.model_id = config.model
        # 最近一次调用的诊断信息（finish_reason / completion_tokens），用于报错定位
        self._last_meta = {}

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
        # 「不是机器人任务」也要能**干净地**表达出来（否则模型只能拒答/回非 JSON，
        # 我们就被迫把"过滤"报成"解析失败"）。skill 为空串 = 不匹配任何技能。
        variants.append({"type": "object",
                         "properties": {"skill": {"const": ""}, "reason": {"type": "string"}},
                         "required": ["skill", "reason"], "additionalProperties": False})
        response_schema = {"oneOf": variants}
        system = ("你是机器人意图解析器。根据 Skill 契约和示例选择一个 Skill，并生成严格符合 schema 的 parameters。禁止解释、Markdown、电机命令或未声明动作。对于 positions 对象，只能使用 inputSchema.properties 中列出的真实关节名，禁止把字段名 positions 当成关节名。Skill契约：" + json.dumps(contracts, ensure_ascii=False, separators=(",", ":"))
                  # 2026-10-19 板上实测：网关不支持 json_schema 时会降级到 json_object/none，
                  # 模型便自己发明顶层键（返回 {"type": "move_joint", ...}）⇒ 被 schema 校验拒绝。
                  # 这里显式给出顶层形状与可选技能名，**不放松校验**，只是把要求说清楚。
                  + "\n输出要求：只输出一个 JSON 对象，顶层**只能**有两个字段："
                    '{"skill": "<技能名>", "parameters": { ... }}；不要使用 type/action/name 等别名。'
                    "skill 必须取以下之一：" + ", ".join(sorted(contracts)) + "。"
                    # 机器人开发模式下，平台会把**整段对话**转发过来；与机器人无关的话必须能被
                    # 干净地判掉（这是"过滤"的实现点），而不是让模型拒答或输出非 JSON。
                    + "\n如果这句话**不是机器人任务**（与上面任何 Skill 都无关，例如闲聊、天气、软件问答），"
                      '输出 {"skill": "", "reason": "<一句话原因>"}，不要硬套某个技能。')
        content = self._chat(system, text, response_schema)
        try:
            intent = json.loads(content.strip())
        except ValueError:
            try:
                intent = _extract_json_object(content)
            except IntentProviderError as exc:
                # 带上 finish_reason/max_tokens：否则"模型未返回严格 JSON"无法判断是
                # 截断、拒答还是网关返回空（2026-10-19 板卡上就是这样卡了一轮）。
                raise IntentProviderError(
                    "%s（finish_reason=%s, completion_tokens=%s, content_len=%d, max_tokens=%d）"
                    % (exc, self._last_meta.get("finish_reason"),
                       self._last_meta.get("completion_tokens"),
                       len(content or ""), int(self.config.max_tokens or 1024))) from exc
        errors = sorted(Draft202012Validator(response_schema).iter_errors(intent), key=lambda e: list(e.path))
        if errors:
            raise IntentProviderError("模型意图不符合 Skill schema: " + errors[0].message)
        if not intent.get("skill"):
            raise IntentNoMatch(intent.get("reason") or "", matches=allowed)
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
                "temperature": 0, "max_tokens": int(self.config.max_tokens or 1024),
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
                choice = raw["choices"][0]
                self._last_meta = {"finish_reason": choice.get("finish_reason"),
                                   "completion_tokens": (raw.get("usage") or {}).get("completion_tokens")}
                return choice["message"]["content"]
            except (KeyError, IndexError, TypeError) as exc:
                raise IntentProviderError("模型响应缺少 choices[0].message.content") from exc
        raise IntentProviderError("意图 Provider 调用失败（所有 response_format 档位均被拒绝）：%s"
                                  % last_error)

    @staticmethod
    def _validate_profile_semantics(intent, contracts):
        contract = contracts.get(intent.get("skill"), {})
        joints = set(contract.get("robotJoints") or ())
        positions = (intent.get("parameters") or {}).get("positions")
        if joints and isinstance(positions, dict):
            unknown = set(positions) - joints
            if unknown:
                raise IntentProviderError("模型意图包含未声明关节: " + str(sorted(unknown)))


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
