"""OpenAI-compatible Model Provider，仅负责受约束意图解析。"""
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


class OpenAICompatibleIntentProvider:
    name = "openai-compatible-intent"
    version = "1.1.0"

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
        body = {"model": self.config.model, "messages": [{"role": "system", "content": system}, {"role": "user", "content": text}], "temperature": 0, "max_tokens": 256, "chat_template_kwargs": {"enable_thinking": False}, "response_format": {"type": "json_schema", "json_schema": {"name": "robot_intent", "schema": response_schema}}}
        headers = {"Content-Type": "application/json"}
        if self.config.token:
            headers["Authorization"] = "Bearer " + self.config.token
        request = Request(self.config.endpoint.rstrip("/") + "/chat/completions", data=json.dumps(body, ensure_ascii=False).encode(), headers=headers, method="POST")
        context = None if self.config.verify_tls else ssl._create_unverified_context()
        try:
            with urlopen(request, timeout=self.config.timeout_seconds, context=context) as response:
                raw = json.loads(response.read().decode())
        except (HTTPError, URLError, TimeoutError, ValueError) as exc:
            raise IntentProviderError("意图 Provider 调用失败: " + str(exc)) from exc
        try:
            intent = json.loads(raw["choices"][0]["message"]["content"].strip())
        except (KeyError, IndexError, TypeError, ValueError) as exc:
            raise IntentProviderError("模型未返回严格 JSON 意图") from exc
        errors = sorted(Draft202012Validator(response_schema).iter_errors(intent), key=lambda e: list(e.path))
        if errors:
            raise IntentProviderError("模型意图不符合 Skill schema: " + errors[0].message)
        self._validate_profile_semantics(intent, contracts)
        return intent

    @staticmethod
    def _validate_profile_semantics(intent, contracts):
        contract = contracts.get(intent.get("skill"), {})
        joints = set(contract.get("robotJoints") or ())
        positions = (intent.get("parameters") or {}).get("positions")
        if joints and isinstance(positions, dict):
            unknown = set(positions) - joints
            if unknown:
                raise IntentProviderError("模型意图包含未声明关节: " + str(sorted(unknown)))
