# -*- coding: utf-8 -*-
"""意图 Provider 的响应解析与 response_format 降级链（2026-10-19 板卡实测驱动的回归）。

为什么要这些用例：
  · 板卡上首次语义调用成功走到 LLM，但 400 卡在 `response_format=json_schema`
    （网关不支持）⇒ 加降级链后必须有"降级顺序 + 不降级校验"的回归。
  · 我修降级链时把 `_validate_profile_semantics` 挤成了嵌套函数，类上失去该属性，
    板上报 AttributeError。**只测 _chat 不测 parse 就会漏掉** ⇒ 这里直接测 parse()。
"""
import io
import json
import sys
import unittest
from pathlib import Path
from urllib.error import HTTPError

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "src"))

from iraf_adapters.model.openai_intent import (  # noqa: E402
    IntentProviderConfig,
    IntentProviderError,
    OpenAICompatibleIntentProvider,
)

CONTRACTS = {
    "pick_object": {
        "inputSchema": {
            "type": "object",
            "required": ["target_id", "grasp_pose"],
            "properties": {
                "target_id": {"type": "string"},
                "grasp_pose": {"type": "object"},
            },
            "additionalProperties": False,
        }
    }
}


def make_provider(mode="auto"):
    return OpenAICompatibleIntentProvider(
        IntentProviderConfig("http://gw.invalid/v1", "deepseek-flash", "k", 5.0, True, mode))


def ok_response(content):
    return {"choices": [{"message": {"content": content}}]}


class IntentProviderTest(unittest.TestCase):
    def test_parse_returns_validated_intent(self):
        """parse() 必须能在类上找到 _validate_profile_semantics 并跑完校验。"""
        provider = make_provider("json_schema")
        provider._post = lambda body: ok_response(
            json.dumps({"skill": "pick_object",
                        "parameters": {"target_id": "box_01",
                                       "grasp_pose": {"frame_id": "world"}}}))
        intent = provider.parse("抓取 box_01", CONTRACTS)
        self.assertEqual(intent["skill"], "pick_object")

    def test_falls_back_when_gateway_rejects_json_schema(self):
        """网关 400 + 提到 response_format ⇒ 依次降级，最终用 json_object 成功。"""
        seen = []

        def fake_post(body):
            seen.append(body.get("response_format", {}).get("type", "none"))
            if body.get("response_format", {}).get("type") == "json_schema":
                raise HTTPError("u", 400, "bad", {},
                                io.BytesIO(b'{"error":{"message":"This response_format type is unavailable now"}}'))
            return ok_response(json.dumps({"skill": "pick_object",
                                           "parameters": {"target_id": "box_01",
                                                          "grasp_pose": {"frame_id": "world"}}}))

        provider = make_provider("auto")
        provider._post = fake_post
        intent = provider.parse("抓取 box_01", CONTRACTS)
        self.assertEqual(intent["skill"], "pick_object")
        self.assertEqual(seen, ["json_schema", "json_object"])

    def test_other_400_is_not_swallowed(self):
        """不相关的 400（例如鉴权/参数错）必须原样失败，不能靠降级掩盖。"""
        def fake_post(body):
            raise HTTPError("u", 400, "bad", {},
                            io.BytesIO(b'{"error":{"message":"invalid api key"}}'))

        provider = make_provider("auto")
        provider._post = fake_post
        with self.assertRaises(IntentProviderError) as ctx:
            provider.parse("抓取 box_01", CONTRACTS)
        self.assertIn("invalid api key", str(ctx.exception))

    def test_markdown_fenced_json_is_extracted(self):
        """降级到 none 时模型可能包 ```json 围栏：定位后仍要用 schema 校验。"""
        provider = make_provider("none")
        provider._post = lambda body: ok_response(
            "```json\n" + json.dumps({"skill": "pick_object",
                                      "parameters": {"target_id": "box_01",
                                                     "grasp_pose": {"frame_id": "world"}}}) + "\n```")
        self.assertEqual(provider.parse("抓取 box_01", CONTRACTS)["skill"], "pick_object")

    def test_schema_violation_is_rejected(self):
        provider = make_provider("none")
        provider._post = lambda body: ok_response(json.dumps({"skill": "no_such_skill",
                                                              "parameters": {}}))
        with self.assertRaises(IntentProviderError) as ctx:
            provider.parse("随便", CONTRACTS)
        self.assertIn("schema", str(ctx.exception).lower())

    def test_unknown_joint_is_rejected(self):
        contracts = {"move_joint": {
            "inputSchema": {"type": "object", "required": ["positions"],
                            "properties": {"positions": {"type": "object"}},
                            "additionalProperties": False},
            "robotJoints": ["j1", "j2"],
        }}
        provider = make_provider("none")
        provider._post = lambda body: ok_response(json.dumps(
            {"skill": "move_joint", "parameters": {"positions": {"bogus": 1.0}}}))
        with self.assertRaises(IntentProviderError) as ctx:
            provider.parse("动一下", contracts)
        self.assertIn("未声明关节", str(ctx.exception))


if __name__ == "__main__":
    unittest.main()
