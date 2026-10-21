# -*- coding: utf-8 -*-
""""非机器人任务"必须是**可判读的过滤**，不能混成解析失败。

背景（2026-10-21 使用者需求）：机器人开发模式下，平台会把整段对话都转发给 IRAF，
与机器人无关的话（天气、闲聊）要由 IRAF 判掉。判掉要给出明确码 IRAF-NOT-ROBOT-TASK，
并在 reason 里列可用技能，平台据此回复用户"这不是机器人任务"。
"""
import unittest

from iraf_adapters.agentos.bridge import AgentOSBridge
from iraf_adapters.model.openai_intent import IntentNoMatch


class _Registry:
    def intent_contracts(self):
        return {}


class _Profile:
    joints = ()


class _Dispatcher:
    registry = _Registry()
    profile = _Profile()

    def __init__(self):
        self.calls = []

    def record_pre_dispatch_failure(self, request, context, code, reason, metadata):
        self.calls.append((code, reason))
        return {"status": "FAILED", "error_code": code, "reason": reason}


class _NoMatchProvider:
    def parse(self, text, contracts):
        raise IntentNoMatch("与机器人任务无关", matches=("pick_object", "place_object"))


class _BoomProvider:
    def parse(self, text, contracts):
        raise RuntimeError("should not be used")


class IntentNoMatchMappingTest(unittest.TestCase):
    def test_no_match_maps_to_dedicated_code(self):
        dispatcher = _Dispatcher()
        bridge = AgentOSBridge(_NoMatchProvider(), dispatcher)
        request = {"text": "今天天气怎么样", "request_id": "r1",
                   "correlation_id": "c1", "idempotency_key": "k1"}
        result = bridge.execute_intent(request, object())
        self.assertEqual(result["error_code"], "IRAF-NOT-ROBOT-TASK")
        self.assertIn("pick_object", result["reason"])
        self.assertEqual(dispatcher.calls[0][0], "IRAF-NOT-ROBOT-TASK")

    def test_no_match_exception_carries_matches(self):
        exc = IntentNoMatch("闲聊", matches=("a", "b"))
        self.assertEqual(exc.matches, ["a", "b"])
        self.assertIn("IRAF-NOT-ROBOT-TASK", str(exc))


if __name__ == "__main__":
    unittest.main()
