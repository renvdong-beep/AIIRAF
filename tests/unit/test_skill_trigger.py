# -*- coding: utf-8 -*-
"""技能选择收口：manifest 的 intentTrigger 与 intent_contracts 的 trigger 字段。

实机（2026-10-23 AgentOS 会话）：说"抓取 box_01" 时模型选到了 visual_pick，
于是自己猜坐标 ⇒ 必然失败 `distance=0.094719m tolerance=0.005000m`。
根因是技能意图示例里 "视觉抓取方块" 被"抓取"命中，而平台只看 examples、不知道
"这个技能需要显式 vision_file、不能一句话触发"。

修法：manifest 增加可选 `spec.intentTrigger`（natural_language / requires / note），
registry 容错读入，intent_contracts 连同 examples 一起给平台。
"""
import ast
import unittest
from pathlib import Path

from iraf_core.registry import SkillRegistry, load_skill_manifest

REPO = Path(__file__).resolve().parents[2]
SKILLS = REPO / "skills"


class IntentTriggerTest(unittest.TestCase):
    def test_pick_object_allows_natural_language(self):
        m = load_skill_manifest(SKILLS / "pick_object" / "skill.yaml")
        self.assertTrue(m.intent_trigger.get("natural_language"))

    def test_visual_pick_is_not_natural_language(self):
        m = load_skill_manifest(SKILLS / "visual_pick" / "skill.yaml")
        self.assertFalse(m.intent_trigger.get("natural_language"))
        self.assertIn("vision_file", m.intent_trigger.get("requires") or [])

    def test_place_object_declares_scene_requirement(self):
        m = load_skill_manifest(SKILLS / "place_object" / "skill.yaml")
        requires = " ".join(m.intent_trigger.get("requires") or [])
        self.assertIn("接收体", requires)
        self.assertIn("tray_01", m.intent_trigger.get("note") or "")

    def test_manifest_without_trigger_is_backward_compatible(self):
        """没写 intentTrigger 的技能必须照旧可用（缺省视为可自然语言触发）。"""
        registry = SkillRegistry().load_directory(str(SKILLS))
        contracts = registry.intent_contracts()
        missing = [n for n, c in contracts.items() if "trigger" not in c]
        self.assertEqual([], missing, "intent_contracts 必须给每个技能带 trigger：%s" % missing)
        # 至少有一个技能是"没声明"的 ⇒ 走缺省自然语言
        defaults = [n for n, c in contracts.items()
                    if c["trigger"] == {"natural_language": True}]
        self.assertTrue(defaults, "缺省 trigger 没有任何技能命中，容错分支没被覆盖")

    def test_server_exposes_skill_triggers(self):
        """平台要拿到 skill_triggers，否则只能靠读 examples 猜（会再次选错技能）。"""
        server = REPO / "src/iraf_adapters/mcp/server.py"
        tree = ast.parse(server.read_text(encoding="utf-8"))
        keys = set()
        for node in ast.walk(tree):
            if isinstance(node, ast.Subscript) and isinstance(node.ctx, ast.Store):
                sl = node.slice
                if isinstance(sl, ast.Constant) and isinstance(sl.value, str):
                    keys.add(sl.value)
        self.assertIn("skill_triggers", keys, "server 的返回里没有 skill_triggers")

    def test_routing_policy_mentions_trigger_rules(self):
        from iraf_adapters.mcp.mode import ROUTING_POLICY
        self.assertIn("skill_triggers", ROUTING_POLICY)
        self.assertIn("natural_language=false", ROUTING_POLICY)


if __name__ == "__main__":
    unittest.main()
