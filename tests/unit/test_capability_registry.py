# -*- coding: utf-8 -*-
"""能力参数表与 Skill 目录对账（静态检查，不 import grpc）。

为什么要有这条：
  2026-10-19 板上实测：语义解析出 `stop` 后被执行层打成
  `IRAF-CAPABILITY-UNKNOWN 能力 'stop' 未登记参数形状` —— 因为
  `provider_service._POSITIONAL` 只登记了 `emergency_stop`，没有 `stop`。
  这类"技能存在、能力没登记"的缺口不会在编译期或普通单测里暴露，
  只能在板上跑到才炸；用 AST 静态对账可以提前拦住。
"""
import ast
import sys
import unittest
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
SERVICE = REPO / "src/iraf_adapters/remote/provider_service.py"
SKILLS_DIR = REPO / "skills"

#: 不是"能力实现"的技能：display_pick 是组合演示、engineering.robot_adapter 是开发工具，
#: 二者在别处编排，不经过 Provider 的单能力入口。
NOT_PROVIDER_CAPABILITIES = {"display_pick", "engineering.robot_adapter"}


def positional_capabilities() -> set:
    tree = ast.parse(SERVICE.read_text(encoding="utf-8"))
    for node in tree.body:
        if isinstance(node, ast.Assign) and any(
                isinstance(t, ast.Name) and t.id == "_POSITIONAL" for t in node.targets):
            return {k.value for k in node.value.keys}
    raise AssertionError("provider_service.py 里找不到 _POSITIONAL 定义")


def skill_names() -> set:
    return {d.name for d in SKILLS_DIR.iterdir()
            if d.is_dir() and (d / "skill.yaml").is_file()}


class CapabilityRegistryTest(unittest.TestCase):
    def test_every_skill_is_registered_as_capability(self):
        missing = sorted(skill_names() - NOT_PROVIDER_CAPABILITIES - positional_capabilities())
        self.assertEqual(missing, [], "这些技能在 Provider 能力表里没有登记参数形状：%s" % missing)

    def test_stop_is_zero_arg_capability(self):
        """零参数能力必须是空元组（否则调用时会传错参数个数）。"""
        tree = ast.parse(SERVICE.read_text(encoding="utf-8"))
        for node in tree.body:
            if isinstance(node, ast.Assign) and any(
                    isinstance(t, ast.Name) and t.id == "_POSITIONAL" for t in node.targets):
                table = {k.value: (v.elts if isinstance(v, (ast.Tuple, ast.List)) else None)
                         for k, v in zip(node.value.keys, node.value.values)}
                self.assertIn("stop", table)
                self.assertEqual(len(table["stop"]), 0)
                return
        self.fail("找不到 _POSITIONAL")

    def test_registry_names_exist_as_skills_or_are_documented(self):
        """反向：表里多余的名字要么有对应技能，要么是显式保留（read_state/emergency_stop）。"""
        reserved = {"read_state", "emergency_stop"}
        orphans = sorted(positional_capabilities() - skill_names() - reserved)
        self.assertEqual(orphans, [], "能力表里有对不上任何技能的名字：%s" % orphans)


if __name__ == "__main__":
    unittest.main()
