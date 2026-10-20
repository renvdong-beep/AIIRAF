# -*- coding: utf-8 -*-
"""IRAF 的北向 MCP 绑定（任务级）。

定位：`iraf_adapters` 层的一个**北向适配器**，把 IRAF 的公开任务入口暴露成 MCP 工具，
供 AgentOS 等平台把「语义 → 任务」这一步交出去，而**机器人 Skill 的选择/校验/策略
仍然完全在 IRAF 的 Skill Manager 内**。

层次纪律（2026-10-20 使用者纠正后固化）：
  · AgentOS 的 skills 与 IRAF Skill Manager 管理的机器人 Skill **不是一个概念**，不得互相注册；
  · 因此本适配器只提供任务级工具（submit/get），**禁止**提供"逐个技能"的工具
    （那等于允许平台绕过 Skill Manager 直连 skill.execute）；
  · 回归门禁见 `tests/unit/test_mcp_server_layer.py`。

运行方式：`python3 -m iraf_adapters.mcp.server`（stdio）。
"""
from .server import app, main  # noqa: F401

__all__ = ["app", "main"]
