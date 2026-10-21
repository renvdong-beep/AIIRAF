# -*- coding: utf-8 -*-
"""IRAF 侧的"机器人开发模式"状态机（纯逻辑，不依赖 mcp/grpc，可单测）。

为什么需要它（2026-10-21 使用者口径）：
  平台（AgentOS 等）自己的 skill 与 IRAF 的机器人 Skill 是两套概念。当一句话有歧义
  （"抓取"既可能是平台动作也可能指机器人任务），平台模型容易走偏。
  解法不靠改别人的容器/框架，只在 **IRAF 自己的北向**加一个显式模式：
    用户说"开始机器人开发模式" → 平台调用 iraf_robot_mode(action=enter)
    → 之后 iraf_submit_task 的返回里带 mode=robot_dev 与技能清单
    → 语义被显式收进"机器人任务域"，平台自己也知道现在在跟机器人打交道。

状态是**进程内**的：stdio 形态一个 server 进程就是一个会话；HTTP 形态按 server 进程共享。
如需跨进程共享，以后可换成事件库/文件，但当前不需要（一个会话一个进程）。
"""
from __future__ import annotations

import time

DEFAULT_TTL_S = 1800
MODE_IDLE = "idle"
MODE_ROBOT = "robot_dev"

HINT_ENTER = ("机器人开发模式已开启：后续所有动作类请求都应当作为一个**任务**提交给 "
              "iraf_submit_task，由 IRAF 的 Skill Manager 选择机器人技能；"
              "不要把它拆成平台自己的动作，也不要在平台侧伪造结果。")
HINT_IDLE = ("当前不在机器人开发模式：如要开发/调试机器人应用，先调用 "
             "iraf_robot_mode(action=\"enter\")，再提交任务。")


class RobotModeState:
    """一个进程内的模式状态：idle ⇄ robot_dev，带 TTL，可显式退出。"""

    def __init__(self, clock=time.time):
        self._clock = clock
        self._entered_at = None
        self._expires_at = None

    # ---- 查询 -------------------------------------------------------------
    def active(self) -> bool:
        if self._expires_at is None:
            return False
        if self._clock() >= self._expires_at:
            self._entered_at = None
            self._expires_at = None
            return False
        return True

    def name(self) -> str:
        return MODE_ROBOT if self.active() else MODE_IDLE

    def remaining_s(self) -> int:
        if not self.active():
            return 0
        return max(0, int(self._expires_at - self._clock()))

    # ---- 变更 -------------------------------------------------------------
    def enter(self, ttl_s: int = DEFAULT_TTL_S) -> int:
        if ttl_s <= 0:
            raise ValueError("ttl_seconds 必须为正数")
        now = self._clock()
        self._entered_at = now
        self._expires_at = now + ttl_s
        return ttl_s

    def exit(self) -> bool:
        was = self.active()
        self._entered_at = None
        self._expires_at = None
        return was

    # ---- 快照 -------------------------------------------------------------
    def snapshot(self) -> dict:
        return {
            "mode": self.name(),
            "active": self.active(),
            "remaining_s": self.remaining_s(),
            "hint": HINT_ENTER if self.active() else HINT_IDLE,
        }


_STATE = RobotModeState()


def state() -> RobotModeState:
    """进程级单例（server 进程 = 一个会话）。"""
    return _STATE
