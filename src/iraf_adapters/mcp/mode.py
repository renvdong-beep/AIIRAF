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
              "不要把它拆成平台自己的动作，也不要在平台侧伪造结果。"
              "进入时只做只读盘点（把 skills / intents 报给用户），"
              "**不要提交任何任务做验证** —— 一次抓取会搬走仿真目标、弄脏场景。")
HINT_IDLE = ("当前不在机器人开发模式：如要开发/调试机器人应用，先调用 "
             "iraf_robot_mode(action=\"enter\")，再提交任务。")

#: 机器人开发模式的**分派规则**（给平台读的，不是给用户看的）。
#: 关键：这是 "robot-first"（优先交给 IRAF），不是"无脑转发"——平台仍要自己判断
#: 项目与事务类请求；IRAF 只接管机器人语义，并对其余请求给出显式 IRAF-NOT-ROBOT-TASK。
ROUTING_POLICY = (
    "① 凡可能涉及机器人本体/物理动作的请求（移动、抓取、放置、标定、视觉、停止/急停），"
    "优先调用 iraf_submit_task，由 IRAF Skill Manager 选技能并执行；不要先在平台侧自己解释或执行。"
    "② 若 iraf_submit_task 返回 IRAF-NOT-ROBOT-TASK，说明 IRAF 判定该请求与机器人无关 ⇒ "
    "平台按自己的工具与流程处理；不要硬塞给 IRAF，也不要把失败包装成成功。"
    "③ 平台自身的项目/文件/构建/代码/对话类事务仍由平台判断与处理，IRAF 不接管。"
    "④ 需要核对事实用 iraf_get_task(execution_id) 读事件库，不采信模型自述。"
)


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
            "routing_policy": ROUTING_POLICY,
        }


_STATE = RobotModeState()


def state() -> RobotModeState:
    """进程级单例（server 进程 = 一个会话）。"""
    return _STATE
