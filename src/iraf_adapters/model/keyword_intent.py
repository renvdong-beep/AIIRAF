# -*- coding: utf-8 -*-
"""无 LLM 环境的确定性意图解析（受控词表，仅用于本地/无板卡验收）。

不是通用 NLU：只识别明确列出的机器人 + 动作；未知语句显式 IntentNoMatch，
不猜、不降级到“随便选一个技能”。
"""
from __future__ import annotations

from iraf_adapters.model.openai_intent import IntentNoMatch


class KeywordIntentProvider:
    name = "keyword-intent"
    version = "0.1.0"

    def parse(self, text, skill_contracts):
        raw = str(text or "").strip()
        if not raw:
            raise IntentNoMatch("意图文本为空")
        contracts = skill_contracts if isinstance(skill_contracts, dict) else {}
        allowed = set(contracts)

        is_g1 = any(token in raw for token in ("g1", "G1", "人形", "机器人"))
        if is_g1:
            if any(token in raw for token in ("站起来", "站立", "站起", "stand")):
                return self._intent("stand", {}, allowed)
            if any(token in raw for token in ("走", "前进", "行走", "walk", "locomote")):
                return self._intent("locomote", {
                    "velocity": {"vx_mps": 0.5, "vy_mps": 0.0, "wz_rad_s": 0.0},
                    "duration_ms": 2000,
                }, allowed)
            if any(token in raw for token in ("停", "停止", "stop")):
                return self._intent("stop", {}, allowed)
        if any(token in raw for token in ("franka", "Franka", "panda", "Panda")):
            if any(token in raw for token in ("移动", "动", "伸", "move")):
                positions = {
                    "joint1": 0.2, "joint2": 0.1, "joint3": 0.0, "joint4": -0.6,
                    "joint5": 0.0, "joint6": 0.6, "joint7": 0.0,
                    "rq2f85_right_driver_joint": 0.0, "rq2f85_right_coupler_joint": 0.0,
                    "rq2f85_right_spring_link_joint": 0.0, "rq2f85_right_follower_joint": 0.0,
                    "rq2f85_left_driver_joint": 0.0, "rq2f85_left_coupler_joint": 0.0,
                    "rq2f85_left_spring_link_joint": 0.0, "rq2f85_left_follower_joint": 0.0,
                }
                return self._intent("move_joint", {"positions": positions, "duration_ms": 2000}, allowed)
        if any(token in raw for token in ("停", "停止", "stop")):
            return self._intent("stop", {}, allowed)
        raise IntentNoMatch("关键词模式无法匹配：%r" % raw, matches=sorted(allowed))

    @staticmethod
    def _intent(skill, parameters, allowed):
        if skill not in allowed:
            raise IntentNoMatch("技能 %s 不在当前契约中" % skill, matches=sorted(allowed))
        return {"skill": skill, "parameters": parameters}
