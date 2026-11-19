# -*- coding: utf-8 -*-
"""无 LLM 环境的确定性意图解析（受控词表，仅用于本地/无板卡验收）。"""
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

        if any(token in raw for token in ("g1", "G1", "人形", "机器人")):
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
            # GRIPPER_BRANCH
            if any(token in raw for token in ("夹爪", "gripper", "开", "合")):
                action = "open" if any(token in raw for token in ("打开", "张开", "open")) else "close"
                return self._intent("gripper", {"action": action, "duration_ms": 1000}, allowed)
            if any(token in raw for token in ("移动", "动", "伸", "move")):
                positions = {
                    "joint1": 0.2, "joint2": 0.1, "joint3": 0.0, "joint4": -0.6,
                    "joint5": 0.0, "joint6": 0.6, "joint7": 0.0,
                }
                return self._intent("move_joint", {"positions": positions, "duration_ms": 2000}, allowed)
            if any(token in raw for token in ("停", "停止", "stop")):
                return self._intent("stop", {}, allowed)

        # UR5_PICK_BRANCH: 单机 UR5e 抓取（本地 MujocoBackend / scene_report）
        low = raw.lower()
        if 'ur5' in low or low.startswith('ur ') or low.startswith('ur('):
            if any(token in raw for token in ("抓", "抓取", "拿", "pick")):
                return self._intent("pick_object", {
                    "target_id": "box_01",
                    "grasp_pose": {"frame_id": "world", "pose_source": "live_target_body"},
                    "duration_ms": 30000,
                }, allowed)
        if any(token in raw for token in ("停", "停止", "stop")):
            return self._intent("stop", {}, allowed)
        raise IntentNoMatch("关键词模式无法匹配：%r" % raw, matches=sorted(allowed))

    @staticmethod
    def _intent(skill, parameters, allowed):
        if skill not in allowed:
            raise IntentNoMatch("技能 %s 不在当前契约中" % skill, matches=sorted(allowed))
        return {"skill": skill, "parameters": parameters}
