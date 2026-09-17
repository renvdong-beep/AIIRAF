"""工程向导 skill 的端到端验证。

验证目标（对应用户"封装为给外部客户使用的 skill"的需求）：
1. 客户在会话中回答问卷 → 经 TaskFlow/Runtime/Policy/Provider 产出工件；
2. 产出的 profile.yaml 能被既有 load_robot_profile 真正加载，
   且结构段与问卷一致——这是"生成物可用"的硬证据，
   而不是"文件写出来了"这种弱证据；
3. 向导不驱动机器人：用探针 Backend 断言没有任何运动方法被调用；
4. 既有文件拒绝覆写（铁律 5），且整体判定为失败。
"""

import json
import shutil
import sys
from pathlib import Path

ROOT = Path("/home/coretek/AIIRAF")
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT / "scripts"))

from iraf_core.authority import ControlAuthorityManager  # noqa: E402
from iraf_core.policy import AuthenticatedContext  # noqa: E402
from iraf_core.profile import load_robot_profile, load_safety_policy  # noqa: E402
from iraf_core.registry import SkillRegistry  # noqa: E402
from iraf_core.runtime import SkillRuntime  # noqa: E402
from iraf_core.store import SqliteExecutionStore  # noqa: E402

FRANKA_ANSWERS = {
    "robot_id": "franka_panda",
    "preset": "franka",
    "joints": [
        "panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4",
        "panda_joint5", "panda_joint6", "panda_joint7",
        "panda_finger_joint1", "panda_finger_joint2",
    ],
    "joint_limits": {
        "panda_joint1": [-2.8973, 2.8973],
        "panda_joint2": [-1.7628, 1.7628],
        "panda_joint3": [-2.8973, 2.8973],
        "panda_joint4": [-3.0718, -0.0698],
        "panda_joint5": [-2.8973, 2.8973],
        "panda_joint6": [-0.0175, 3.7525],
        "panda_joint7": [-2.8973, 2.8973],
        "panda_finger_joint1": [0.0, 0.04],
        "panda_finger_joint2": [0.0, 0.04],
    },
    "arm_joints": [
        "panda_joint1", "panda_joint2", "panda_joint3",
        "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7",
    ],
    "control_frequency_hz": 1000.0,
    "wrist_body": "panda_link8",
    "finger_geoms": ["panda_leftfinger", "panda_rightfinger"],
    "gripper_drive_joints": ["panda_finger_joint1", "panda_finger_joint2"],
    "open_positions": {"panda_finger_joint1": 0.04, "panda_finger_joint2": 0.04},
    "closed_positions": {"panda_finger_joint1": 0.0, "panda_finger_joint2": 0.0},
    "simulation": True,
    "pad_offset_m": 0.0,
    "camera_pos_m": [0.3, -0.7, 0.7],
    "camera_look_at_m": [0.2, 0.0, 0.02],
}


class MotionProbeBackend:
    """探针 Backend：任何运动调用都会让验证失败。"""

    def __init__(self):
        self.motion_calls = []

    @classmethod
    def from_config(cls, config, profile, authority):
        return cls()

    def move_joint(self, positions, duration_ms, lease):
        self.motion_calls.append("move_joint")
        raise AssertionError("向导不得调用 move_joint")

    def pick_object(self, target_id, grasp_pose, duration_ms, lease):
        self.motion_calls.append("pick_object")
        raise AssertionError("向导不得调用 pick_object")

    def visual_pick(self, inputs, lease):
        self.motion_calls.append("visual_pick")
        raise AssertionError("向导不得调用 visual_pick")

    def stop(self, lease):
        self.motion_calls.append("stop")
        raise AssertionError("向导不得调用 stop")

    def step(self, count=1):
        self.motion_calls.append("step")
        raise AssertionError("向导不得调用 step")

    def runtime_inventory(self):
        return {"safety": {"estop": False}, "manipulation": {"target_visible": False}}


def build_runtime(backend):
    # 工程向导使用**独立的构建期 Profile + SafetyPolicy**，而不是改写机器人 Profile：
    # - 该 Profile 只声明 engineering.robot_adapter，不含任何运动能力，
    #   因此这个 Runtime 在策略层就不可能执行任何运动 skill；
    # - 该 SafetyPolicy 只放行 engineering.* 能力。
    # 这样既满足"所有请求必须经 Policy"，又避免把工程能力混进机器人配置。
    profile = load_robot_profile(ROOT / "profiles/engineering_tooling.yaml")
    safety = load_safety_policy(
        ROOT / "profiles/safety/engineering_tooling.yaml"
    )
    authority = ControlAuthorityManager()
    registry = SkillRegistry().load_directory(ROOT / "skills")
    return SkillRuntime(
        profile,
        safety,
        backend,
        registry,
        authority,
        SqliteExecutionStore(":memory:"),
    ), profile, safety


def submit(runtime, profile, safety, parameters, token):
    import time

    now = int(time.time() * 1000)
    request = {
        "request_id": token,
        "idempotency_key": token + "-" + str(now),
        "correlation_id": token,
        "skill": "engineering.robot_adapter",
        "skill_version_constraint": "1.0.0",
        "parameters": parameters,
        "deadline_unix_ms": now + 120000,
        "profile_name": profile.name,
        "profile_version": profile.version,
        "profile_digest": profile.digest,
        "safety_policy_name": safety.name,
        "safety_policy_version": safety.version,
        "safety_policy_digest": safety.digest,
        "resource_id": "piper-mujoco",
        "controller": "robot-adapter-wizard",
    }
    return runtime.execute(
        request,
        AuthenticatedContext(
            "robot-adapter-wizard", frozenset({"task.submit", "task.read"}), "local"
        ),
    )


def main():
    output_dir = Path("/tmp/irafout/wizard-out")
    if output_dir.exists():
        shutil.rmtree(output_dir)

    backend = MotionProbeBackend()
    runtime, profile, safety = build_runtime(backend)

    # --- 用例 1：正常生成工件 ---
    result = submit(
        runtime,
        profile,
        safety,
        {"answers": FRANKA_ANSWERS, "output_dir": str(output_dir)},
        "wizard-happy-path",
    )
    print("execution status:", result.get("status"))
    if result.get("status") != "SUCCEEDED":
        print("REASON:", result.get("reason"))
        return 1
    payload = result["result"]
    print("accepted:", payload["accepted"])
    print("artifacts:", json.dumps(payload["artifacts"], ensure_ascii=False))
    if backend.motion_calls:
        print("FAIL: 向导调用了运动方法:", backend.motion_calls)
        return 1
    print("motion calls: none (correct)")

    # --- 用例 2：生成的 profile 必须能被既有加载器真正加载 ---
    profile_path = output_dir / "franka_panda_profile.yaml"
    generated = load_robot_profile(profile_path)
    print("generated profile name:", generated.name)
    print("generated joints:", list(generated.joints))
    print("generated capabilities:", sorted(generated.capabilities))
    print("generated gripper drive_joints:", generated.gripper["drive_joints"])
    print("generated joint_roles:", generated.joint_roles)
    assert generated.name == "franka_panda", generated.name
    assert len(generated.joints) == 9, generated.joints
    assert set(generated.gripper["drive_joints"]) == {
        "panda_finger_joint1", "panda_finger_joint2",
    }, generated.gripper["drive_joints"]
    assert generated.gripper["open_positions"] == {
        "panda_finger_joint1": 0.04, "panda_finger_joint2": 0.04,
    }, generated.gripper["open_positions"]
    assert generated.gripper["closed_positions"] == {
        "panda_finger_joint1": 0.0, "panda_finger_joint2": 0.0,
    }, generated.gripper["closed_positions"]
    assert generated.joint_roles["panda_joint1"] == "arm"
    assert generated.joint_roles["panda_finger_joint1"] == "gripper_drive_left"
    assert generated.joint_roles["panda_finger_joint2"] == "gripper_drive_right"
    assert generated.camera is not None and generated.camera["lookat_m"] == [0.2, 0.0, 0.02]
    print("PROFILE_LOAD_OK")

    # --- 用例 3：adapter 骨架必须语法有效 ---
    skeleton = output_dir / "franka_panda_backend.py"
    compile(skeleton.read_text(encoding="utf-8"), str(skeleton), "exec")
    text = skeleton.read_text(encoding="utf-8")
    for symbol in ("def move_joint", "def pick_object", "def stop", "def runtime_inventory"):
        assert symbol in text, symbol
    assert "NotImplementedError" in text
    print("ADAPTER_SKELETON_OK")

    # --- 用例 4：既有文件必须拒绝覆写且整体判失败 ---
    result2 = submit(
        runtime,
        profile,
        safety,
        {"answers": FRANKA_ANSWERS, "output_dir": str(output_dir)},
        "wizard-overwrite-guard",
    )
    if result2.get("status") != "SUCCEEDED":
        print("FAIL: 覆写保护场景不应抛异常，应以 accepted=false 表达")
        print("REASON:", result2.get("reason"))
        return 1
    payload2 = result2["result"]
    print("second run accepted:", payload2["accepted"])
    print("blocked count:", len(payload2["blocked_overwrites"]))
    assert payload2["accepted"] is False, "既有文件被覆写却未报失败"
    assert len(payload2["blocked_overwrites"]) >= 3, payload2["blocked_overwrites"]
    print("OVERWRITE_GUARD_OK")

    # --- 用例 5：缺必填项必须显式失败 ---
    bad = dict(FRANKA_ANSWERS)
    bad.pop("joint_limits")
    result3 = submit(
        runtime,
        profile,
        safety,
        {"answers": bad, "output_dir": str(output_dir / "bad")},
        "wizard-missing-field",
    )
    print("missing-field status:", result3.get("status"))
    assert result3.get("status") == "FAILED", "缺必填项竟然成功"
    assert "joint_limits" in str(result3.get("reason")), result3.get("reason")
    print("MISSING_FIELD_GUARD_OK")

    print("WIZARD_ALL_OK")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
