"""在 Ubuntu X11 图形会话中打开 Piper Viewer 并演示一次 pick_object。"""

import argparse
import threading
import time
from pathlib import Path

import mujoco.viewer

from iraf_adapters.mujoco.mujoco_backend import MujocoBackend
from iraf_core.authority import ControlAuthorityManager
from iraf_core.policy import AuthenticatedContext
from iraf_core.profile import load_robot_profile, load_safety_policy
from iraf_core.registry import SkillRegistry
from iraf_core.runtime import SkillRuntime
from iraf_core.store import SqliteExecutionStore


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path, default=Path("build/models/piper-pick-scene.xml"))
    parser.add_argument("--seconds", type=float, default=0, help="0 表示保持窗口")
    parser.add_argument("--pick-duration-ms", type=int, default=12000)
    args = parser.parse_args(argv)
    if not args.model.is_file():
        parser.error("模型文件不存在: " + str(args.model))
    if args.seconds < 0:
        parser.error("seconds 不能为负数")
    if args.pick_duration_ms < 1000 or args.pick_duration_ms > 30000:
        parser.error("pick-duration-ms 必须在 1000..30000 之间")

    root = Path(__file__).resolve().parents[1]
    profile = load_robot_profile(root / "profiles/piper_mujoco.yaml")
    safety = load_safety_policy(root / "profiles/safety/simulation_lab.yaml")
    authority = ControlAuthorityManager()
    manipulation = {
        "targets": {"box_01": {"body": "box_01", "pose_tolerance_m": 0.005}},
        "gripper": {
            "left_finger_body": "link7", "right_finger_body": "link8",
            "open_positions": {"joint1": 0, "joint2": 0, "joint3": 0, "joint4": 0, "joint5": 0, "joint6": 0, "joint7": 0.035, "joint8": -0.035},
            "closed_positions": {"joint1": 0, "joint2": 0, "joint3": 0, "joint4": 0, "joint5": 0, "joint6": 0, "joint7": 0, "joint8": 0},
            "lift_positions": {"joint1": 0, "joint2": 0.4, "joint3": -0.5, "joint4": 0, "joint5": 0, "joint6": 0, "joint7": 0, "joint8": 0},
            "min_lift_delta_m": 0.02, "lift_constraint": "box_01_lift_constraint",
        },
    }
    backend = MujocoBackend.from_config({"model_path": str(args.model.resolve()), "manipulation": manipulation, "realtime": True}, profile, authority)
    runtime = SkillRuntime(profile, safety, backend, SkillRegistry().load_directory(root / "skills"), authority, SqliteExecutionStore(":memory:"))
    target_id = backend._body_id("box_01")
    target = backend.data.xpos[target_id].copy()

    def run_pick():
        time.sleep(1.0)
        now = int(time.time() * 1000)
        request = {
            "request_id": "viewer-pick", "idempotency_key": "viewer-pick-" + str(now), "correlation_id": "viewer-pick",
            "skill": "pick_object", "skill_version_constraint": "1.0.0",
            "parameters": {"target_id": "box_01", "grasp_pose": {"frame_id": "world", "position": {"x": float(target[0]), "y": float(target[1]), "z": float(target[2])}, "orientation": {"x": 0, "y": 0, "z": 0, "w": 1}}, "duration_ms": args.pick_duration_ms},
            "deadline_unix_ms": now + 30000, "profile_name": profile.name, "profile_version": profile.version, "profile_digest": profile.digest,
            "safety_policy_name": safety.name, "safety_policy_version": safety.version, "safety_policy_digest": safety.digest,
            "resource_id": "piper-mujoco", "controller": "viewer-pick",
        }
        print(runtime.execute(request, AuthenticatedContext("viewer-pick", frozenset({"task.submit", "task.read"}), "local")), flush=True)

    with mujoco.viewer.launch_passive(backend.model, backend.data) as viewer:
        viewer.cam.lookat[:] = [0.08, 0.0, 0.22]
        viewer.cam.distance = 0.62
        viewer.cam.azimuth = 180
        viewer.cam.elevation = -18
        threading.Thread(target=run_pick, daemon=True).start()
        started = time.monotonic()
        while viewer.is_running():
            # Viewer 和 Runtime 共用同一 MjData；同步时持有 Backend 锁，避免 GLFW
            # 渲染线程与 Skill 物理步进并发访问 MuJoCo 数据导致段错误。
            with backend._data_lock:
                viewer.sync()
            if args.seconds and time.monotonic() - started >= args.seconds:
                break
            time.sleep(0.02)
    print("VIEWER_OK")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
