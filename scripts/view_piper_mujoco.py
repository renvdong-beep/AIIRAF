"""在 Ubuntu X11 图形会话中打开 Piper Viewer 并演示一次 pick_object。"""

import argparse
import json
import subprocess
import sys
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
    vision_file = root / "build/calibration/piper-vision-target.json"
    try:
        subprocess.run([sys.executable, str(Path(__file__).resolve().parent / "detect_piper_target.py"), "--model", str(args.model), "--output", str(vision_file)], check=True, cwd=root)
    except Exception as exc:
        print("VISION_REFRESH_ERROR", exc, flush=True)
    profile = load_robot_profile(root / "profiles/piper_mujoco.yaml")
    safety = load_safety_policy(root / "profiles/safety/simulation_lab.yaml")
    authority = ControlAuthorityManager()
    manipulation = {
        "targets": {"box_01": {"body": "box_01", "pose_tolerance_m": 0.005}},
        "gripper": {
            "left_finger_body": "link7", "right_finger_body": "link8",
            "open_positions": {"joint1": 0, "joint2": 0, "joint3": 0, "joint4": 0, "joint5": 0, "joint6": 0, "joint7": 0.035, "joint8": -0.035},
            "closed_positions": {"joint1": 0, "joint2": 0, "joint3": 0, "joint4": 0, "joint5": 0, "joint6": 0, "joint7": 0, "joint8": 0},
            "home_positions": {"joint1": 0.0, "joint2": 0.65, "joint3": -1.15, "joint4": 0.0, "joint5": 0.35, "joint6": 0.0},
            "approach_positions": {"joint1": -0.055153, "joint2": 0.027323, "joint3": -0.015914, "joint4": -0.015760, "joint5": 0.771044, "joint6": 0.0},
            "grasp_positions": {"joint1": -0.055153, "joint2": 0.027323, "joint3": -0.015914, "joint4": -0.015760, "joint5": 0.771044, "joint6": 0.0},
            "lift_positions": {"joint1": 0, "joint2": 0.2, "joint3": -0.25, "joint4": 0, "joint5": 0, "joint6": 0, "joint7": 0, "joint8": 0},
            "min_lift_delta_m": 0.02, "min_normal_force_n": 0.2,
            "max_force_imbalance_ratio": 4.0,
            "lift_constraint": "box_01_lift_constraint",
            "lift_anchor_body": "grasp_anchor",
        },
    }
    backend = MujocoBackend.from_config({"model_path": str(args.model.resolve()), "manipulation": manipulation, "realtime": True}, profile, authority)
    runtime = SkillRuntime(profile, safety, backend, SkillRegistry().load_directory(root / "skills"), authority, SqliteExecutionStore(":memory:"))
    # MuJoCo 模型的 qpos 可能来自闭合姿态；演示必须从张开夹爪开始，避免初始与目标重叠。
    backend._set_gripper_controls(manipulation["gripper"]["open_positions"])
    target_id = backend._body_id("box_01")
    target = backend.data.xpos[target_id].copy()
    vision_file = root / "build/calibration/piper-vision-target.json"
    vision_source = "scene"
    if vision_file.is_file():
        vision = json.loads(vision_file.read_text())
        observed = vision.get("vision_world_position_m")
        if isinstance(observed, list) and len(observed) == 3:
            target = observed
            vision_source = "camera"
    print("PICK_TARGET_SOURCE", vision_source, flush=True)
    pick_done = threading.Event()
    pick_error = []

    def run_pick():
        try:
            time.sleep(1.0)
            now = int(time.time() * 1000)
            request = {
            "request_id": "viewer-pick", "idempotency_key": "viewer-pick-" + str(now), "correlation_id": "viewer-pick",
            "skill": "pick_object", "skill_version_constraint": "1.0.0",
            "parameters": {"target_id": "box_01", "grasp_pose": {"frame_id": "world", "position": {"x": float(target[0]), "y": float(target[1]), "z": float(target[2])}, "orientation": {"x": 0, "y": 0, "z": 0, "w": 1}}, "duration_ms": args.pick_duration_ms},
            "deadline_unix_ms": now + max(90000, args.pick_duration_ms * 8), "profile_name": profile.name, "profile_version": profile.version, "profile_digest": profile.digest,
            "safety_policy_name": safety.name, "safety_policy_version": safety.version, "safety_policy_digest": safety.digest,
            "resource_id": "piper-mujoco", "controller": "viewer-pick",
        }
            result = runtime.execute(request, AuthenticatedContext("viewer-pick", frozenset({"task.submit", "task.read"}), "local"))
            report = root / "build/acceptance/piper-pick/viewer-result.json"
            report.parent.mkdir(parents=True, exist_ok=True)
            report.write_text(__import__("json").dumps(result, ensure_ascii=True, indent=2) + "\n")
            print(result, flush=True)
        except Exception as exc:
            pick_error.append(f"{type(exc).__name__}: {exc}")
            print("VIEWER_SKILL_ERROR", pick_error[-1], flush=True)
        finally:
            pick_done.set()

    with mujoco.viewer.launch_passive(backend.model, backend.data) as viewer:
        viewer.cam.lookat[:] = [0.06, 0.0, 0.12]
        viewer.cam.distance = 1.05
        viewer.cam.azimuth = 180
        viewer.cam.elevation = -8
        # Home 动作必须在 Viewer 已打开后执行，否则用户看不到起始运动。
        home_positions = {
            "joint1": 0.0, "joint2": 0.65, "joint3": -1.15,
            "joint4": 0.0, "joint5": 0.35, "joint6": 0.0,
        }
        backend._set_controls(home_positions)
        home_deadline = time.monotonic() + 3.0
        while viewer.is_running() and time.monotonic() < home_deadline:
            with backend._data_lock:
                backend.step()
                viewer.sync()
            time.sleep(0.02)
        threading.Thread(target=run_pick, daemon=True).start()
        started = time.monotonic()
        hold_applied = False
        while viewer.is_running():
            # Viewer 和 Runtime 共用同一 MjData；同步时持有 Backend 锁，避免 GLFW
            # 渲染线程与 Skill 物理步进并发访问 MuJoCo 数据导致段错误。
            with backend._data_lock:
                viewer.sync()
                if pick_done.is_set() and not hold_applied:
                    # Runtime 收尾会清零控制量；演示窗口保持最后抓取姿态，避免机械臂在重力下塌回零位。
                    backend._set_controls(manipulation["gripper"]["grasp_positions"])
                    backend._set_gripper_controls(manipulation["gripper"]["closed_positions"])
                    hold_applied = True
            if args.seconds and time.monotonic() - started >= args.seconds:
                break
            time.sleep(0.02)
        pick_done.wait(timeout=5.0)
    print("VIEWER_OK")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
