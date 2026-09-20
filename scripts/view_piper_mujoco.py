"""打开 Piper MuJoCo Viewer 并演示一次抓取全过程。

本脚本是薄封装：显示编排全部委托给 iraf_adapters.mujoco.viewer_runner，
抓取参数取自基线配置与 RobotProfile，不再内联任何魔数。

命令行接口保持向后兼容（--model / --seconds / --pick-duration-ms）。
"""

import argparse
import json
import sys
from pathlib import Path

import yaml

from iraf_adapters.bootstrap import build_runtime_from_env
from iraf_adapters.mujoco import viewer_runner
from iraf_core.policy import AuthenticatedContext


def _manipulation_from_scene_report(scene_report, target_id):
    """从场景旁挂报告构造 manipulation 段。

    场景构建器已通过 IK 求出 approach/grasp/lift 位形与 pad_offset_m，
    这里直接复用，避免在显示入口里重算或内联任何魔数。
    """
    targets = scene_report.get("targets") or []
    entry = None
    for item in targets:
        if str(item.get("id")) == str(target_id):
            entry = item
            break
    if entry is None:
        raise ValueError("场景报告中找不到目标: " + str(target_id))

    gripper = scene_report.get("gripper") or {}
    required = (
        # 夹爪几何字段必须随场景报告一起传入后端：后端已不再提供机型默认值
        # （原先缺省指向 piper_left_finger / link6 等 Piper 专有名）。
        "wrist_body",
        "left_finger_body",
        "right_finger_body",
        "left_finger_geom",
        "right_finger_geom",
        "open_positions",
        "closed_positions",
        "approach_positions",
        "grasp_positions",
    )
    missing = [key for key in required if not gripper.get(key)]
    if missing:
        raise ValueError(
            "场景报告缺少夹爪字段 " + str(missing) + "，请重新构建场景"
        )

    gripper_entry = {
        key: gripper[key]
        for key in (
            "wrist_body",
            "left_finger_body",
            "right_finger_body",
            "left_finger_geom",
            "right_finger_geom",
            "open_positions",
            "closed_positions",
            "approach_positions",
            "grasp_positions",
            "lift_positions",
            "home_positions",
            "min_lift_delta_m",
            "min_normal_force_n",
            "max_force_imbalance_ratio",
            "pad_offset_m",
            "pad_offset_axis",
        )
        if gripper.get(key) is not None
    }
    gripper_entry.setdefault("max_tilt_deg", 30.0)

    target_entry = {
        "body": str(entry.get("body", target_id)),
        "pose_tolerance_m": float(scene_report.get("pose_tolerance_m", 0.005)),
    }
    return {"targets": {str(target_id): target_entry}, "gripper": gripper_entry}


def _load_scene_report(model_path):
    """读取场景旁挂报告（与模型同目录、同名 .json）。"""
    report_path = model_path.with_suffix(".json")
    if not report_path.is_file():
        raise ViewerError(
            "缺少场景报告 " + str(report_path) + "，请先运行 scripts/build_piper_baseline.py"
        )
    return json.loads(report_path.read_text(encoding="utf-8"))


def _default_model(root):
    return root / "build/models/piper-pick-scene.xml"


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path, default=None)
    parser.add_argument("--seconds", type=float, default=0.0, help="0 表示保持窗口")
    parser.add_argument("--pick-duration-ms", type=int, default=12000)
    parser.add_argument(
        "--baseline",
        type=Path,
        default=None,
        help="基线配置路径（默认 config/piper_simulation_baseline.yaml）",
    )
    parser.add_argument("--target-id", default=None, help="抓取目标 ID")
    parser.add_argument("--correlation-id", default="viewer-pick")
    args = parser.parse_args(argv)

    root = Path(__file__).resolve().parents[1]
    model = (args.model or _default_model(root)).resolve()
    baseline_path = (args.baseline or root / "config/piper_simulation_baseline.yaml")
    if not model.is_file():
        parser.error("模型文件不存在: " + str(model))
    if not baseline_path.is_file():
        parser.error("基线配置不存在: " + str(baseline_path))
    if args.seconds < 0:
        parser.error("seconds 不能为负数")
    if not 1000 <= args.pick_duration_ms <= 30000:
        parser.error("pick-duration-ms 必须在 1000..30000 之间")

    baseline = yaml.safe_load(baseline_path.read_text(encoding="utf-8")) or {}
    scene_report = _load_scene_report(model)
    target_id = (
        args.target_id
        or str(scene_report.get("target_id") or "")
        or str((baseline.get("target") or {}).get("id", "box_01"))
    )

    # 显示通道先探测后使用：不可用即显式失败，不静默假装成功。
    display_mode, detail, fallback_reason = viewer_runner.resolve_display_mode()
    print("DISPLAY_MODE", display_mode, flush=True)
    print("DISPLAY_DETAIL", detail, flush=True)
    if fallback_reason:
        print("FALLBACK_REASON", fallback_reason, flush=True)
    if display_mode == viewer_runner.DISPLAY_UNAVAILABLE:
        print("VIEWER_UNAVAILABLE", detail, file=sys.stderr, flush=True)
        return 2

    import os

    # realtime=False：抓取在仿真时间内完成（实测约 2 秒，SUCCEEDED）。
    # 若开启 realtime，每步强制等待一个 timestep，pick_object 需约 92 秒，
    # 必然超出技能声明的 30 秒租约——这是显示实时性与超时预算的固有冲突，
    # 由显示层通过"抓取快速完成 + 主循环持续渲染回放"解耦，而非放宽后端时序。
    os.environ["IRAF_BACKEND_CONFIG"] = json.dumps(
        {
            "model_path": str(model),
            "manipulation": _manipulation_from_scene_report(scene_report, target_id),
            "realtime": False,
        }
    )
    os.environ.setdefault(
        "IRAF_PROFILE", str(root / "profiles/piper_mujoco.yaml")
    )
    os.environ.setdefault(
        "IRAF_SAFETY_POLICY", str(root / "profiles/safety/simulation_lab.yaml")
    )
    os.environ.setdefault("IRAF_SKILL_ROOT", str(root / "skills"))
    os.environ.setdefault(
        "IRAF_BACKEND_ENTRYPOINT", "iraf_adapters.mujoco.mujoco_backend:MujocoBackend"
    )
    os.environ.setdefault(
        "IRAF_EVENT_STORE", str(root / "build/iraf-viewer.db")
    )

    runtime = build_runtime_from_env()
    profile = runtime.profile
    backend = runtime.backend

    # 抓取位姿：优先用视觉坐标，缺失时回退场景真值（仅自检，不作为感知输入）。
    vision_file = root / "build/calibration/piper-vision-target.json"
    source = "scene"
    if vision_file.is_file():
        vision = json.loads(vision_file.read_text(encoding="utf-8"))
        observed = vision.get("vision_world_position_m")
        if isinstance(observed, list) and len(observed) == 3:
            position = [float(v) for v in observed]
            source = "camera"
        else:
            position = None
    else:
        position = None
    if position is None:
        body_name = target_id
        for item in scene_report.get("targets") or []:
            if str(item.get("id")) == str(target_id):
                body_name = str(item.get("body", target_id))
                break
        position = _scene_target_position(backend, body_name)
    print("PICK_TARGET_SOURCE", source, flush=True)

    session = viewer_runner.ViewerSession(
        profile,
        runtime.safety_policy,
        backend,
        runtime,
        AuthenticatedContext(
            "viewer-pick", frozenset({"task.submit", "task.read"}), "local"
        ),
    )
    session.home(args.pick_duration_ms)
    camera = viewer_runner.camera_settings(profile)

    if display_mode == viewer_runner.DISPLAY_INTERACTIVE:
        holder, held = viewer_runner.run_interactive(
            session,
            target_id,
            position,
            args.pick_duration_ms,
            args.correlation_id,
            camera,
            seconds=args.seconds,
        )
    else:
        holder, held, _ = viewer_runner.run_offscreen(
            session,
            target_id,
            position,
            args.pick_duration_ms,
            args.correlation_id,
            root / "build/acceptance/piper-mujoco-view/frames",
            24,
        )

    report = {
        "schema_version": viewer_runner.REPORT_SCHEMA,
        "display_mode": display_mode,
        "display_detail": detail,
        "fallback_reason": fallback_reason,
        "target_id": target_id,
        "target_source": source,
        "camera": camera,
        "phases": session.phases,
        "hold_pose_applied": bool(held),
        "pick_result": holder["result"],
        "pick_error": holder["error"],
    }
    report_path = root / "build/acceptance/piper-mujoco-view/viewer-report.json"
    report_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.write_text(
        json.dumps(report, ensure_ascii=False, indent=2, default=str) + "\n",
        encoding="utf-8",
    )

    if holder["error"]:
        print("VIEWER_SKILL_ERROR", holder["error"], flush=True)
        return 1
    print("VIEWER_OK", flush=True)
    return 0


def _scene_target_position(backend, body_name):
    """回退路径：从场景取目标当前位置（显示自检兜底，非感知输入）。

    只使用后端的公开属性 model/data 与 MuJoCo 公开 API，不触碰私有成员；
    真值仅在验收脚本中用于比对。
    """
    import mujoco

    mujoco.mj_forward(backend.model, backend.data)
    body_id = mujoco.mj_name2id(
        backend.model, mujoco.mjtObj.mjOBJ_BODY, str(body_name)
    )
    if body_id < 0:
        raise ValueError("场景中找不到目标 body: " + str(body_name))
    return [float(v) for v in backend.data.xpos[body_id]]


if __name__ == "__main__":
    raise SystemExit(main())
