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

    # **直接透传场景报告的 gripper 段**，不再维护手工白名单。
    # 手工白名单必然漂移：实测它漏掉 `gravity_feedforward` 后，
    # viewer 里的 UR5e 抓取退化为 16.6mm（丢掉了重力前馈），
    # 而同一配置走统一验收却是 0.000119m —— 同一份报告两条路径行为不一致。
    # 场景报告就是 manipulation 配置的唯一真源，除 targets 外无需裁剪。
    gripper_entry = {str(key): value for key, value in gripper.items()}
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
            "缺少场景报告 " + str(report_path) + "，请先运行 scripts/build_baseline.py"
        )
    return json.loads(report_path.read_text(encoding="utf-8"))


def _declared(baseline, key, label):
    """从基线的 build 段取声明；缺失即显式失败（不隐式假设机型）。"""
    value = (baseline.get("build") or {}).get(key)
    if not value:
        raise SystemExit("基线配置缺少 build.%s（%s）" % (key, label))
    return value


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path, default=None)
    parser.add_argument("--seconds", type=float, default=0.0, help="0 表示保持窗口")
    parser.add_argument("--pick-duration-ms", type=int, default=12000)
    parser.add_argument(
        "--baseline",
        type=Path,
        required=True,
        help="基线配置路径（决定场景 / Profile / 视觉来源，见配置的 build 段）",
    )
    parser.add_argument("--target-id", default=None, help="抓取目标 ID")
    parser.add_argument("--correlation-id", default="viewer-pick")
    parser.add_argument(
        "--live",
        action="store_true",
        help=(
            "实时播放完整抓取过程（窗口先开、抓取后启）。"
            "skill/安全策略/时序取自基线 viewer 段声明，缺声明即显式失败"
        ),
    )
    args = parser.parse_args(argv)

    root = Path(__file__).resolve().parents[1]
    baseline_path = args.baseline if args.baseline.is_absolute() else root / args.baseline
    if not baseline_path.is_file():
        parser.error("基线配置不存在: " + str(baseline_path))
    baseline = yaml.safe_load(baseline_path.read_text(encoding="utf-8")) or {}

    # --live：窗口先开、抓取后启，HOME->APPROACH->DESCEND->GRIP->LIFT 全程可见。
    # 与默认静态路径的区别只在本函数：默认路径受并发约束限制为"先抓完再开窗"，
    # 结果是一帧末态（实测用户只能看到抓取完成后的画面）。
    viewer_cfg = baseline.get("viewer") or {}
    live_skill = ""
    live_policy = ""
    if args.live:
        live_skill = str(viewer_cfg.get("live_skill") or "")
        live_policy = str(viewer_cfg.get("live_safety_policy") or "")
        if not live_skill or not live_policy:
            parser.error(
                "--live 需要基线声明 viewer.live_skill 与 viewer.live_safety_policy: "
                + str(baseline_path)
            )
    model = (args.model or (root / _declared(baseline, "scene", "场景 MJCF"))).resolve()
    if not model.is_file():
        parser.error("模型文件不存在: " + str(model))
    if args.seconds < 0:
        parser.error("seconds 不能为负数")
    if not 1000 <= args.pick_duration_ms <= 30000:
        parser.error("pick-duration-ms 必须在 1000..30000 之间")

    scene_report = _load_scene_report(model)
    target_id = (
        args.target_id
        or str(scene_report.get("target_id") or "")
        or str((baseline.get("target") or {}).get("id", "box_01"))
    )

    # 显示通道先探测后使用：不可用即显式失败，不静默假装成功。
    display_mode, detail, fallback_reason = viewer_runner.resolve_display_mode()
    print("DISPLAY_MODE", display_mode, flush=True)
    print("VIEWER_MODE", "live" if args.live else "static", flush=True)
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
            # 视觉 Provider 声明随场景报告下传：后端已无任何机型默认路径，
            # 未声明时 visual_pick 会要求请求显式给出 vision_file。
            "vision": scene_report.get("vision"),
            # realtime 由基线 viewer 段声明决定：静态路径必须 False
            # （pick_object 在 realtime 下约 92 秒，会超出 30s 租约）；
            # live 路径需要 True，物理时间与真实时间对齐才能看见运动。
            "realtime": bool(args.live and viewer_cfg.get("live_realtime", True)),
        }
    )
    os.environ.setdefault(
        "IRAF_PROFILE",
        str(root / _declared(baseline, "profile", "RobotProfile 路径")),
    )
    if args.live:
        # 显示策略由基线声明（时长上限放宽），验收口径不受影响：
        # 验收链仍使用 build.safety_policy（simulation_lab，30000ms）。
        os.environ["IRAF_SAFETY_POLICY"] = str(root / live_policy)
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

    # 抓取位姿：优先用**声明**的视觉证据，缺失时回退场景真值（仅自检，不作感知输入）。
    declared_vision = ((scene_report.get("vision") or {}).get("evidence_file"))
    vision_file = (
        (root / declared_vision) if declared_vision else None
    )
    source = "scene"
    if vision_file is not None and vision_file.is_file():
        vision = json.loads(vision_file.read_text(encoding="utf-8"))
        # 兼容两种证据格式：检测器输出的**多目标**（targets[] 按 id 选取）
        # 与早期单目标格式（顶层 vision_world_position_m）。
        # 只读顶层字段会让"检测器已接入"的机型静默退回场景真值（实测如此）。
        observed = None
        for item in vision.get("targets") or []:
            if str(item.get("id")) == str(target_id):
                observed = item.get("vision_world_position_m") or item.get("position_m")
                break
        if observed is None:
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
        if args.live:
            # 全程可见路径：渲染基于 SnapshotMirror 副本，抓取线程并发执行。
            print("VIEWER_LIVE_SKILL", live_skill, flush=True)
            print("VIEWER_LIVE_POLICY", live_policy, flush=True)
            holder, held = viewer_runner.run_interactive_live(
                session,
                target_id,
                position,
                args.pick_duration_ms,
                args.correlation_id,
                camera,
                seconds=args.seconds,
                skill=live_skill,
            )
        else:
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
            root / "build/acceptance/mujoco-view/frames",
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
    # 报告按基线命名：两台机型同时在窗口里跑时共用一个文件名会互相覆盖
    # （实测后完成的那个把先完成的证据冲掉）。
    report_path = (
        root
        / "build/acceptance/mujoco-view"
        / (baseline_path.stem + "-viewer-report.json")
    )
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
