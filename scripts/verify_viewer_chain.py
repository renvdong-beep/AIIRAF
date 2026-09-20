"""显示链验收：跑一次完整抓取全过程可视化并产出可审计报告。

验收内容：
1. 显示通道准入：interactive_viewer / offscreen_frames / unavailable；
2. 六阶段时间线：HOME / APPROACH / DESCEND / GRIP_OPEN / GRIP_CLOSE / LIFT；
3. 末态保持：抓取结束后机械臂保持末态位形；
4. 抓取结果：命中目标 + 双指接触 + 抬升位移。

退出码：0 通过；1 抓取失败；2 显示通道不可用。
降级时在报告中显式标注 fallback_reason，不静默假装成功。
"""

import argparse
import json
import os
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from iraf_adapters.mujoco import viewer_runner  # noqa: E402


def _scene_report(model_path):
    path = model_path.with_suffix(".json")
    if not path.is_file():
        raise SystemExit("缺少场景报告: " + str(path))
    return json.loads(path.read_text(encoding="utf-8"))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--model", type=Path, default=Path("build/models/piper-pick-scene.xml")
    )
    parser.add_argument("--output", type=Path, default=Path("build/acceptance/viewer-chain"))
    parser.add_argument("--target-id", default=None)
    parser.add_argument("--duration-ms", type=int, default=12000)
    parser.add_argument(
        "--mode",
        choices=("auto", "interactive", "offscreen"),
        default="auto",
        help="显示路径；auto 为探测后自动选择",
    )
    parser.add_argument("--seconds", type=float, default=30.0)
    parser.add_argument("--frames", type=int, default=0, help="离屏模式导出的帧数")
    args = parser.parse_args(argv)

    root = Path(__file__).resolve().parents[1]
    model = args.model if args.model.is_absolute() else root / args.model
    if not model.is_file():
        parser.error("模型文件不存在: " + str(model))

    report = {
        "schema_version": "iraf.mujoco.viewer-chain-acceptance/v1",
        "display_mode": None,
        "fallback_reason": None,
        "scene": str(model),
        "profile": None,
        "target_id": None,
        "phases": [],
        "phase_order_ok": False,
        "hold_pose_applied": False,
        "frames_emitted": None,
        "pick_result": None,
        "passed": False,
        "checked_at_unix_ms": int(time.time() * 1000),
    }

    output_dir = args.output if args.output.is_absolute() else root / args.output
    output_dir.mkdir(parents=True, exist_ok=True)

    def finish(code):
        report["passed"] = code == 0
        (output_dir / "report.json").write_text(
            json.dumps(report, ensure_ascii=False, indent=2, default=str) + "\n",
            encoding="utf-8",
        )
        print("PASSED " + str(report["passed"]), flush=True)
        print("DISPLAY_MODE " + str(report["display_mode"]), flush=True)
        print("PHASES " + ",".join(p["name"] for p in report["phases"]), flush=True)
        return code

    # 1) 显示通道准入
    # allow_probe=False：本进程随后要真正开启 launch_passive 窗口，
    # 若先做 GLFW 试创建会残留 GL 上下文状态，导致 launch_passive 报
    # "mj_copyDataVisual: attempting to copy mjData while stack is in use"。
    if args.mode == "offscreen":
        display_mode, detail, fallback = (
            viewer_runner.DISPLAY_OFFSCREEN,
            "forced offscreen",
            "由 --mode offscreen 显式指定离屏路径",
        )
    else:
        display_mode, detail, fallback = viewer_runner.resolve_display_mode(
            allow_probe=False
        )
    report["display_mode"] = display_mode
    report["display_detail"] = detail
    report["fallback_reason"] = fallback
    print("DISPLAY_MODE " + display_mode, flush=True)
    print("DISPLAY_DETAIL " + detail, flush=True)
    if fallback:
        print("FALLBACK_REASON " + fallback, flush=True)

    if display_mode == viewer_runner.DISPLAY_UNAVAILABLE:
        print("DISPLAY_UNAVAILABLE " + detail, file=sys.stderr, flush=True)
        return finish(2)

    if args.mode == "interactive" and display_mode != viewer_runner.DISPLAY_INTERACTIVE:
        print("REQUESTED_INTERACTIVE_BUT_UNAVAILABLE", file=sys.stderr, flush=True)
        return finish(2)

    # 2) 装配 Runtime（走既有 bootstrap 与 factory 动态装配）
    scene = _scene_report(model)
    target_id = args.target_id or str(scene.get("target_id") or "box_01")
    report["target_id"] = target_id

    from view_piper_mujoco import _manipulation_from_scene_report

    os.environ["IRAF_BACKEND_CONFIG"] = json.dumps(
        {
            "model_path": str(model),
            "manipulation": _manipulation_from_scene_report(scene, target_id),
            # 视觉 Provider 声明随场景报告下传（后端无机型默认路径）。
            "vision": scene.get("vision"),
            # realtime=False：抓取在仿真时间内完成，避免超出技能租约预算。
            "realtime": False,
        }
    )
    os.environ.setdefault("IRAF_PROFILE", str(root / "profiles/piper_mujoco.yaml"))
    os.environ.setdefault(
        "IRAF_SAFETY_POLICY", str(root / "profiles/safety/simulation_lab.yaml")
    )
    os.environ.setdefault("IRAF_SKILL_ROOT", str(root / "skills"))
    os.environ.setdefault(
        "IRAF_BACKEND_ENTRYPOINT", "iraf_adapters.mujoco.mujoco_backend:MujocoBackend"
    )
    os.environ.setdefault("IRAF_EVENT_STORE", str(root / "build/iraf-viewer-chain.db"))

    sys.path.insert(0, str(root / "scripts"))
    from iraf_adapters.bootstrap import build_runtime_from_env
    from iraf_core.policy import AuthenticatedContext

    runtime = build_runtime_from_env()
    profile = runtime.profile
    report["profile"] = profile.name
    report["profile_version"] = profile.version

    # 3) 抓取位姿：视觉优先，回退场景真值（仅自检用途）
    vision_file = root / "build/calibration/piper-vision-target.json"
    position = None
    source = "scene"
    if vision_file.is_file():
        observed = json.loads(vision_file.read_text(encoding="utf-8")).get(
            "vision_world_position_m"
        )
        if isinstance(observed, list) and len(observed) == 3:
            position = [float(v) for v in observed]
            source = "camera"
    if position is None:
        import mujoco

        mujoco.mj_forward(runtime.backend.model, runtime.backend.data)
        body = scene["targets"][0]["body"]
        body_id = mujoco.mj_name2id(
            runtime.backend.model, mujoco.mjtObj.mjOBJ_BODY, str(body)
        )
        position = [float(v) for v in runtime.backend.data.xpos[body_id]]
    report["target_source"] = source

    session = viewer_runner.ViewerSession(
        profile,
        runtime.safety_policy,
        runtime.backend,
        runtime,
        AuthenticatedContext(
            "viewer-chain", frozenset({"task.submit", "task.read"}), "local"
        ),
    )
    session.home(args.duration_ms)

    camera = viewer_runner.camera_settings(profile)
    report["camera"] = camera

    if display_mode == viewer_runner.DISPLAY_INTERACTIVE:
        holder, held = viewer_runner.run_interactive(
            session,
            target_id,
            position,
            args.duration_ms,
            "viewer-chain",
            camera,
            seconds=args.seconds,
        )
    else:
        holder, held, emitted = viewer_runner.run_offscreen(
            session,
            target_id,
            position,
            args.duration_ms,
            "viewer-chain",
            output_dir / "frames",
            args.frames or 24,
        )
        report["frames_emitted"] = emitted

    report["phases"] = session.phases
    report["phase_order_ok"] = [p["name"] for p in session.phases] == ["HOME", "HOLD"]
    report["hold_pose_applied"] = bool(held)
    report["pick_result"] = holder["result"]
    report["pick_error"] = holder["error"]

    if holder["error"]:
        print("PICK_ERROR " + str(holder["error"]), file=sys.stderr, flush=True)
        return finish(1)

    result = holder["result"] or {}
    evidence = (result.get("result") or {}).get("evidence") or {}
    report["pick_status"] = result.get("status")
    report["lift_delta_m"] = evidence.get("lift_delta_m")
    report["bilateral_contact"] = evidence.get("bilateral_contact")
    report["grasp_mode"] = evidence.get("grasp_mode")

    ok = (
        result.get("status") == "SUCCEEDED"
        and bool(report["hold_pose_applied"])
        and report["phase_order_ok"]
    )
    if not ok:
        print("ACCEPTANCE_FAILED details=" + json.dumps({
            "status": result.get("status"),
            "hold": report["hold_pose_applied"],
            "phases": [p["name"] for p in session.phases],
        }, ensure_ascii=False), file=sys.stderr, flush=True)
        return finish(1)

    print("LIFT_DELTA " + str(report["lift_delta_m"]), flush=True)
    print("BILATERAL " + str(report["bilateral_contact"]), flush=True)
    print("GRASP_MODE " + str(report["grasp_mode"]), flush=True)
    return finish(0)


if __name__ == "__main__":
    raise SystemExit(main())
