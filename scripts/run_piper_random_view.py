"""随机摆放 + 可视化抓取：每轮随机生成位置，在 MuJoCo Viewer 中可见地完成抓取。

与 scripts/run_piper_random_pick.py 的区别：
- 后者是无头（headless/EGL）批量入口，用于统计与验收；
- 本脚本面向"看得见"的演示：每轮先用采样器随机一个位置、构建场景并求解参考姿态，
  然后调用与既有 view_piper_mujoco.py 完全相同的显示链路
  （viewer_runner.ViewerSession + run_interactive），
  抓取经 TaskFlow -> Runtime -> Policy -> Provider 完成，不旁路任何安全策略。

显示通道先探测后使用：不可用即显式失败（不静默假装成功）。
"""

import argparse
import json
import os
import sys
import time
from pathlib import Path

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from build_piper_baseline import (  # noqa: E402
    _resolve,
    build_reference_poses,
    load_baseline,
    validate_grasp_pose,
)
from build_piper_pick_scene import build_scene  # noqa: E402
from iraf_core.randomization import (  # noqa: E402
    RandomizationSpec,
    UniformDiscSampler,
    classify_rejection,
    euler_deg_for_yaw,
    write_report,
)


def _build_random_scene(root, baseline, sample, scene_path):
    """按采样点构建场景并求解参考姿态；返回 (scene_report, config, target_id)。"""
    config = json.loads(json.dumps(baseline))
    target_cfg = config.get("target") or {}
    target_id = str(target_cfg.get("id", "box_01"))
    half_size = float(target_cfg.get("half_size_m", 0.025))
    position = [
        round(float(sample.x_m), 9),
        round(float(sample.y_m), 9),
        round(float(sample.z_m), 9),
    ]
    config["targets"] = [
        {
            "id": target_id,
            "pos_m": position,
            "euler_deg": euler_deg_for_yaw(sample.yaw_deg),
            "rgba": target_cfg.get("rgba", "0.82 0.22 0.12 1"),
        }
    ]
    config.setdefault("grasp", {})["finger_center_xy_m"] = [position[0], position[1]]

    reference = build_reference_poses(root, config, target_id=target_id)
    scene_report = build_scene(
        _resolve(root, baseline["model"]["source"]),
        scene_path,
        target_id=target_id,
        half_size=half_size,
        config=config,
        reference=reference,
    )
    # 抓取姿态门禁：与验收链一致，不通过即显式失败（由调用方重采样）。
    scene_report["grasp_pose_validation"] = validate_grasp_pose(
        scene_path, config, reference
    )
    return scene_report, config, target_id, position


def _manipulation_from_scene_report(scene_report, target_id):
    """从场景旁挂报告构造 manipulation 段（与 view_piper_mujoco.py 同源逻辑）。"""
    entry = None
    for item in scene_report.get("targets") or []:
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
        raise ValueError("场景报告缺少夹爪字段 " + str(missing))
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
    return {
        "targets": {
            str(target_id): {
                "body": str(entry.get("body", target_id)),
                "pose_tolerance_m": float(scene_report.get("pose_tolerance_m", 0.005)),
            }
        },
        "gripper": gripper_entry,
    }


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--baseline", type=Path, default=Path("config/piper_simulation_baseline.yaml")
    )
    parser.add_argument(
        "--scene", type=Path, default=Path("build/models/piper-random-view.xml")
    )
    parser.add_argument(
        "--output", type=Path, default=Path("build/acceptance/piper-random-view")
    )
    parser.add_argument(
        "--vision-file",
        type=Path,
        default=Path("build/calibration/piper-random-vision.json"),
    )
    parser.add_argument("--rounds", type=int, default=5, help="演示轮数")
    parser.add_argument("--seed", type=int, default=None, help="随机种子；给定即可复现")
    parser.add_argument("--seconds", type=float, default=8.0, help="每轮窗口保持秒数")
    parser.add_argument("--pick-duration-ms", type=int, default=12000)
    parser.add_argument(
        "--realtime",
        action="store_true",
        help="按真实时序播放完整抓取过程（约 92 秒/轮）；需配合 display_showcase 策略",
    )
    parser.add_argument(
        "--skill",
        default=None,
        help="调用的 skill 名；realtime 时缺省为 display_pick，否则为 visual_pick",
    )
    parser.add_argument("--root", type=Path, default=Path(__file__).resolve().parents[1])
    args = parser.parse_args(argv)
    if args.rounds < 1:
        parser.error("rounds 必须为正整数")
    if args.seconds < 0:
        parser.error("seconds 不能为负数")
    if not 1000 <= args.pick_duration_ms <= 30000:
        parser.error("pick-duration-ms 必须在 1000..30000 之间")

    root = args.root.resolve()
    baseline = load_baseline(_resolve(root, args.baseline))
    target_cfg = baseline.get("target") or {}
    workbench = baseline.get("workbench") or {}
    grasp_cfg = baseline.get("grasp") or {}
    half_size = float(target_cfg.get("half_size_m", 0.025))
    top_z = float(workbench.get("top_z_m", 0.0))
    nominal_xy = grasp_cfg.get("finger_center_xy_m")
    if not nominal_xy or len(nominal_xy) != 2:
        raise ValueError("基线配置缺少 grasp.finger_center_xy_m")
    spec = RandomizationSpec.from_config(baseline, half_size)

    # --- 显示通道探测：不可用即显式失败，不静默降级 ---
    from iraf_adapters.mujoco import viewer_runner

    display_mode, detail, fallback_reason = viewer_runner.resolve_display_mode()
    print("DISPLAY_MODE", display_mode, flush=True)
    print("DISPLAY_DETAIL", detail, flush=True)
    if fallback_reason:
        print("FALLBACK_REASON", fallback_reason, flush=True)
    if display_mode != viewer_runner.DISPLAY_INTERACTIVE:
        print(
            "VIEWER_UNAVAILABLE：当前不是交互式显示通道，无法在屏幕上显示窗口",
            file=sys.stderr,
            flush=True,
        )
        return 2

    base_seed = (
        int(args.seed)
        if args.seed is not None
        else int(time.time() * 1000) % (2**31)
    )
    rounds = []

    # 采样语义必须与无头脚本（verify_piper_random_pick.py）逐位一致，
    # 否则"同一 seed"其实采样到不同几何，两条路径的通过率无法对比。
    # 因此这里同样采用"每轮独立子种子 base_seed + index"，
    # 并把实际使用的子种子写进报告，便于跨路径核对。
    def _round_sampler(round_index):
        return UniformDiscSampler(spec, base_seed + round_index)

    scene_path = args.scene.resolve()
    vision_file = args.vision_file.resolve()

    for index in range(args.rounds):
        # 采样 + 构建场景 + 姿态门禁；不通过则重采样（上限 max_attempts）。
        # 每轮独立 sampler，保证与无头路径同 seed 同几何。
        sampler = _round_sampler(index)
        built = None
        failures = []
        sample = None
        for attempt in range(1, spec.max_attempts + 1):
            sample = sampler.draw(nominal_xy, top_z, half_size, index, attempt)
            try:
                scene_report, config, target_id, position = _build_random_scene(
                    root, baseline, sample, scene_path
                )
            except ValueError as exc:
                failures.append(
                    {
                        "attempt": attempt,
                        "category": classify_rejection(str(exc)),
                        "reason": str(exc),
                    }
                )
                continue
            built = (scene_report, config, target_id, position)
            break

        if built is None:
            print(
                "ROUND %d SCENE_REJECTED attempts=%d" % (index, len(failures)),
                flush=True,
            )
            rounds.append(
                {
                    "index": index,
                    "sample": sample.to_dict() if sample else None,
                    "ok": False,
                    "stage": "scene_rejected",
                    "failures": failures,
                }
            )
            continue

        scene_report, config, target_id, position = built
        # realtime 播放必须用 display_pick（时长上限放宽），否则会被 30s 租约截断。
        skill_name = args.skill or ("display_pick" if args.realtime else "visual_pick")
        print(
            "ROUND %d SAMPLE x=%.4f y=%.4f yaw=%.1f offset=%.4f resample=%d "
            "skill=%s realtime=%s"
            % (
                index,
                sample.x_m,
                sample.y_m,
                sample.yaw_deg,
                sample.offset_m,
                sample.attempt - 1,
                skill_name,
                bool(args.realtime),
            ),
            flush=True,
        )

        # --- 用与既有 viewer 相同的环境与显示链路 ---
        # realtime 由调用方决定：
        # - realtime=True 时物理时间与真实时间对齐（约 92 秒），
        #   这样窗口里能看到完整的 HOME->APPROACH->DESCEND->GRIP->LIFT 过程；
        # - 代价是必须使用放宽时长的显示策略（display_showcase），
        #   否则会被既有 30s 租约截断。
        os.environ["IRAF_BACKEND_CONFIG"] = json.dumps(
            {
                "model_path": str(scene_path),
                "vision": scene_report.get("vision"),
                "manipulation": _manipulation_from_scene_report(scene_report, target_id),
                "realtime": bool(args.realtime),
            }
        )
        os.environ["IRAF_PROFILE"] = str(root / "profiles/piper_mujoco.yaml")
        os.environ["IRAF_SAFETY_POLICY"] = str(
            root
            / (
                "profiles/safety/display_showcase.yaml"
                if args.realtime
                else "profiles/safety/simulation_lab.yaml"
            )
        )
        os.environ["IRAF_SKILL_ROOT"] = str(root / "skills")
        os.environ["IRAF_BACKEND_ENTRYPOINT"] = (
            "iraf_adapters.mujoco.mujoco_backend:MujocoBackend"
        )
        os.environ["IRAF_EVENT_STORE"] = str(root / "build/iraf-random-view.db")

        # 每轮重建 Runtime/Backend。
        # 注意：只清缓存不够——MujocoBackend 若被模块级缓存复用，
        # 后续轮次会带着上一轮的物理末态继续，导致"每轮都接近但都差一点"
        # 的系统性失败（实测 joint3 落在退化带、残差稳定在 6~9mm）。
        # 因此这里显式清掉 backend 模块，强制重新实例化并重新加载 MJCF。
        for name in (
            "iraf_adapters.mujoco.mujoco_backend",
            "iraf_adapters.bootstrap",
        ):
            sys.modules.pop(name, None)
        import importlib

        import iraf_adapters.bootstrap as bootstrap_module

        importlib.reload(bootstrap_module)
        from iraf_core.policy import AuthenticatedContext

        runtime = bootstrap_module.build_runtime_from_env()
        profile = runtime.profile
        backend = runtime.backend
        session = viewer_runner.ViewerSession(
            profile,
            runtime.safety_policy,
            backend,
            runtime,
            AuthenticatedContext(
                "viewer-random-pick", frozenset({"task.submit", "task.read"}), "local"
            ),
        )

        try:
            session.home(args.pick_duration_ms)
            camera = viewer_runner.camera_settings(profile)
            # 显示编排委托给机器人无关的适配器契约：
            # 窗口先开、抓取后启，渲染基于 SnapshotMirror 副本，
            # 因此完整阶段过程可见。换机器人时本脚本无需改动。
            holder, _held = viewer_runner.run_interactive_live(
                session,
                target_id,
                position,
                args.pick_duration_ms,
                "viewer-random-%d" % index,
                camera,
                seconds=args.seconds,
                skill=skill_name,
            )
            error = holder.get("error")
            execution = holder.get("result") or {}
            status = execution.get("status")
            reason = execution.get("reason")
            evidence = (execution.get("result") or {}).get("evidence") or {}
            ok = error is None and status == "SUCCEEDED"
            print(
                "ROUND %d RESULT ok=%s status=%s error=%s reason=%s"
                % (index, ok, status, error, reason),
                flush=True,
            )
            rounds.append(
                {
                    "index": index,
                    "round_seed": base_seed + index,
                    "ok": ok,
                    "sample": sample.to_dict(),
                    "resample_count": sample.attempt - 1,
                    "status": status,
                    "error": error,
                    # 失败原因必须落盘：否则只能看到 FAILED 而无法归因。
                    "reason": reason,
                    "error_code": execution.get("error_code"),
                    "stage": "executed",
                    "failures": failures,
                    "grasp_mode": evidence.get("grasp_mode"),
                    "bilateral_contact": evidence.get("bilateral_contact"),
                    "lift_delta_m": evidence.get("lift_delta_m"),
                    "center_distance_m": (
                        evidence.get("grasp_alignment") or {}
                    ).get("center_distance_m"),
                }
            )
        except Exception as exc:  # noqa: BLE001
            print(
                "ROUND %d RESULT ok=False error=%s: %s"
                % (index, type(exc).__name__, exc),
                flush=True,
            )
            rounds.append(
                {
                    "index": index,
                    "ok": False,
                    "sample": sample.to_dict(),
                    "resample_count": sample.attempt - 1,
                    "stage": "viewer_failed",
                    "error": type(exc).__name__ + ": " + str(exc),
                    "failures": failures,
                }
            )
        finally:
            # 关闭每轮的 backend，释放 GL/物理资源。
            closer = getattr(backend, "close", None)
            if callable(closer):
                try:
                    closer()
                except Exception:  # noqa: BLE001
                    pass

    ok_count = sum(1 for item in rounds if item.get("ok"))
    report = {
        "schema_version": "iraf.piper-random-view/v1",
        "simulation_only": True,
        "display_mode": display_mode,
        "display_detail": detail,
        "base_seed": base_seed,
        "sampling_semantics": "每轮独立子种子 base_seed + index（与无头脚本一致）",
        "rounds_requested": args.rounds,
        "randomization": spec.to_dict(),
        "summary": {
            "total": len(rounds),
            "ok": ok_count,
            "failed": len(rounds) - ok_count,
        },
        "rounds": rounds,
    }
    write_report(args.output / "report.json", report)
    print(
        json.dumps(
            {"base_seed": base_seed, "ok": ok_count, "total": len(rounds)},
            ensure_ascii=True,
        ),
        flush=True,
    )
    return 0 if ok_count == len(rounds) else 1


if __name__ == "__main__":
    raise SystemExit(main())
