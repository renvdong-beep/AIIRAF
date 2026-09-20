"""启动即随机摆放并抓取：单次可复现的随机抓取入口。

行为（对应用户确认的需求：做成脚本、每次运行随机生成位置并抓取、支持 --seed 复现）：
1. 按 grasp.randomization 声明在圆盘内随机采样一个 xy 位置，yaw 取 90° 对称等价类，
   z 由 台面 + 半边长 推出（贴台面）；
2. 以该采样点为抓取点求解 home/approach/grasp/lift 参考姿态，
   经 build_piper_baseline 的 clearance 配平与 validate_grasp_pose 门禁；
3. 门禁不通过则**记录拒绝原因并重采样**，上限 max_attempts；
   全部耗尽仍不通过则显式失败（AGENTS.md 铁律 5，禁止静默兜底）；
4. 生成场景后经 TaskFlow -> Runtime -> Policy -> Provider 提交 visual_pick，
   与既有验收链完全同一条闭环，不做任何旁路。

输出报告含本次采样几何、门禁记账与执行结果，便于 20-30 次采样聚合统计。
"""

import argparse
import json
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
from iraf_adapters.mujoco.mujoco_backend import MujocoBackend  # noqa: E402
from iraf_core.authority import ControlAuthorityManager  # noqa: E402
from iraf_core.policy import AuthenticatedContext  # noqa: E402
from iraf_core.profile import load_robot_profile, load_safety_policy  # noqa: E402
from iraf_core.randomization import (  # noqa: E402
    RandomizationSpec,
    UniformDiscSampler,
    euler_deg_for_yaw,
    write_report,
)
from iraf_core.registry import SkillRegistry  # noqa: E402
from iraf_core.runtime import SkillRuntime  # noqa: E402
from iraf_core.store import SqliteExecutionStore  # noqa: E402


def _randomized_config(baseline, sample):
    """把一次采样注入基线配置：位置进 targets，yaw 进 euler_deg。"""
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
    # 多目标模式才读 targets，但抓取点(xy)仍需显式声明供参考姿态求解使用。
    config.setdefault("grasp", {})["finger_center_xy_m"] = [position[0], position[1]]
    return config, target_id, half_size, position


def _run_once(root, baseline, args, sample, scene_path, vision_file):
    """对一次采样执行参考姿态求解 + 场景生成 + Runtime 抓取。"""
    config, target_id, half_size, position = _randomized_config(baseline, sample)
    rejection = None
    reference = None
    try:
        reference = build_reference_poses(root, config, target_id=target_id)
    except ValueError as exc:
        return {"rejected": True, "stage": "reference_poses", "reason": str(exc)}

    scene = build_scene(
        _resolve(root, baseline["model"]["source"]),
        scene_path,
        target_id=target_id,
        half_size=half_size,
        config=config,
        reference=reference,
    )
    scene["reference_poses"] = reference

    try:
        scene["grasp_pose_validation"] = validate_grasp_pose(
            scene_path, config, reference
        )
    except ValueError as exc:
        return {"rejected": True, "stage": "validate_grasp_pose", "reason": str(exc)}

    tolerance = float(scene.get("pose_tolerance_m", 0.005))
    profile = load_robot_profile(root / "profiles/piper_mujoco.yaml")
    safety = load_safety_policy(root / "profiles/safety/simulation_lab.yaml")
    authority = ControlAuthorityManager()
    backend = MujocoBackend.from_config(
        {
            "model_path": str(Path(scene_path).resolve()),
            "vision": scene.get("vision"),
            "manipulation": {
                "targets": {target_id: {"body": target_id, "pose_tolerance_m": tolerance}},
                "gripper": scene["gripper"],
            },
        },
        profile,
        authority,
    )
    runtime = SkillRuntime(
        profile,
        safety,
        backend,
        SkillRegistry().load_directory(root / "skills"),
        authority,
        SqliteExecutionStore(":memory:"),
    )
    now = int(time.time() * 1000)
    token = "piper-random-%d-%d" % (sample.index, sample.attempt)
    request = {
        "request_id": token,
        "idempotency_key": token + "-" + str(now),
        "correlation_id": "piper-random-pick",
        "skill": "visual_pick",
        "skill_version_constraint": "1.0.0",
        "parameters": {
            "target_id": target_id,
            "duration_ms": int(args.duration_ms),
            "vision_file": str(vision_file),
        },
        "deadline_unix_ms": now + 120000,
        "profile_name": profile.name,
        "profile_version": profile.version,
        "profile_digest": profile.digest,
        "safety_policy_name": safety.name,
        "safety_policy_version": safety.version,
        "safety_policy_digest": safety.digest,
        "resource_id": "piper-mujoco",
        "controller": "piper-random-pick",
    }
    execution = runtime.execute(
        request,
        AuthenticatedContext(
            "piper-random-pick", frozenset({"task.submit", "task.read"}), "local"
        ),
    )

    evidence = ((execution.get("result") or {}).get("evidence")) or {}
    vision = evidence.get("vision") or {}
    scene_validated = scene.get("grasp_pose_validation")
    return {
        "rejected": False,
        "stage": "executed",
        "target_id": target_id,
        "scene_position_m": position,
        "scene": scene,
        "target_truth_position_m": scene.get("target_position"),
        "grasp_pose_validation": scene_validated,
        "execution_status": execution.get("status"),
        "execution_reason": execution.get("reason"),
        "detected_position_m": vision.get("vision_world_position_m"),
        "detected_quaternion_wxyz": vision.get("vision_quaternion_wxyz"),
        "grasp_mode": evidence.get("grasp_mode"),
        "bilateral_contact": bool(evidence.get("bilateral_contact")),
        "lift_delta_m": evidence.get("lift_delta_m"),
        "left_normal_force_n": evidence.get("left_normal_force_n"),
        "right_normal_force_n": evidence.get("right_normal_force_n"),
        "center_distance_m": (evidence.get("grasp_alignment") or {}).get(
            "center_distance_m"
        ),
        "vision_residual_m": vision.get("vision_residual_m"),
        "rejection": rejection,
    }


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--baseline", type=Path, default=Path("config/piper_simulation_baseline.yaml")
    )
    parser.add_argument(
        "--scene", type=Path, default=Path("build/models/piper-random-scene.xml")
    )
    parser.add_argument(
        "--output", type=Path, default=Path("build/acceptance/piper-random-pick")
    )
    parser.add_argument(
        "--vision-file",
        type=Path,
        default=Path("build/calibration/piper-random-vision.json"),
    )
    parser.add_argument("--seed", type=int, default=None, help="随机种子；给定即可复现")
    parser.add_argument("--duration-ms", type=int, default=12000)
    parser.add_argument("--root", type=Path, default=Path(__file__).resolve().parents[1])
    args = parser.parse_args(argv)
    if not 1 <= args.duration_ms <= 30000:
        parser.error("duration-ms 必须在 1..30000 之间")

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
    # seed 未给定时由系统时间派生，但会写进报告以便事后复现。
    seed = int(args.seed) if args.seed is not None else int(time.time() * 1000) % (2**31)
    sampler = UniformDiscSampler(spec, seed)

    attempts = []
    index = 0
    for attempt in range(1, spec.max_attempts + 1):
        sample = sampler.draw(nominal_xy, top_z, half_size, index, attempt)
        outcome = _run_once(
            root, baseline, args, sample, args.scene.resolve(), args.vision_file.resolve()
        )
        record = {"sample": sample.to_dict(), **{
            key: value
            for key, value in outcome.items()
            if key not in {"scene"}
        }}
        attempts.append(record)
        if not outcome["rejected"]:
            break
        index += 1
    else:
        report = {
            "schema_version": "iraf.piper-random-pick/v1",
            "simulation_only": True,
            "seed": seed,
            "randomization": spec.to_dict(),
            "attempts": attempts,
            "accepted": False,
            "reason": "随机采样在 %d 次内均未通过 IK/抓取姿态门禁" % spec.max_attempts,
        }
        write_report(args.output / "report.json", report)
        print(json.dumps({k: report[k] for k in ("seed", "accepted", "reason")}, ensure_ascii=True))
        return 1

    accepted = attempts[-1]
    report = {
        "schema_version": "iraf.piper-random-pick/v1",
        "simulation_only": True,
        "seed": seed,
        "randomization": spec.to_dict(),
        "nominal_xy_m": [float(nominal_xy[0]), float(nominal_xy[1])],
        "attempts": attempts,
        "accepted": True,
        "accepted_sample": accepted["sample"],
        "resample_count": len(attempts) - 1,
        "scene_path": str(args.scene),
        "vision_file": str(args.vision_file),
        "execution_status": accepted.get("execution_status"),
    }
    write_report(args.output / "report.json", report)
    print(
        json.dumps(
            {
                "seed": seed,
                "accepted": True,
                "resample_count": report["resample_count"],
                "sample": report["accepted_sample"],
                "execution_status": report["execution_status"],
            },
            ensure_ascii=True,
        )
    )
    return 0 if accepted.get("execution_status") == "SUCCEEDED" else 1


if __name__ == "__main__":
    raise SystemExit(main())
