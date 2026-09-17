"""随机摆放验证：20-30 次单目标采样的通过率与感知误差分布。

口径（用户已确认）：
- 每次采样独立记一次结果；"采样成功且抓取成功"记为 pass，
  任何环节失败记为 fail 并按类别归因，最终给出通过率 + 感知误差分布
  （均值/标准差/最大/最小）；
- 采样阶段不做可达性预筛：退化构型也是合法的一次采样结果，
  由下游门禁判定并记录（单列 IK_DEGENERATE），以保证"通过率"是真实通过率；
- 误差定义为"感知解算位置 vs 仿真真值"，这是需求"验证相机与机械臂末端
  位置计算"的直接量化。真值仅在验收脚本中用于比对（AGENTS.md 铁律 13）。

复用 scripts/run_piper_random_pick.py 的单次执行路径，保证验证口径与
"启动即随机抓取"入口完全一致，避免两套逻辑各自漂移。
"""

import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from build_piper_baseline import _resolve, load_baseline  # noqa: E402
from iraf_core.randomization import (  # noqa: E402
    RandomizationSpec,
    UniformDiscSampler,
    classify_rejection,
    describe_distribution,
    rejection_counts,
    write_report,
)
from run_piper_random_pick import _run_once  # noqa: E402


#: joint3 接近 0 时臂近似伸直，雅可比秩亏导致末端可达性骤降。
#: 实测 25 次采样中 10 次失败样本的 |joint3| 落在 0.032..0.203，
#: 而通过样本均不在此区间，因此把该构型特征单独识别为退化类。
DEGENERATE_JOINT3_THRESHOLD_RAD = 0.25


def _degenerate_joint3(reason):
    """从夹爪对齐失败信息里解析 joint3，判断是否落在退化区。

    只在已经判定失败的前提下使用，用于把"对齐不达标"进一步细分为
    "关节空间退化"与"其他对齐问题"，以便报告能给出可行动的归因。
    """
    import re

    match = re.search(r"'joint3':\s*(-?[0-9.eE+-]+)", str(reason or ""))
    if match is None:
        return None
    value = abs(float(match.group(1)))
    return value if value <= DEGENERATE_JOINT3_THRESHOLD_RAD else None


def _classify_execution_failure(outcome):
    """把执行失败细分为稳定类别：退化构型优先于泛化的对齐不达标。"""
    reason = outcome.get("execution_reason")
    base = classify_rejection(reason)
    if base == "ALIGNMENT_TOLERANCE":
        joint3 = _degenerate_joint3(reason)
        if joint3 is not None:
            return "IK_DEGENERATE", joint3
    return base, None


def _truth_from_scene(scene_path):
    """读取场景真值（仅用于本脚本的比对，不进入任何感知链路）。"""
    import mujoco

    model = mujoco.MjModel.from_xml_path(str(scene_path))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    truth = {}
    report_path = Path(scene_path).with_suffix(".json")
    if not report_path.is_file():
        return truth
    report = json.loads(report_path.read_text(encoding="utf-8"))
    for item in report.get("targets") or []:
        name = str(item["id"])
        index = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
        if index < 0:
            continue
        truth[name] = {
            "position": np.asarray(data.xpos[index], dtype=float).copy(),
            "rotation": np.asarray(data.xmat[index], dtype=float).reshape(3, 3).copy(),
        }
    return truth


def _quaternion_angle_deg(first, second, symmetry_fold=4):
    """两个四元数夹角（度），考虑正方体绕法向的 90° 对称等价类。"""
    import math

    a = np.asarray(first, dtype=float)
    b = np.asarray(second, dtype=float)
    a = a / float(np.linalg.norm(a))
    b = b / float(np.linalg.norm(b))
    folds = max(1, int(symmetry_fold))
    best = None
    for index in range(folds):
        angle = 2.0 * math.pi * index / folds
        half = angle / 2.0
        twist = np.array([math.cos(half), 0.0, 0.0, math.sin(half)], dtype=float)
        candidate_w = (
            twist[0] * b[0] - twist[1] * b[1] - twist[2] * b[2] - twist[3] * b[3]
        )
        candidate = np.array(
            [
                twist[0] * b[0] - twist[1] * b[1] - twist[2] * b[2] - twist[3] * b[3],
                twist[0] * b[1] + twist[1] * b[0] + twist[2] * b[3] - twist[3] * b[2],
                twist[0] * b[2] - twist[1] * b[3] + twist[2] * b[0] + twist[3] * b[1],
                twist[0] * b[3] + twist[1] * b[2] - twist[2] * b[1] + twist[3] * b[0],
            ],
            dtype=float,
        )
        candidate = candidate / float(np.linalg.norm(candidate))
        dot = abs(float(np.clip(np.dot(a, candidate), -1.0, 1.0)))
        value = math.degrees(2.0 * math.acos(dot))
        if best is None or value < best:
            best = value
    return float(best)


def _matrix_to_quaternion(matrix):
    import math

    m = np.asarray(matrix, dtype=float).reshape(3, 3)
    trace = float(np.trace(m))
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        return np.array(
            [
                0.25 * s,
                (m[2, 1] - m[1, 2]) / s,
                (m[0, 2] - m[2, 0]) / s,
                (m[1, 0] - m[0, 1]) / s,
            ]
        )
    if m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
        return np.array(
            [
                (m[2, 1] - m[1, 2]) / s,
                0.25 * s,
                (m[0, 1] + m[1, 0]) / s,
                (m[0, 2] + m[2, 0]) / s,
            ]
        )
    if m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
        return np.array(
            [
                (m[0, 2] - m[2, 0]) / s,
                (m[0, 1] + m[1, 0]) / s,
                0.25 * s,
                (m[1, 2] + m[2, 1]) / s,
            ]
        )
    s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
    return np.array(
        [
            (m[1, 0] - m[0, 1]) / s,
            (m[0, 2] + m[2, 0]) / s,
            (m[1, 2] + m[2, 1]) / s,
            0.25 * s,
        ]
    )


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--baseline", type=Path, default=Path("config/piper_simulation_baseline.yaml")
    )
    parser.add_argument(
        "--scene", type=Path, default=Path("build/models/piper-random-scene.xml")
    )
    parser.add_argument(
        "--output", type=Path, default=Path("build/acceptance/piper-random-sweep")
    )
    parser.add_argument(
        "--vision-file",
        type=Path,
        default=Path("build/calibration/piper-random-vision.json"),
    )
    parser.add_argument("--samples", type=int, default=25, help="采样轮数，建议 20-30")
    parser.add_argument("--seed", type=int, default=20260916)
    parser.add_argument("--duration-ms", type=int, default=12000)
    parser.add_argument("--root", type=Path, default=Path(__file__).resolve().parents[1])
    args = parser.parse_args(argv)
    if not 1 <= args.duration_ms <= 30000:
        parser.error("duration-ms 必须在 1..30000 之间")
    if args.samples < 1:
        parser.error("samples 必须为正整数")

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
    tolerance = float((baseline.get("acceptance") or {}).get("pose_tolerance_m", 0.005))
    max_orientation = float(
        (baseline.get("acceptance") or {}).get("max_orientation_error_deg", 10.0)
    )

    # 每轮使用独立子种子，既保证整批可复现，也保证轮与轮之间互不相关。
    base_seed = int(args.seed)
    records = []
    for index in range(args.samples):
        round_seed = base_seed + index
        sampler = UniformDiscSampler(spec, round_seed)
        accepted_sample = None
        failures = []
        outcome = None
        for attempt in range(1, spec.max_attempts + 1):
            sample = sampler.draw(nominal_xy, top_z, half_size, index, attempt)
            result = _run_once(
                root,
                baseline,
                args,
                sample,
                args.scene.resolve(),
                args.vision_file.resolve(),
            )
            if result["rejected"]:
                failures.append(
                    {
                        "attempt": attempt,
                        "stage": result["stage"],
                        "category": classify_rejection(result.get("reason")),
                        "reason": result.get("reason"),
                    }
                )
                continue
            accepted_sample = sample
            outcome = result
            break

        if outcome is None:
            records.append(
                {
                    "index": index,
                    "round_seed": round_seed,
                    "passed": False,
                    "failure_category": "SCENE_REJECTED",
                    "failures": failures,
                    "execution_status": None,
                }
            )
            print("ROUND %d FAIL scene_rejected attempts=%d" % (index, len(failures)), flush=True)
            continue

        truth_position = None
        truth_rotation = None
        try:
            truth = _truth_from_scene(args.scene.resolve())
            entry = truth.get(str(outcome["target_id"])) or {}
            truth_position = entry.get("position")
            truth_rotation = entry.get("rotation")
        except Exception as exc:  # noqa: BLE001
            truth = {}
            failures.append(
                {
                    "attempt": accepted_sample.attempt,
                    "stage": "truth_read",
                    "category": "TRUTH_UNAVAILABLE",
                    "reason": str(exc),
                }
            )

        detected_position = outcome.get("detected_position_m")
        detected_quaternion = outcome.get("detected_quaternion_wxyz")
        position_error = None
        orientation_error = None
        if truth_position is not None and detected_position is not None:
            position_error = float(
                np.linalg.norm(
                    np.asarray(detected_position, dtype=float)
                    - np.asarray(truth_position, dtype=float)
                )
            )
        if truth_rotation is not None and detected_quaternion is not None:
            orientation_error = _quaternion_angle_deg(
                _matrix_to_quaternion(truth_rotation), detected_quaternion, 4
            )

        execution_ok = outcome.get("execution_status") == "SUCCEEDED"
        # 通过判据：抓取链路成功 + 感知位置误差在门禁内（姿态同理）。
        # 任一项不满足即记为 fail，并给出具体归因，避免"只看执行状态"漏判。
        error_ok = position_error is not None and position_error <= tolerance
        orientation_ok = orientation_error is not None and orientation_error <= max_orientation
        passed = bool(execution_ok and error_ok and orientation_ok)
        degenerate_joint3 = None
        if passed:
            category = ""
        elif not execution_ok:
            category, degenerate_joint3 = _classify_execution_failure(outcome)
        elif not error_ok:
            category = "PERCEPTION_POSITION_ERROR"
        else:
            category = "PERCEPTION_ORIENTATION_ERROR"

        records.append(
            {
                "index": index,
                "round_seed": round_seed,
                "passed": passed,
                "failure_category": category,
                "degenerate_joint3_rad": (
                    round(degenerate_joint3, 9) if degenerate_joint3 is not None else None
                ),
                "failures": failures,
                "sample": accepted_sample.to_dict(),
                "resample_count": accepted_sample.attempt - 1,
                "execution_status": outcome.get("execution_status"),
                "execution_reason": outcome.get("execution_reason"),
                "grasp_mode": outcome.get("grasp_mode"),
                "detected_position_m": detected_position,
                "detected_quaternion_wxyz": detected_quaternion,
                "truth_position_m": (
                    [round(float(v), 9) for v in truth_position]
                    if truth_position is not None
                    else None
                ),
                "position_error_m": (
                    round(position_error, 9) if position_error is not None else None
                ),
                "orientation_error_deg": (
                    round(orientation_error, 6) if orientation_error is not None else None
                ),
                "position_error_ok": bool(error_ok),
                "orientation_error_ok": bool(orientation_ok),
                "center_distance_m": outcome.get("center_distance_m"),
                "lift_delta_m": outcome.get("lift_delta_m"),
                "vision_residual_m": outcome.get("vision_residual_m"),
                "left_normal_force_n": outcome.get("left_normal_force_n"),
                "right_normal_force_n": outcome.get("right_normal_force_n"),
            }
        )
        print(
            "ROUND %d passed=%s pos_err=%s ori_err=%s resample=%d status=%s"
            % (
                index,
                passed,
                round(position_error, 6) if position_error is not None else None,
                round(orientation_error, 3) if orientation_error is not None else None,
                accepted_sample.attempt - 1,
                outcome.get("execution_status"),
            ),
            flush=True,
        )

    passed_records = [item for item in records if item["passed"]]
    passed_count = len(passed_records)
    total = len(records)
    # 误差分布只统计"通过全部门禁"的样本，避免把失败样本的
    # 异常值混入分布而让统计量失去可解释性；失败原因单独列表分解。
    position_distribution = describe_distribution(
        [item["position_error_m"] for item in passed_records]
    )
    orientation_distribution = describe_distribution(
        [item["orientation_error_deg"] for item in passed_records]
    )
    failure_breakdown = rejection_counts(records, key="failure_category")
    inner_failure_breakdown = {}
    for item in records:
        for failure in item["failures"]:
            label = failure["category"]
            inner_failure_breakdown[label] = inner_failure_breakdown.get(label, 0) + 1

    # 采样半径 → 通过率的相关性：用于判断"随机范围是否已超出可达工作区"。
    # 这是本轮实测最关键的结论来源（半径 <=20mm 时 100%，>20mm 后骤降），
    # 因此固化进报告，避免该结论只存在于一次性分析脚本里。
    radius_bins = [(0.0, 0.01), (0.01, 0.02), (0.02, 0.03), (0.03, 0.04), (0.04, 0.050001)]
    radius_correlation = []
    for low, high in radius_bins:
        bucket = [
            item
            for item in records
            if (item.get("sample") or {}).get("offset_m") is not None
            and low <= (item["sample"]["offset_m"]) < high
        ]
        if not bucket:
            continue
        ok = sum(1 for item in bucket if item["passed"])
        radius_correlation.append(
            {
                "offset_min_m": low,
                "offset_max_m": high,
                "samples": len(bucket),
                "passed": ok,
                "pass_rate": round(ok / len(bucket), 6),
            }
        )
    # 侧向偏置统计：实测失败样本全部落在 y > 0，需在报告中显式暴露。
    passed_y = [
        (item.get("sample") or {}).get("y_m")
        for item in passed_records
        if (item.get("sample") or {}).get("y_m") is not None
    ]
    failed_y = [
        (item.get("sample") or {}).get("y_m")
        for item in records
        if not item["passed"] and (item.get("sample") or {}).get("y_m") is not None
    ]
    lateral_bias = {
        "passed_y_m": describe_distribution(passed_y),
        "failed_y_m": describe_distribution(failed_y),
        "deferred_joint1_damping_note": (
            "joint1 阻尼 300 远高于其他关节，大转角下稳态跟踪误差可达 2cm 以上"
            "（config/piper_multi_target.yaml 已记录）；+y 侧需更大的 joint1 转角。"
        ),
    }

    # 感知位置误差的实测分布 → 对 5mm 阈值的重新评估建议。
    threshold_note = None
    if position_distribution:
        observed_max = position_distribution["max"]
        threshold_note = {
            "configured_max_position_error_m": tolerance,
            "observed_max_m": observed_max,
            "headroom_m": round(tolerance - observed_max, 9),
            "recommendation": (
                "阈值可维持"
                if observed_max < tolerance * 0.6
                else "实测最大值已逼近阈值，建议复核阈值或排查 joint1 大转角误差"
            ),
        }

    report = {
        "schema_version": "iraf.piper-random-sweep/v1",
        "simulation_only": True,
        "baseline": str(args.baseline),
        "scene": str(args.scene),
        "seed": base_seed,
        "samples": total,
        "randomization": spec.to_dict(),
        "summary": {
            "total": total,
            "passed": passed_count,
            "failed": total - passed_count,
            "pass_rate": round(passed_count / total, 6) if total else None,
            "failure_breakdown": failure_breakdown,
            "inner_failure_breakdown": dict(sorted(inner_failure_breakdown.items())),
        },
        "position_error_m": position_distribution,
        "orientation_error_deg": orientation_distribution,
        "pass_rate_by_offset": radius_correlation,
        "lateral_bias": lateral_bias,
        "max_position_error_threshold_m": threshold_note,
        "rounds": records,
    }
    write_report(args.output / "report.json", report)
    print(
        json.dumps(
            {
                "total": total,
                "passed": passed_count,
                "pass_rate": report["summary"]["pass_rate"],
                "failure_breakdown": failure_breakdown,
                "position_error_m": position_distribution,
                "pass_rate_by_offset": radius_correlation,
                "max_position_error_threshold_m": threshold_note,
            },
            ensure_ascii=True,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
