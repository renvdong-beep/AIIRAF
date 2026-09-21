"""重心平移可行性探针（只读）：把 §16 的「静力学点结论」升级为**可行域**。

用途（战役 iraf-24h-2 步骤 02b 的决策包第 3 轮）
----------------------------------------------
B1（ADR-0008 决策 1d）落地后 wave/trot 双复评在同一批 9 项判据上失败，实测根因是
**支撑集按实测接触力判定 ⇒ 自锁**。人类要决策的是**支撑集判定语义**：是否引入
「声明相位 ∧ 实测接触」（摆动窗口内的腿不参与 mg 分摊、不承受力控）。

- §15（`probe_go2_support_classification.py`）量出该语义下的自锁窗口；
- §16（`probe_go2_support_set_feasibility.py`）量出**关键帧中立位形这一个点**上，
  三腿支撑集能否兑现期望力旋量 —— 结论是「刀刃状态」（2 组不可兑现 / 2 组第三腿仅 0.2983 % mg）。
  §16 的 `not_proved` 明确写着「重心转移/落点变化后的位形是否可行（位形改变则需重跑）」。

**本探针补的就是那一条**：把力旋量参考点（躯干体心）在水平面内平移 Δ（四足足迹不动），
沿 Δ 扫描，用**生产环境同一个分配器** `iraf_adapters.unitree.balance.allocate_foot_forces`
给出每个单腿摆动相位下的**可行域**（`clamped_legs` 为空 = 物理约束未被违反），从而回答：

1. 每个相位要把重心平移多远、朝哪个方向，分配器才无截断（最近可行点，网格分辨率受限）；
2. 沿「支撑三角形形心」方向（最均衡点，几何量）的**射线扫描**：首次无截断距离与最小法向力曲线；
3. 四个相位的可行域**交集**是否为空 —— 即在**分配器口径**下（不是几何余量口径，§16.2 已证
   两者不一致）「一个常量重心平移」能否同时服务四个相位。

边界（诚实声明）
----------------
- 只读：不改声明、不跑控制回路、不写仓库（`--json` 只在证据区）。
- **建模口径**：平移的是分配器的力旋量参考点 `center_world`（躯干体心）。分配器把足端合力矩
  钉在参考点上 ⇒ 等价于「重心投影相对四足足迹平移 Δ」（脚不动）。**未**单独建模 `subtree_com`
  与躯干体心的固有偏移（该偏移在 Δ=0 时同样存在于当前生产行为里，见 §16 口径），
  因此本探针的数字是**相对量**，不声称是绝对重心坐标。
- 位形取**关键帧中立位形**（`mj_resetDataKeyframe(0)`）⇒ 静力学结论，不含冲击、摩擦滑移与阻尼；
  三个支撑足的足端位置在该位形下取定（与 §15.3 / §16 同一口径）。
- 结论属于仿真（`simulation=true`）；真机/目标端 `DEFERRED`。
- 本探针**不选路**：只报告「某假设在静力学上是否可行及其可行域」，不实现任何候选机制、不改配置。

用法
----
    /usr/bin/python3 scripts/probe_go2_com_shift_feasibility.py \
        --baseline config/go2_loopback.yaml \
        [--model build/scenes/handoff_lab/handoff_lab.xml] \
        [--half-m 0.08] [--step-m 0.004] [--ray-step-m 0.0005] [--max-ray-m 0.10] \
        [--json build/iraf-24h-2/02b/com-shift-feasibility.json]

退出码
------
    0 成功 / 1 用法错误 / 2 声明非法（缺 gait.legs / balance / robot.profile / trunk_body）
    3 引用完整性失败（模型编译、关键帧、足端几何缺失）/ 4 力分配不可解（显式抛出）
    5 正例对照不成立（四腿支撑集有截断或残差 ≫ 0）⇒ 实验组结论**不得据此判读**
"""

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_ROOT / "src"))
sys.path.insert(0, str(_ROOT / "scripts"))

import mujoco  # noqa: E402

from iraf_adapters.unitree import balance as balance_module  # noqa: E402
from probe_go2_support_set_feasibility import (  # noqa: E402
    ProbeError,
    load_declarations,
    neutral_state,
    resolve_trunk_body,
    summarize,
)

DEFAULT_MODEL = "build/scenes/handoff_lab/handoff_lab.xml"
MIN_STANCE_LEGS_EXPECTED = 3


# --------------------------------------------------------------------------- 纯函数（可单测）

def square_offsets(half_m, step_m):
    """确定性顺序的平移网格（先 x 后 y，**含**原点）：[-half, +half] 上按 step 取样。

    分辨率 = 实际点距（`round` 后可能与请求值相差 ≤ 1e-12），并**显式回报给报告**，
    免得下游把「网格受限」读成「真实阈值」。
    """
    if not (half_m > 0.0):
        raise ProbeError("--half-m 必须为正（当前 %r）" % (half_m,), code=1)
    if not (step_m > 0.0):
        raise ProbeError("--step-m 必须为正（当前 %r）" % (step_m,), code=1)
    count = int(round(float(half_m) / float(step_m)))
    if count < 1:
        raise ProbeError("网格退化：--half-m=%r 小于一个 --step-m=%r" % (half_m, step_m), code=1)
    values = [index * float(step_m) for index in range(-count, count + 1)]
    return [(dx, dy) for dx in values for dy in values], float(step_m)


def scan_region(probe, offsets):
    """对每个平移调用一次 `probe(dx, dy)`，按**三态**归类（可行 / 被截断 / 分配器显式失败）。

    三态而不是两态：分配器抛异常（秩不足、支撑腿数不足）既不是「可行」也不是「被物理约束截断」，
    混进任一侧都会伪造结论。
    """
    rows = []
    for dx, dy in offsets:
        try:
            row = probe(dx, dy)
        except Exception as exc:  # 显式失败也算事实，但单独归类
            rows.append({"offset_m": [float(dx), float(dy)], "allocator_error": str(exc)})
            continue
        rows.append({
            "offset_m": [float(dx), float(dy)],
            "clamped_legs": list(row["clamped_legs"]),
            "min_normal_force_n": row["min_normal_force_n"],
            "min_normal_over_mg": row["min_normal_over_mg"],
            "normal_sum_n": row["normal_sum_n"],
            "residual_force_n": row["residual_force_n"],
        })
    return rows


def region_metrics(rows, resolution_m):
    """可行域度量。判据只取**事实** `clamped_legs` 为空（本探针不引入任何新阈值）。"""
    feasible = [row for row in rows
                if row.get("allocator_error") is None and not row["clamped_legs"]]
    errors = [row for row in rows if row.get("allocator_error") is not None]
    nearest = None
    for row in feasible:
        distance = math.hypot(row["offset_m"][0], row["offset_m"][1])
        if nearest is None or distance < nearest["distance_m"]:
            nearest = {"offset_m": list(row["offset_m"]), "distance_m": float(distance)}
    metrics = {
        "point_count": int(len(rows)),
        "feasible_count": int(len(feasible)),
        "clamped_count": int(len(rows) - len(feasible) - len(errors)),
        "allocator_error_count": int(len(errors)),
        "resolution_m": float(resolution_m),
        "feasible_fraction": (float(len(feasible)) / len(rows)) if rows else None,
        "nearest_feasible": nearest,
        "area_lower_bound_m2": float(len(feasible)) * float(resolution_m) ** 2,
    }
    if nearest is not None:
        northeast = math.degrees(math.atan2(nearest["offset_m"][1], nearest["offset_m"][0]))
        metrics["nearest_feasible"]["direction_deg"] = float(northeast)
    return metrics


def intersect_regions(rows_by_case, resolution_m):
    """多个相位可行域的**交集**（逐点求交，只取事实）。返回交集点、面积与两两交点数。"""
    if not rows_by_case:
        raise ProbeError("交集计算需要至少一个相位", code=1)
    keys = sorted(rows_by_case)
    per_case_lookup = {}
    for key in keys:
        lookup = {}
        for row in rows_by_case[key]:
            dx, dy = row["offset_m"]
            lookup[(round(dx, 12), round(dy, 12))] = (
                row.get("allocator_error") is None and not row["clamped_legs"])
        per_case_lookup[key] = lookup
    common = set(per_case_lookup[keys[0]])
    for key in keys[1:]:
        common &= set(per_case_lookup[key])
    intersection = []
    for dx, dy in sorted(common):
        if all(per_case_lookup[key][(dx, dy)] for key in keys):
            intersection.append([float(dx), float(dy)])
    pairwise = {}
    for index, left in enumerate(keys):
        for right in keys[index + 1:]:
            shared = set()
            for dx, dy in set(per_case_lookup[left]) & set(per_case_lookup[right]):
                if per_case_lookup[left][(dx, dy)] and per_case_lookup[right][(dx, dy)]:
                    shared.add((dx, dy))
            pairwise["%s+%s" % (left, right)] = int(len(shared))
    return {
        "cases": keys,
        "intersection_points": intersection,
        "intersection_point_count": int(len(intersection)),
        "intersection_area_lower_bound_m2": float(len(intersection)) * float(resolution_m) ** 2,
        "pairwise_feasible_point_counts": pairwise,
        "resolution_m": float(resolution_m),
    }


def ray_scan(probe, direction, max_distance_m, step_m):
    """沿单位方向射线扫描：逐点登记（距离、截断腿、最小法向力、% mg）。不做单调性假设。"""
    norm = math.hypot(direction[0], direction[1])
    if norm <= 0.0:
        raise ProbeError("射线方向为零矢量", code=1)
    unit = (direction[0] / norm, direction[1] / norm)
    count = int(round(float(max_distance_m) / float(step_m)))
    samples = []
    for index in range(count + 1):
        distance = index * float(step_m)
        dx, dy = unit[0] * distance, unit[1] * distance
        try:
            row = probe(dx, dy)
            samples.append({
                "distance_m": float(distance),
                "clamped_legs": list(row["clamped_legs"]),
                "min_normal_force_n": row["min_normal_force_n"],
                "min_normal_over_mg": row["min_normal_over_mg"],
            })
        except Exception as exc:
            samples.append({"distance_m": float(distance), "allocator_error": str(exc)})
    return {"unit_direction": [float(unit[0]), float(unit[1])],
            "step_m": float(step_m), "samples": samples}


def first_clamp_free_distance(scan):
    """射线扫描中**首个**无截断且分配器未报错的距离（不存在则 None）。"""
    for sample in scan["samples"]:
        if sample.get("allocator_error") is None and not sample["clamped_legs"]:
            return float(sample["distance_m"])
    return None


def triangle_centroid(points):
    """支撑三角形形心（三点均值）——几何量，仅用作射线方向与「最均衡点」参考，不设阈值。"""
    array = np.asarray(points, dtype=float)
    return array.mean(axis=0)


# --------------------------------------------------------------------------- 入口

def main(argv=None):
    parser = argparse.ArgumentParser(description="四足重心平移可行性探针（只读）")
    parser.add_argument("--baseline", default="config/go2_loopback.yaml")
    parser.add_argument("--model", default=DEFAULT_MODEL)
    parser.add_argument("--profile", default=None)
    parser.add_argument("--half-m", type=float, default=0.08, help="平移网格半径（m）")
    parser.add_argument("--step-m", type=float, default=0.004, help="平移网格步长（m）")
    parser.add_argument("--ray-step-m", type=float, default=0.0005, help="射线扫描步长（m）")
    parser.add_argument("--max-ray-m", type=float, default=0.10, help="射线扫描最大距离（m）")
    parser.add_argument("--json", default=None)
    args = parser.parse_args(argv)

    try:
        offsets, resolution = square_offsets(args.half_m, args.step_m)
    except ProbeError as exc:
        print("[重心平移可行性探针] %s" % exc, file=sys.stderr)
        return exc.code

    try:
        baseline, legs, contact_geoms, params = load_declarations(args.baseline)
        trunk_name, profile_path = resolve_trunk_body(baseline, args.profile)
        model, data, feet, _com, center = neutral_state(
            args.model, legs, contact_geoms, trunk_name
        )
    except ProbeError as exc:
        print("[重心平移可行性探针] %s" % exc, file=sys.stderr)
        return exc.code

    mass = float(model.body_mass.sum())
    # 口径与适配器逐项对齐（与 §16 同一份代码路径）：重力取绝对值、高度目标取声明的
    # `gait.verification.height_target_m`（不是「把当前高度当目标」）。
    gravity = abs(float(model.opt.gravity[2]))
    weight = mass * gravity
    height = float(data.qpos[2])
    gait_verification = (baseline.get("gait") or {}).get("verification") or {}
    if "height_target_m" not in gait_verification:
        print("[重心平移可行性探针] 基线未声明 gait.verification.height_target_m："
              "高度目标无依据（不得用当前高度兜底）", file=sys.stderr)
        return 2
    height_target = float(gait_verification["height_target_m"])
    wrench = balance_module.desired_wrench(
        params,
        mass_kg=mass,
        gravity_mps2=gravity,
        height_m=height,
        height_target_m=height_target,
        vertical_velocity_mps=float(data.qvel[2]),
        roll_rad=0.0,
        pitch_rad=0.0,
        horizontal_velocity_world_mps=np.asarray(data.qvel[0:2], dtype=float),
        angular_velocity_world_rad_s=np.asarray(data.qvel[3:6], dtype=float),
    )

    def make_probe(support):
        def probe(dx, dy):
            shifted = np.array([float(center[0]) + dx, float(center[1]) + dy, float(center[2])],
                               dtype=float)
            stance = {code: np.asarray(feet[code], dtype=float).copy() for code in support}
            allocation = balance_module.allocate_foot_forces(params, wrench, stance, shifted)
            return summarize(allocation, support, mass, gravity)
        return probe

    print("model=%s  keyframe=%s  simulation=true  baseline=%s"
          % (args.model, mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_KEY, 0), args.baseline))
    print("整机质量 = %.6f kg  重力 = %.6f m/s²  mg = %.9f N  min_stance_legs = %d"
          % (mass, gravity, weight, int(params["allocation"]["min_stance_legs"])))
    print("期望力旋量（同 §16 口径）= 力 %s N、力矩 %s N·m"
          % ([round(float(v), 9) for v in wrench["force_n"]],
             [round(float(v), 9) for v in wrench["torque_nm"]]))
    print("平移口径 = 力旋量参考点（躯干体心）在水平面内平移，四足足迹不动 "
          "⇒ 「重心投影相对足迹平移 Δ」（相对量，见 docstring 边界）")
    print("网格 = ±%.3f m / 步长 %.4f m（每相位 %d 点）；射线步长 %.4f m、最大 %.3f m"
          % (args.half_m, resolution, len(offsets), args.ray_step_m, args.max_ray_m))
    if int(params["allocation"]["min_stance_legs"]) != MIN_STANCE_LEGS_EXPECTED:
        print("⚠ 声明 min_stance_legs=%d ≠ %d：三腿相位是否被分配器允许需先复核"
              % (int(params["allocation"]["min_stance_legs"]), MIN_STANCE_LEGS_EXPECTED))

    # ---- 正例对照：四腿支撑集（现状的实测接触判定恒为四腿）
    print("")
    print("【正例对照】四腿支撑集")
    try:
        reference_probe = make_probe(list(legs))
        reference_zero = reference_probe(0.0, 0.0)
    except Exception as exc:
        print("[重心平移可行性探针] 分配器失败: %s" % exc, file=sys.stderr)
        return 4
    reference_ok = (not reference_zero["clamped_legs"]
                    and reference_zero["residual_force_n"] <= 1.0e-9 * weight)
    print("  Δ=0：截断腿 = %s；逐腿法向力 = %s N；残差力 = %.3e N"
          % (reference_zero["clamped_legs"] or "无",
             {c: round(v, 6) for c, v in reference_zero["normal_force_n"].items()},
             reference_zero["residual_force_n"]))
    print("  对照结论：%s"
          % ("成立（无截断且残差 ≈ 0）" if reference_ok
             else "**不成立** ⇒ 本探针或分配器异常，实验组结果不得据此判读"))

    reference_rows = scan_region(reference_probe, offsets)
    reference_metrics = region_metrics(reference_rows, resolution)
    print("  网格：可行 %d/%d 点（%.1f%%）"
          % (reference_metrics["feasible_count"], reference_metrics["point_count"],
             100.0 * (reference_metrics["feasible_fraction"] or 0.0)))

    # ---- 实验组：四个单腿摆动相位
    phases = []
    print("")
    print("【实验组】单腿摆动相位（三腿支撑集）—— 逐相位可行域")
    for code in legs:
        support = [name for name in legs if name != code]
        probe = make_probe(support)
        try:
            zero_row = probe(0.0, 0.0)
        except Exception as exc:
            print("  排除 %-3s：Δ=0 分配器显式失败 → %s" % (code, exc))
            phases.append({"excluded_leg": code, "support_legs": support,
                           "allocator_error": str(exc)})
            continue
        rows = scan_region(probe, offsets)
        metrics = region_metrics(rows, resolution)
        support_points = [feet[name][0:2] for name in support]
        centroid = triangle_centroid(support_points)
        direction = np.array([float(centroid[0]) - float(center[0]),
                              float(centroid[1]) - float(center[1])], dtype=float)
        centroid_distance = float(math.hypot(direction[0], direction[1]))
        scan = ray_scan(probe, direction, args.max_ray_m, args.ray_step_m)
        first_free = first_clamp_free_distance(scan)
        try:
            centroid_row = probe(float(direction[0]), float(direction[1]))
            centroid_summary = {
                "offset_m": [float(direction[0]), float(direction[1])],
                "clamped_legs": list(centroid_row["clamped_legs"]),
                "min_normal_over_mg": centroid_row["min_normal_over_mg"],
                "min_normal_force_n": centroid_row["min_normal_force_n"],
                "normal_force_n": {c: float(v) for c, v in centroid_row["normal_force_n"].items()},
            }
        except Exception as exc:
            centroid_summary = {"offset_m": [float(direction[0]), float(direction[1])],
                               "allocator_error": str(exc)}
        case = {
            "excluded_leg": code,
            "support_legs": support,
            "zero_offset": {
                "clamped_legs": list(zero_row["clamped_legs"]),
                "min_normal_force_n": zero_row["min_normal_force_n"],
                "min_normal_over_mg": zero_row["min_normal_over_mg"],
                "normal_force_n": {c: float(v) for c, v in zero_row["normal_force_n"].items()},
                "residual_force_n": zero_row["residual_force_n"],
            },
            "region": metrics,
            "ray": {"unit_direction": scan["unit_direction"], "step_m": scan["step_m"],
                    "first_clamp_free_distance_m": first_free,
                    "samples": scan["samples"]},
            "support_centroid_offset_m": [float(direction[0]), float(direction[1])],
            "support_centroid_distance_m": centroid_distance,
            "support_centroid": centroid_summary,
        }
        case["rows"] = rows
        phases.append(case)

        print("")
        print("  ■ 排除 %-3s（支撑 %s）" % (code, ",".join(support)))
        print("    Δ=0：截断腿 = %s；最小法向力 = %.9f N（%.4f %% mg）；残差 %.3e N  "
              "← 与 §16 同口径复现"
              % (zero_row["clamped_legs"] or "无", zero_row["min_normal_force_n"],
                 100.0 * (zero_row["min_normal_over_mg"] or 0.0), zero_row["residual_force_n"]))
        print("    网格：可行 %d/%d（%.1f%%）、被截断 %d、分配器报错 %d；分辨率 %.4f m"
              % (metrics["feasible_count"], metrics["point_count"],
                 100.0 * (metrics["feasible_fraction"] or 0.0), metrics["clamped_count"],
                 metrics["allocator_error_count"], metrics["resolution_m"]))
        if metrics["nearest_feasible"]:
            print("    最近可行平移 = (%+.6f, %+.6f) m（距原点 %.6f m = %.2f 个网格步长，方向 %+.2f°）"
                  % (metrics["nearest_feasible"]["offset_m"][0],
                     metrics["nearest_feasible"]["offset_m"][1],
                     metrics["nearest_feasible"]["distance_m"],
                     metrics["nearest_feasible"]["distance_m"] / resolution,
                     metrics["nearest_feasible"]["direction_deg"]))
        else:
            print("    最近可行平移 = 无（本网格范围内该相位**整个不可行**）")
        print("    支撑三角形形心方向 = (%+.6f, %+.6f) m（距原点 %.6f m）"
              % (direction[0], direction[1], centroid_distance))
        print("    射线扫描（步长 %.4f m）：首次无截断距离 = %s"
              % (scan["step_m"],
                 "无（%.3f m 内始终有腿被截断）" % args.max_ray_m
                 if first_free is None else "%.4f m" % first_free))
        curve_points = [sample for sample in scan["samples"]
                        if sample["distance_m"] <= max(args.max_ray_m, centroid_distance) + 1.0e-9]
        step_stride = max(1, int(round(0.005 / scan["step_m"])))
        printed = [sample for index, sample in enumerate(curve_points)
                   if index % step_stride == 0]
        if curve_points and curve_points[-1] not in printed:
            printed.append(curve_points[-1])
        print("    离原点距离 (m) | 截断腿 | 最小法向力 (N) | % mg")
        for sample in printed:
            if sample.get("allocator_error") is not None:
                print("      %12.4f | ERROR  | %s" % (sample["distance_m"], sample["allocator_error"]))
                continue
            print("      %12.4f | %-6s | %13.6f | %.4f"
                  % (sample["distance_m"], ",".join(sample["clamped_legs"]) or "无",
                     sample["min_normal_force_n"],
                     100.0 * (sample["min_normal_over_mg"] or 0.0)))
        if "allocator_error" in centroid_summary:
            print("    形心处：分配器显式失败 → %s" % centroid_summary["allocator_error"])
        else:
            print("    形心处（最均衡点）：截断腿 = %s；最小法向力 = %.6f N（%.4f %% mg）"
                  % (centroid_summary["clamped_legs"] or "无",
                     centroid_summary["min_normal_force_n"],
                     100.0 * (centroid_summary["min_normal_over_mg"] or 0.0)))

    # ---- 四个相位的可行域交集：常量重心平移在**分配器口径**下能否同时服务四相位
    rows_by_case = {}
    for case in phases:
        if "rows" in case:
            rows_by_case[case["excluded_leg"]] = case.pop("rows")
    intersection = intersect_regions(rows_by_case, resolution) if rows_by_case else None

    print("")
    print("【常量重心平移】四个相位可行域的交集（分配器口径，非几何余量口径）")
    if intersection is None:
        print("  无可比对相位（全部相位在分配阶段即失败）⇒ 不做结论")
    else:
        print("  交集点数 = %d（网格 ±%.3f m / %.4f m，共 %d 点/相位）；面积下界 = %.3e m²"
              % (intersection["intersection_point_count"], args.half_m, resolution,
                 len(offsets), intersection["intersection_area_lower_bound_m2"]))
        for pair, count in sorted(intersection["pairwise_feasible_point_counts"].items()):
            print("  两两交点数 %-14s = %d" % (pair, count))
        print("  结论（只陈述实测）：%s"
              % ("交集在网格上为空 ⇒ 本网格分辨率与范围内**不存在**能同时服务四个相位的常量平移"
                 if intersection["intersection_point_count"] == 0
                 else "交集非空（%d 点）⇒ 存在可行常量平移，需进一步在更细网格上复核"
                      % intersection["intersection_point_count"]))

    print("")
    print("说明：本探针只做**静力学可行域**测量，不实现任何候选机制、不改声明/判据/阈值；")
    print("      可行域非空 ≠ 动力学下站得住；不可行 ≠ 整条路线不可行（动力学与摩擦滑移另测）。")

    if args.json:
        out = Path(args.json)
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(json.dumps({
            "simulation": True,
            "probe": "go2_com_shift_feasibility",
            "model": args.model,
            "baseline": args.baseline,
            "profile": profile_path,
            "trunk_body": trunk_name,
            "keyframe": mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_KEY, 0),
            "mass_kg": mass,
            "gravity_mps2": gravity,
            "weight_n": weight,
            "min_stance_legs": int(params["allocation"]["min_stance_legs"]),
            "allocation_limits": {
                "normal_force_floor_n": float(params["allocation"]["normal_force_floor_n"]),
                "max_normal_force_n": float(params["allocation"]["max_normal_force_n"]),
                "max_horizontal_force_n": float(params["allocation"]["max_horizontal_force_n"]),
            },
            "desired_wrench": {"force_n": [float(v) for v in wrench["force_n"]],
                               "torque_nm": [float(v) for v in wrench["torque_nm"]]},
            "reference_point_xy_m": [float(center[0]), float(center[1])],
            "foot_xy_m": {code: [float(feet[code][0]), float(feet[code][1])] for code in legs},
            "grid": {"half_m": float(args.half_m), "step_m": resolution,
                     "points_per_case": int(len(offsets))},
            "ray": {"step_m": float(args.ray_step_m), "max_distance_m": float(args.max_ray_m)},
            "reference_ok": bool(reference_ok),
            "reference_zero_offset": {
                "clamped_legs": list(reference_zero["clamped_legs"]),
                "min_normal_force_n": reference_zero["min_normal_force_n"],
                "residual_force_n": reference_zero["residual_force_n"],
            },
            "reference_region": reference_metrics,
            "phases": phases,
            "constant_shift_intersection": intersection,
            "modeling_assumption": (
                "平移力旋量参考点 center_world（躯干体心），四足足迹不动 ⇒ 等价于重心投影相对"
                "足迹平移 Δ；未单独建模 subtree_com 与躯干体心的固有偏移（相对量）"),
            "probe_scope": "statics_at_neutral_keyframe",
            "not_proved": [
                "动力学下该平移是否真的能站住（本探针只判静力学可行域）",
                "摩擦滑移/冲击/阻尼对可行域边界的影响",
                "落点变化（迈步）后的位形（足迹改变则需重跑）",
            ],
        }, ensure_ascii=False, indent=1), encoding="utf-8")
        print("written %s" % out)

    if not reference_ok:
        print("[重心平移可行性探针] 正例对照不成立 ⇒ 退出码 5，实验组结论不得据此判读",
              file=sys.stderr)
        return 5
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
