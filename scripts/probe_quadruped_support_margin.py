"""静态稳定余量探针：量出「抬单腿时的支撑三角形」对整机重心投影的有符号余量。

用途（战役 iraf-24h-2 步骤 02 的根因取证）
------------------------------------------
准静态步态（wave/creep）的静态稳定前提是「重心投影落在支撑多边形内部」。
本探针把它变成数字：从**编译后的模型**与基线声明（`gait.legs.*.contact_geom`）出发，
①量四足接触点、②量整机重心投影、③对「抬任意一条腿」给出支撑三角形的最小有符号余量
（>0 = 重心在三角形内）、④给出把余量提到目标值所需的重心平移矢量、
⑤穷举**常量**重心平移，判定「四个单支撑相位的余量同时 ≥ 0」的可行域是否为**空集**。

第 ⑤ 项的意义：修法「给足端目标加常量平移偏移」（含 `gait.stabilization` 速度阻尼、相位锁定平移）
失败 7/7，但那是**经验**结论；若四个相位所需的平移两两互斥，则**常量平移这一形式本身**不可行，
结论从「没调好」升级为「形式上不可能」，也说明要走 A 路线必须让站姿**逐相位变化**（迈步式），
而不是加一个常量偏置。

实测（2026-09-21，Go2 + handoff_lab 场景）：中立站姿四足接触点构成 0.3868 × 0.2840 m 的矩形，
重心投影距支撑三角形最紧边的余量仅 ±0.000227483 m（抬 RL/RR 时为**负**）——注意这是**几何事实**：
**矩形站姿的对角线恒过矩形中心**，所以抬任意一腿后支撑三角形必有一条边穿过重心投影，
余量与站位宽度无关。因此「≥3 条腿支撑 ⇒ 静态稳定」在这类站姿下**恒不成立**，
必须靠重心平移（重心进入三角形内部并留出余量）才谈得上静态稳定。

进一步（同一份输出，第 ⑤ 项）：一对**对角腿**所需的平移恰好反向（夹角 180.00°），
且「两相位余量之和」的上确界实测 = +0.000000000 m；穷举 ±0.10 m（10201 点）后四相位同时可得的
最优余量 = −0.000227483 m ⇒ **任何常量重心平移都不可能让四个单支撑相位同时余量为正**。
所以「给足端目标加常量平移偏移」这一族修法（`gait.stabilization` 速度阻尼、相位锁定平移）
不是"没调好"，而是**形式上不可行**；要嘛让站姿逐相位变化（迈步式重新落点），要嘛引入反馈平衡器。

边界
----
- 本脚本**只读**：不写仓库、不改声明、不跑控制回路；结论属于仿真（`simulation=true`）。
- 余量按**球-平面接触点**（足端球心在地面的投影）近似；未计足端滑动与摩擦锥。
- 抬腿后其余三足的位置取**中立位形**下的位置（未考虑机身平移/转动后的重算）——余量因此是
  「命令抬腿瞬间」的量，这一点在报告里显式标注。

用法
----
    /usr/bin/python3 scripts/probe_quadruped_support_margin.py \\
        --baseline config/go2_loopback.yaml \\
        --model build/scenes/handoff_lab/handoff_lab.xml \\
        [--target-margin-m 0.01] [--shift-span-m 0.10] [--shift-step-m 0.002] \\
        [--json build/iraf-24h-2/02/support-margin.json]
"""

import argparse
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

import mujoco  # noqa: E402
import yaml  # noqa: E402

DEFAULT_MODEL = "build/scenes/handoff_lab/handoff_lab.xml"


def edge_margin(point, a, b):
    """点相对有向边 a→b 的有符号距离（取|normal|归一化；符号按内法线统一在调用处处理）。"""
    edge = b - a
    normal = np.array([-edge[1], edge[0]], dtype=float)
    length = float(np.linalg.norm(normal))
    if length <= 0.0:
        raise SystemExit("退化边：%s → %s" % (tuple(a), tuple(b)))
    return float((point - a).dot(normal) / length)


def margins_and_normals(feet, com_xy, legs):
    """纯几何：给定接触点（足端球心投影）与重心投影，返回抬每条腿时的最小有符号余量与内法线。"""
    margins = {}
    normals = {}
    for lifted in legs:
        stance = [feet[name] for name in legs if name != lifted]
        centroid = np.mean(stance, axis=0)
        best = None
        for index in range(len(stance)):
            a = stance[index]
            b = stance[(index + 1) % len(stance)]
            inward = 1.0 if edge_margin(centroid, a, b) > 0.0 else -1.0
            value = edge_margin(com_xy, a, b) * inward
            if best is None or value < best[0]:
                normal = np.array([-(b - a)[1], (b - a)[0]], dtype=float)
                normal = normal / float(np.linalg.norm(normal)) * inward
                best = (value, normal)
        margins[lifted] = float(best[0])
        normals[lifted] = best[1]
    return margins, normals


def required_shifts(margins, normals, target_margin_m):
    """把「抬某条腿」的余量提到目标值所需的重心平移矢量（方向 = 内法线）。"""
    return {code: normals[code] * (target_margin_m - margins[code]) for code in margins}


def support_margins(model, data, mujoco_module, legs, contact_geoms):
    """从编译后的模型取足端接触点与重心投影，再交给纯几何函数。"""
    feet = {}
    for code in legs:
        gid = mujoco_module.mj_name2id(model, mujoco_module.mjtObj.mjOBJ_GEOM, contact_geoms[code])
        if gid < 0:
            raise SystemExit("足端几何 %s 不在模型里" % contact_geoms[code])
        center = np.asarray(data.geom_xpos[gid], dtype=float)
        feet[code] = np.array([float(center[0]), float(center[1])])
    com_xy = np.array([float(data.subtree_com[0][0]), float(data.subtree_com[0][1])])
    margins, normals = margins_and_normals(feet, com_xy, legs)
    return feet, com_xy, margins, normals


def margin_for_shift(feet, com_xy, legs, lifted, offset):
    """给定一个**常量**重心平移 offset 时，抬 lifted 腿的最小有符号余量。

    与 `support_margins` 的差别：这里只接收「平移后的重心位置」，足端几何保持不变
    （常量平移不改变站姿矩形）——正是要用来判定「常量平移这一形式能不能行」。
    """
    stance = [feet[name] for name in legs if name != lifted]
    centroid = np.mean(stance, axis=0)
    point = com_xy + np.asarray(offset, dtype=float)
    best = None
    for index in range(len(stance)):
        a = stance[index]
        b = stance[(index + 1) % len(stance)]
        inward = 1.0 if edge_margin(centroid, a, b) > 0.0 else -1.0
        value = edge_margin(point, a, b) * inward
        if best is None or value < best:
            best = value
    return float(best)


def constant_shift_feasibility(feet, com_xy, legs, span_m, step_m):
    """穷举常量重心平移，判定「四个单支撑相位余量同时 ≥ 0」是否可能。

    之所以要显式做这一步：`support_margins` 给出的「所需平移」在一对**对角腿**上方向相反，
    因此常量平移的可行域可能是**空集**——空集意味着「加常量足端目标偏移」这一族修法
    （速度为阻尼、相位锁定平移）在形式上就不可行，无需再扫参数。
    """
    axis = np.arange(-span_m, span_m + 0.5 * step_m, step_m)
    best = None
    for dx in axis:
        for dy in axis:
            value = min(margin_for_shift(feet, com_xy, legs, code, (dx, dy)) for code in legs)
            if best is None or value > best[0]:
                best = (float(value), (float(dx), float(dy)))
    return {
        "grid_span_m": float(span_m),
        "grid_step_m": float(step_m),
        "grid_points": int(axis.size * axis.size),
        "best_min_margin_m": best[0],
        "best_offset_m": [best[1][0], best[1][1]],
    }


def diagonal_pairs(feet, legs):
    """按几何判出两条对角腿对（不写死腿名）：接触点距离最大的两对即矩形对角线。"""
    pairs = []
    for index, left in enumerate(legs):
        for right in legs[index + 1:]:
            distance = float(np.linalg.norm(feet[left] - feet[right]))
            pairs.append((distance, left, right))
    pairs.sort(reverse=True)
    return [(left, right) for _, left, right in pairs[:2]]


def pair_sum_bound(feet, com_xy, legs, pair, span_m, step_m):
    """对角腿对上的**精确互斥界**：max over 常量平移 (余量_left + 余量_right)。

    推导：一对对角腿共用同一条「对角线」边 D，但两个支撑三角形的内法线**相反**，
    于是对任意平移 d 都有 余量_left(d) ≤ s(com+d, D) 且 余量_right(d) ≤ −s(com+d, D)
    ⇒ 两者之和 ≤ 0 ⇒ min ≤ 0。若实测该上确界确实 ≤ 0，则「四相位余量同时 > 0」**可证明不可行**，
    与穷举结果互为独立佐证（一个是几何界，一个是网格搜索）。
    """
    axis = np.arange(-span_m, span_m + 0.5 * step_m, step_m)
    left, right = pair
    best = None
    for dx in axis:
        for dy in axis:
            total = (margin_for_shift(feet, com_xy, legs, left, (dx, dy))
                     + margin_for_shift(feet, com_xy, legs, right, (dx, dy)))
            if best is None or total > best:
                best = total
    return float(best)


def main(argv=None):
    parser = argparse.ArgumentParser(description="四足静态稳定余量探针（只读）")
    parser.add_argument("--baseline", default="config/go2_loopback.yaml",
                        help="基线声明（只读 gait.legs.*.contact_geom）")
    parser.add_argument("--model", default=DEFAULT_MODEL, help="编译后的 MJCF/XML")
    parser.add_argument("--target-margin-m", type=float, default=0.01,
                        help="目标余量（m），用于折算所需的重心平移矢量")
    parser.add_argument("--shift-span-m", type=float, default=0.10,
                        help="常量平移可行性穷举的半跨度（m），默认 ±0.10")
    parser.add_argument("--shift-step-m", type=float, default=0.002,
                        help="常量平移可行性穷举的步长（m），默认 0.002")
    parser.add_argument("--json", default=None, help="可选的 JSON 证据输出路径")
    args = parser.parse_args(argv)

    baseline = yaml.safe_load(Path(args.baseline).read_text(encoding="utf-8"))
    gait = baseline.get("gait") or {}
    legs_section = gait.get("legs") or {}
    if not legs_section:
        raise SystemExit("基线 %s 未声明 gait.legs：稳定余量无依据" % args.baseline)
    legs = sorted(legs_section)
    contact_geoms = {code: str(legs_section[code]["contact_geom"]) for code in legs}

    model = mujoco.MjModel.from_xml_path(args.model)
    data = mujoco.MjData(model)
    if model.nkey == 0:
        raise SystemExit("模型 %s 没有关键帧：无法取中立位形" % args.model)
    mujoco.mj_resetDataKeyframe(model, data, 0)
    mujoco.mj_forward(model, data)

    feet, com_xy, margins, normals = support_margins(model, data, mujoco, legs, contact_geoms)
    centroid = np.mean([feet[code] for code in legs], axis=0)
    shifts = required_shifts(margins, normals, args.target_margin_m)

    print("model=%s  keyframe=%s  simulation=true" % (
        args.model, mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_KEY, 0)))
    print("整机质量 = %.6f kg" % float(model.body_mass.sum()))
    for code in legs:
        print("  接触点 %-3s x=%+.9f y=%+.9f" % (code, feet[code][0], feet[code][1]))
    print("足端形心 xy = (%+.9f, %+.9f)；整机重心投影 xy = (%+.9f, %+.9f)；偏移 = (%+.9f, %+.9f)" % (
        centroid[0], centroid[1], com_xy[0], com_xy[1],
        com_xy[0] - centroid[0], com_xy[1] - centroid[1]))
    print("站姿矩形：长（x）%.9f m  宽（y）%.9f m（矩形对角线恒过中心 ⇒ 余量与宽度无关）" % (
        max(feet[code][0] for code in legs) - min(feet[code][0] for code in legs),
        max(feet[code][1] for code in legs) - min(feet[code][1] for code in legs)))
    worst = None
    for code in legs:
        print("  抬 %-3s：最小有符号余量 = %+.9f m；提到 %+.6f m 需重心平移 (%+.6f, %+.6f) m" % (
            code, margins[code], args.target_margin_m,
            float(shifts[code][0]), float(shifts[code][1])))
        if worst is None or margins[code] < margins[worst]:
            worst = code
    print("结论：最紧余量 = %+.9f m（抬 %s）⇒ %s" % (
        margins[worst], worst,
        "重心投影落在支撑三角形边上/外，静态稳定前提不成立" if margins[worst] <= 0.0
        else "余量仅 %.6f m，不足以抵抗负载转移扰动" % margins[worst]))

    feasibility = constant_shift_feasibility(feet, com_xy, legs,
                                             args.shift_span_m, args.shift_step_m)
    pairs = diagonal_pairs(feet, legs)
    pair_rows = []
    for left, right in pairs:
        first = np.asarray(shifts[left], dtype=float)
        second = np.asarray(shifts[right], dtype=float)
        dot = float(first.dot(second))
        cosine = dot / (float(np.linalg.norm(first)) * float(np.linalg.norm(second)))
        angle = float(np.degrees(np.arccos(max(-1.0, min(1.0, cosine)))))
        pair_rows.append({"legs": [left, right], "required_shift_dot": dot, "angle_deg": angle,
                          "sum_bound_m": pair_sum_bound(feet, com_xy, legs, (left, right),
                                                        args.shift_span_m, args.shift_step_m)})
        print("  对角腿对 %s/%s：所需平移 (%+.6f, %+.6f) 与 (%+.6f, %+.6f) 的夹角 = %.2f°"
              "（点积 %+.3e ⇒ %s）；余量之和的上确界 = %+.9f m ⇒ %s" % (
                  left, right, float(first[0]), float(first[1]),
                  float(second[0]), float(second[1]), angle, dot,
                  "方向相反" if dot < 0.0 else "方向不相反",
                  pair_rows[-1]["sum_bound_m"],
                  "两相位余量不可能同时为正（解析界已兑现）"
                  if pair_rows[-1]["sum_bound_m"] <= 0.0 else "两相位可同时为正"))
    feasible = feasibility["best_min_margin_m"] > 0.0
    print("常量平移可行性（穷举 %d 点，±%.3f m / 步长 %.3f m）：最优常量平移 (%+.6f, %+.6f) 下，"
          "四相位同时可得的余量 = %+.9f m ⇒ %s" % (
              feasibility["grid_points"], args.shift_span_m, args.shift_step_m,
              feasibility["best_offset_m"][0], feasibility["best_offset_m"][1],
              feasibility["best_min_margin_m"],
              "常量平移可行" if feasible
              else "穷举范围内**不存在**可行常量平移（可行域为空集）⇒「给足端目标加常量平移偏移」"
                   "这一族修法在形式上不可行，无需再扫参数"))
    feasibility["feasible"] = feasible
    feasibility["diagonal_pairs"] = pair_rows

    if args.json:
        out = Path(args.json)
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(json.dumps({
            "simulation": True,
            "model": args.model,
            "baseline": args.baseline,
            "target_margin_m": args.target_margin_m,
            "contact_points_m": {code: [float(feet[code][0]), float(feet[code][1])] for code in legs},
            "foot_centroid_xy_m": [float(centroid[0]), float(centroid[1])],
            "com_xy_m": [float(com_xy[0]), float(com_xy[1])],
            "margin_m": {code: float(margins[code]) for code in legs},
            "required_shift_m": {code: [float(shifts[code][0]), float(shifts[code][1])]
                                 for code in legs},
            "tightest_leg": worst,
            "tightest_margin_m": float(margins[worst]),
            "constant_shift": feasibility,
        }, ensure_ascii=False, indent=1), encoding="utf-8")
        print("written %s" % out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
