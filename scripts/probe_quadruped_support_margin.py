"""静态稳定余量探针：量出「抬单腿时的支撑三角形」对整机重心投影的有符号余量。

用途（战役 iraf-24h-2 步骤 02 的根因取证）
------------------------------------------
准静态步态（wave/creep）的静态稳定前提是「重心投影落在支撑多边形内部」。
本探针把它变成数字：从**编译后的模型**与基线声明（`gait.legs.*.contact_geom`）出发，
①量四足接触点、②量整机重心投影、③对「抬任意一条腿」给出支撑三角形的最小有符号余量
（>0 = 重心在三角形内）、④给出把余量提到目标值所需的重心平移矢量。

实测（2026-09-21，Go2 + handoff_lab 场景）：中立站姿四足接触点构成 0.3868 × 0.2840 m 的矩形，
重心投影距支撑三角形最紧边的余量仅 ±0.000227483 m（抬 RL/RR 时为**负**）——注意这是**几何事实**：
**矩形站姿的对角线恒过矩形中心**，所以抬任意一腿后支撑三角形必有一条边穿过重心投影，
余量与站位宽度无关。因此「≥3 条腿支撑 ⇒ 静态稳定」在这类站姿下**恒不成立**，
必须靠重心平移（重心进入三角形内部并留出余量）才谈得上静态稳定。

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
        [--target-margin-m 0.01] [--json build/iraf-24h-2/02/support-margin.json]
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


def support_margins(model, data, mujoco_module, legs, contact_geoms):
    """返回 (feet, com_xy, margins, shifts)：余量与「提到目标余量所需平移」按抬腿逐条给。"""
    feet = {}
    for code in legs:
        gid = mujoco_module.mj_name2id(model, mujoco_module.mjtObj.mjOBJ_GEOM, contact_geoms[code])
        if gid < 0:
            raise SystemExit("足端几何 %s 不在模型里" % contact_geoms[code])
        center = np.asarray(data.geom_xpos[gid], dtype=float)
        feet[code] = np.array([float(center[0]), float(center[1])])
    com_xy = np.array([float(data.subtree_com[0][0]), float(data.subtree_com[0][1])])
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
    return feet, com_xy, margins, normals


def main(argv=None):
    parser = argparse.ArgumentParser(description="四足静态稳定余量探针（只读）")
    parser.add_argument("--baseline", default="config/go2_loopback.yaml",
                        help="基线声明（只读 gait.legs.*.contact_geom）")
    parser.add_argument("--model", default=DEFAULT_MODEL, help="编译后的 MJCF/XML")
    parser.add_argument("--target-margin-m", type=float, default=0.01,
                        help="目标余量（m），用于折算所需的重心平移矢量")
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
    shifts = {
        code: normals[code] * (args.target_margin_m - margins[code]) for code in legs
    }

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
        }, ensure_ascii=False, indent=1), encoding="utf-8")
        print("written %s" % out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
