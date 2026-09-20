"""确认法兰 quat 与夹爪挂载姿态的关系，为"归一化法兰 quat"提供证据。

已知事实：
- 官方 2f85.xml 单独加载时，tendon 从 ctrl=0..255 让 pad 间隙从
  0.0854 单调收到 0.0004（行程 84.8mm），driver 走 0→0.78 rad。
- 组装后同一夹爪：driver 被拖到 -1.32 rad（range 外），
  follower 符号翻转（-0.76 → +0.84），间隙反而变大 +19mm，行程仅 19mm。

假设：UR5e 的 `attachment_site` 给的 quat="-1 1 0 0"（wxyz 归一化后
= (0, 0.7071, 0.7071, 0)）是一个 180° 旋转。夹爪根 body 直接套用该姿态，
使夹爪的局部系相对官方定义被翻转，equality 的 connect anchor（相对
body1 的局部坐标）因而落在错误的参考方向上。

本脚本不改模型，只做**对照实验**：把夹爪挂载 quat 换成单位四元数
（"1 0 0 0"）与法兰 quat，各自跑一次动态 tendon 扫描，看哪种能复现
官方模型的单调行程。这样"该用哪个 quat"就有数据支撑，而不是靠推断。
"""

import argparse
import json
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np

PAD_ASSEMBLY = ("rq2f85_left_pad1", "rq2f85_right_pad1")
ACTUATOR = "rq2f85_fingers_actuator"


def _geom(model, name):
    geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)
    if geom_id < 0:
        raise ValueError("缺少 geom: " + name)
    return int(geom_id)


def pad_gap(model, data):
    left = _geom(model, PAD_ASSEMBLY[0])
    right = _geom(model, PAD_ASSEMBLY[1])
    centres = np.asarray(data.geom_xpos[[left, right]], dtype=float)
    half_y = float(model.geom_size[left][1])
    return float(np.linalg.norm(centres[1] - centres[0])) - 2.0 * half_y


def relocate_meshdir(tree, model_dir):
    compiler = tree.getroot().find("compiler")
    if compiler is not None:
        compiler.set("meshdir", str((model_dir / "assets").resolve()))


def set_mount_quat(tree, quat):
    root = tree.getroot()
    world = root.find("worldbody")
    wrist3 = None
    for body in world.iter("body"):
        if body.get("name") == "wrist_3_link":
            wrist3 = body
            break
    if wrist3 is None:
        raise ValueError("缺少 wrist_3_link")
    target = None
    for body in wrist3.findall("body"):
        if (body.get("name") or "").startswith("rq2f85_base_mount"):
            target = body
            break
    if target is None:
        raise ValueError("未找到夹爪根 body")
    target.set("quat", " ".join("%.9f" % float(v) for v in quat))
    return target.get("name")


def dynamic_sweep(model, values, steps):
    data = mujoco.MjData(model)
    actuator_id = int(
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, ACTUATOR)
    )
    rows = []
    for value in values:
        data.qpos[:] = 0.0
        data.qvel[:] = 0.0
        data.ctrl[:] = 0.0
        mujoco.mj_forward(model, data)
        before = pad_gap(model, data)
        data.ctrl[actuator_id] = float(value)
        for _ in range(int(steps)):
            data.ctrl[actuator_id] = float(value)
            mujoco.mj_step(model, data)
        after = pad_gap(model, data)
        rows.append(
            {
                "ctrl": float(value),
                "gap_before_m": round(before, 6),
                "gap_after_m": round(after, 6),
                "gap_delta_m": round(after - before, 6),
            }
        )
    gaps = [row["gap_after_m"] for row in rows]
    return {
        "rows": rows,
        "stroke_m": round(max(gaps) - min(gaps), 6),
        "monotonic_decreasing": all(
            gaps[i] >= gaps[i + 1] - 1e-4 for i in range(len(gaps) - 1)
        ),
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--assembly", default="build/models/ur5e_2f85/ur5e_2f85.xml")
    parser.add_argument("--steps", type=int, default=2500)
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("build/calibration/ur5-mount-orientation.json"),
    )
    args = parser.parse_args()

    source = Path(args.assembly).resolve()
    values = [0.0, 60.0, 120.0, 180.0, 240.0, 255.0]

    candidates = {
        "flange_site_quat": [0.0, 0.70710678, 0.70710678, 0.0],
        "identity": [1.0, 0.0, 0.0, 0.0],
    }
    report = {"schema_version": "iraf.ur5-mount-orientation/v1", "candidates": {}}

    for label, quat in candidates.items():
        tree = ET.parse(str(source))
        relocate_meshdir(tree, source.parent)
        body_name = set_mount_quat(tree, quat)
        out = source.with_name("mount-probe-%s.xml" % label)
        ET.indent(tree, space="    ")
        tree.write(str(out), encoding="unicode", xml_declaration=True)
        model = mujoco.MjModel.from_xml_path(str(out))
        result = dynamic_sweep(model, values, args.steps)
        report["candidates"][label] = {"quat": quat, "mount_body": body_name, **result}
        print("=== mount quat %s (%s) ===" % (label, quat))
        for row in result["rows"]:
            print("  ctrl=%6.1f gap %+.6f -> %+.6f (delta %+.6f)"
                  % (row["ctrl"], row["gap_before_m"], row["gap_after_m"],
                     row["gap_delta_m"]))
        print("  stroke=%.6f monotonic_decreasing=%s"
              % (result["stroke_m"], result["monotonic_decreasing"]))

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    print("\nWROTE", args.output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
