"""定位"推动方块的是哪个 link/geom"：在接触发生瞬间打印低位几何体与接触对。

`scripts/probe_approach_contact.py` 已证明：
- 方块在 APPROACH 段 t≈1.24s 开始位移，接触对方是 `wrist_2_link` 的官方碰撞几何
  （geom#26/#27），**不是 pad box**；
- 该时刻夹持区中点（4 个 pad geom 中心均值）仍在 z≈0.34m，远高于方块顶面 0.05m。

本脚本把"谁在低位"直接量化：在每个关键瞬间列出所有质心 z < --z-threshold 的
geom（含所属 body、类型、尺寸、世界坐标），以及当前所有涉及方块与机械臂的接触对，
用于判定推动方与接触法向。

用法：
  python3 scripts/probe_contact_geometry.py --scene build/models/ur5-pick-scene.xml
"""

import argparse
import json
import math
from pathlib import Path

import mujoco
import numpy as np


def quintic(start, end, duration_s, t):
    if t >= duration_s:
        return end
    x = t / duration_s
    return start + (end - start) * (10 * x**3 - 15 * x**4 + 6 * x**5)


GEOM_TYPE = {
    0: "plane",
    2: "sphere",
    3: "capsule",
    4: "ellipsoid",
    5: "cylinder",
    6: "box",
    7: "mesh",
}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--scene", default="build/models/ur5-pick-scene.xml")
    parser.add_argument("--duration-ms", type=int, default=12000)
    parser.add_argument("--z-threshold", type=float, default=0.20)
    args = parser.parse_args()

    root = Path(__file__).resolve().parents[1]
    scene_xml = root / args.scene
    scene = json.loads(scene_xml.with_suffix(".json").read_text(encoding="utf-8"))
    gripper = scene["gripper"]

    model = mujoco.MjModel.from_xml_path(str(scene_xml))
    data = mujoco.MjData(model)
    if int(model.nkey) > 0:
        mujoco.mj_resetDataKeyframe(model, data, 0)
    mujoco.mj_forward(model, data)

    def gid(name):
        return int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name))

    def gname(index):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, index)
        return name if name else "<geom#%d>" % index

    def bname(index):
        return mujoco.mj_id2name(
            model, mujoco.mjtObj.mjOBJ_BODY, int(model.geom_bodyid[index])
        )

    box_geom = gid("box_01_geom")
    workbench_geom = gid("workbench")
    pad_names = list(scene["finger_geoms"]["pad_boxes"])
    pad_geoms = [gid(n) for n in pad_names]
    box_body = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, scene["target_id"]))

    def snapshot(label):
        print("=" * 78)
        print(label)
        print("  方块世界坐标: %s" % np.round(data.xpos[box_body], 6))
        print("  低位 geom（质心 z < %.3f）:" % args.z_threshold)
        for index in range(model.ngeom):
            z = float(data.geom_xpos[index][2])
            if z >= args.z_threshold:
                continue
            if index == workbench_geom:
                continue
            print(
                "    %-24s body=%-24s %-8s size=%-28s pos=%s"
                % (
                    gname(index),
                    bname(index),
                    GEOM_TYPE.get(int(model.geom_type[index]), "?"),
                    [round(float(v), 4) for v in model.geom_size[index]],
                    [round(float(v), 5) for v in data.geom_xpos[index]],
                )
            )
        print("  涉及方块的接触对:")
        found = False
        for index in range(int(data.ncon)):
            contact = data.contact[index]
            pair = (int(contact.geom1), int(contact.geom2))
            if box_geom not in pair:
                continue
            other = pair[1] if pair[0] == box_geom else pair[0]
            found = True
            print(
                "    box_01_geom <-> %-22s (body=%-20s) dist=%+.5f pos=%s"
                % (
                    gname(other),
                    bname(other),
                    float(contact.dist),
                    [round(float(v), 5) for v in contact.pos],
                )
            )
        if not found:
            print("    （无）")
        print("  夹持区中点: %s" % np.round(
            np.mean([np.asarray(data.geom_xpos[g]) for g in pad_geoms], axis=0), 6
        ))
        for name in pad_names:
            index = gid(name)
            print(
                "    %-20s center=%s  局部z轴世界方向=%s"
                % (
                    name,
                    [round(float(v), 5) for v in data.geom_xpos[index]],
                    [round(float(v), 4) for v in data.geom_xmat[index].reshape(3, 3)[:, 2]],
                )
            )
        print()

    first_arm_contact = None
    dt = float(model.opt.timestep)
    segment_ms = max(1, args.duration_ms // 5)

    for phase, key in (
        ("HOME_HOLD", "home_positions"),
        ("APPROACH", "approach_positions"),
        ("DESCEND", "grasp_positions"),
    ):
        targets = gripper[key]
        names = list(targets)
        actuators = {
            name: int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name))
            for name in names
        }
        starts = [
            float(data.qpos[int(model.jnt_qposadr[int(model.actuator_trnid[actuators[n], 0])])])
            for n in names
        ]
        steps = max(1, int(math.ceil(segment_ms / 1000.0 / dt)))
        for step in range(1, steps + 1):
            elapsed = step * dt
            for i, name in enumerate(names):
                data.ctrl[actuators[name]] = quintic(
                    starts[i], float(targets[name]), segment_ms / 1000.0, elapsed
                )
            mujoco.mj_step(model, data)
            if first_arm_contact is None:
                for index in range(int(data.ncon)):
                    contact = data.contact[index]
                    pair = (int(contact.geom1), int(contact.geom2))
                    if box_geom not in pair:
                        continue
                    other = pair[1] if pair[0] == box_geom else pair[0]
                    if other == workbench_geom:
                        continue
                    first_arm_contact = (
                        phase,
                        elapsed * 1000.0,
                        gname(other),
                        bname(other),
                        float(contact.dist),
                        [round(float(v), 5) for v in data.xpos[box_body]],
                    )
                    snapshot(
                        "首次非台面接触：phase=%s t=%.3fms 对方=%s(body=%s) dist=%+.5f 方块=%s"
                        % (
                            first_arm_contact[0],
                            first_arm_contact[1],
                            first_arm_contact[2],
                            first_arm_contact[3],
                            first_arm_contact[4],
                            first_arm_contact[5],
                        )
                    )
        settle_ms = max(250, min(16000, segment_ms * 4))
        for name in names:
            data.ctrl[actuators[name]] = float(targets[name])
        for _ in range(max(1, int(math.ceil(settle_ms / 1000.0 / dt)))):
            mujoco.mj_step(model, data)
        mujoco.mj_forward(model, data)
        snapshot("段末：%s（settle=%dms）" % (phase, settle_ms))

    print("首次非台面接触汇总: %s" % json.dumps(first_arm_contact, ensure_ascii=False))


if __name__ == "__main__":
    main()
