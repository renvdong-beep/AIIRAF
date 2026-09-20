"""定位 2F-85 夹爪"pad 不动"的根因：equality 的 site/body 引用是否失效。

前几轮的硬事实：

- E 组：直接枚举左右 driver 关节的 qpos（-0.9 .. +1.2），pad 间隙**恒为
  0.085400 m、pad 中心完全不动**。说明 driver 到 pad 之间没有有效约束。
- A/B/C/D 组：无论用 tendon 还是直接位置执行器、无论 equality 开关，
  driver 都被拖到 ≈ -1.2 rad，而该关节 range 是 [0, 0.8]。
- 官方 2f85.xml 的 mimic 链是：
    equality/connect  body1=right_follower body2=right_coupler
    → follower 被 coupler 拖动 → follower 经 4 杆带动 pad
    equality/joint    joint1=right_driver_joint joint2=left_driver_joint
  这些引用在**加前缀改名**时必须同步改写，否则 MuJoCo 编译期会静默丢约束
  （引用不到的 body/joint 不会报错，只会让约束不生效）。

因此本脚本直接检查：组装后模型里 equality 到底还存在几条、引用了哪些
body/joint，以及原官方模型的名字与组装模型名字的对照。
"""

import argparse
import json
from pathlib import Path

import mujoco
import numpy as np

#: 组装模型里 pad 改了名；官方模型里仍是原名。两套名字都要能解析。
PAD_GEOMS_ASSEMBLY = ("rq2f85_left_pad1", "rq2f85_right_pad1")
PAD_GEOMS_OFFICIAL = ("left_pad1", "right_pad1")

PAD_GEOMS = PAD_GEOMS_ASSEMBLY


def _geom(model, name):
    geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)
    if geom_id < 0:
        raise ValueError("缺少 geom: " + name)
    return int(geom_id)


def pad_gap(model, data, names=None):
    names = names or PAD_GEOMS
    left = _geom(model, names[0])
    right = _geom(model, names[1])
    centres = np.asarray(data.geom_xpos[[left, right]], dtype=float)
    half_y = float(model.geom_size[left][1])
    return float(np.linalg.norm(centres[1] - centres[0])) - 2.0 * half_y


def dump_equality(model, label):
    print("\n=== %s: neq=%d ===" % (label, model.neq))
    rows = []
    for index in range(model.neq):
        eq_type = int(model.eq_type[index])
        obj1_name = None
        obj2_name = None
        if eq_type == int(mujoco.mjtEq.mjEQ_CONNECT):
            obj1 = int(model.eq_obj1id[index])
            obj2 = int(model.eq_obj2id[index])
            obj1_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, obj1)
            obj2_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, obj2)
        elif eq_type == int(mujoco.mjtEq.mjEQ_JOINT):
            obj1 = int(model.eq_obj1id[index])
            obj2 = int(model.eq_obj2id[index])
            obj1_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, obj1)
            obj2_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, obj2)
        rows.append(
            {
                "index": index,
                "type": eq_type,
                "obj1": obj1_name,
                "obj2": obj2_name,
                "obj1_id": int(model.eq_obj1id[index]),
                "obj2_id": int(model.eq_obj2id[index]),
                "active": bool(model.eq_active0[index]),
            }
        )
        print("  eq[%d] type=%d obj1=%s(%d) obj2=%s(%d) active=%s"
              % (index, eq_type, obj1_name, int(model.eq_obj1id[index]),
                 obj2_name, int(model.eq_obj2id[index]),
                 bool(model.eq_active0[index])))
    return rows


def dump_body_chain(model, root_name):
    """从体链条自顶向下打印 pad 所处的运动链，验证父子关系是否完整。"""
    print("\n--- 运动链（从 %s 起）---" % root_name)
    root_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, root_name)
    if root_id < 0:
        print("  未找到 body", root_name)
        return []

    chain = []
    stack = [(int(root_id), 0)]
    while stack:
        body_id, depth = stack.pop()
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body_id)
        joints = [
            mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, j)
            for j in range(model.njnt)
            if int(model.jnt_bodyid[j]) == body_id
        ]
        print("  %s%s joints=%s" % ("  " * depth, name, joints))
        chain.append({"body": name, "depth": depth, "joints": joints})
        children = [
            child for child in range(model.nbody)
            if int(model.body_parentid[child]) == body_id and child != body_id
        ]
        for child in reversed(children):
            stack.append((child, depth + 1))
    return chain


def mimic_probe(model, data):
    """直接写 follower / coupler 的 qpos，看 pad 是否跟随（判断链是否断开）。"""
    rows = []
    targets = [
        "rq2f85_right_follower_joint",
        "rq2f85_right_coupler_joint",
        "rq2f85_right_spring_link_joint",
    ]
    for joint_name in targets:
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
        if joint_id < 0:
            rows.append({"joint": joint_name, "error": "missing"})
            continue
        adr = int(model.jnt_qposadr[joint_id])
        # follower/coupler 关节名在组装模型里带前缀，官方模型里不带；
        # 传进来的 joint_name 已按调用方适配，pad 名字同理。
        pad_names = (
            PAD_GEOMS_ASSEMBLY
            if mujoco.mj_name2id(
                model, mujoco.mjtObj.mjOBJ_GEOM, PAD_GEOMS_ASSEMBLY[0]
            ) >= 0
            else PAD_GEOMS_OFFICIAL
        )
        samples = []
        for value in (-0.3, -0.1, 0.1, 0.3):
            data.qpos[:] = 0.0
            data.qvel[:] = 0.0
            data.qpos[adr] = float(value)
            mujoco.mj_forward(model, data)
            samples.append(
                {
                    "qpos_rad": float(value),
                    "pad_gap_m": round(pad_gap(model, data, pad_names), 6),
                }
            )
        rows.append({"joint": joint_name, "samples": samples})
        print("\n  %s:" % joint_name)
        for sample in samples:
            print("    qpos=%+.2f -> pad_gap %+.6f"
                  % (sample["qpos_rad"], sample["pad_gap_m"]))
    return rows


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--assembly", default="build/models/ur5e_2f85/ur5e_2f85.xml")
    parser.add_argument(
        "--official", default="/home/coretek/MuJoCoBin/mujoco_menagerie/robotiq_2f85/2f85.xml"
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("build/calibration/ur5-gripper-rootcause.json"),
    )
    args = parser.parse_args()

    report = {"schema_version": "iraf.ur5-gripper-rootcause/v1"}

    official = mujoco.MjModel.from_xml_path(str(Path(args.official).resolve()))
    report["official_equality"] = dump_equality(official, "官方 2f85.xml（单独加载）")
    official_data = mujoco.MjData(official)
    official_data.qpos[:] = 0.0
    mujoco.mj_forward(official, official_data)
    report["official_pad_gap_at_zero"] = round(
        pad_gap(official, official_data, PAD_GEOMS_OFFICIAL), 6
    )
    print("\n官方模型 qpos=0 时 pad_gap = %.6f" % report["official_pad_gap_at_zero"])
    report["official_chain"] = dump_body_chain(official, "base_mount")
    report["official_mimic"] = mimic_probe(official, official_data)

    assembly = mujoco.MjModel.from_xml_path(str(Path(args.assembly).resolve()))
    report["assembly_equality"] = dump_equality(assembly, "组装模型")
    assembly_data = mujoco.MjData(assembly)
    assembly_data.qpos[:] = 0.0
    mujoco.mj_forward(assembly, assembly_data)
    report["assembly_pad_gap_at_zero"] = round(pad_gap(assembly, assembly_data), 6)
    print("\n组装模型 qpos=0 时 pad_gap = %.6f" % report["assembly_pad_gap_at_zero"])
    report["assembly_mimic"] = mimic_probe(assembly, assembly_data)

    print("\n--- 名字对照 ---")
    official_bodies = {
        mujoco.mj_id2name(official, mujoco.mjtObj.mjOBJ_BODY, i)
        for i in range(official.nbody)
    }
    assembly_bodies = {
        mujoco.mj_id2name(assembly, mujoco.mjtObj.mjOBJ_BODY, i)
        for i in range(assembly.nbody)
    }
    missing = sorted(name for name in official_bodies if name and name not in assembly_bodies)
    print("官方有、组装缺失的 body 名:", missing)
    report["bodies_missing_in_assembly"] = missing

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    print("\nWROTE", args.output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
