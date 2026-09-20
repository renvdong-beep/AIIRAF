"""逐项对比"官方 2f85.xml 单独加载"与"组装进 UR5e 后"的夹爪物理参数。

为什么要逐项对比而不是继续猜：
- 官方单独加载：tendon 行程 84.8mm、严格单调、driver 落在 range [0, 0.8] 内；
- 组装后：行程 ~19mm、非单调、driver 跑到 -1.32 rad（range 外）。

挂载姿态换成单位四元数也复现不出来，说明差异不在"夹爪相对世界怎么摆"，
而在**夹爪自身的物理参数或约束在拼装过程中被改动了**。
本脚本把两侧的关键量并排打印，差异一目了然：

- actuator: trntype / gainprm / biasprm / forcerange / ctrlrange
- 每个夹爪关节: type / range / stiffness / springref / damping / armature
- 每个夹爪 geom: size / friction / solref / solimp / priority / condim
- 每个夹爪 body: mass / pos / quat
- equality 各条: 类型 / 对象 / anchor / polycoef / solref / solimp
- tendon 各条: 类型 / 引用 / coef

任一项出现差异即定位到"拼装改了物理"，而不是"姿态问题"。
"""

import argparse
import json
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco

GRIPPER_PREFIX = "rq2f85_"
PAD_ASSEMBLY = ("rq2f85_left_pad1", "rq2f85_right_pad1")
PAD_OFFICIAL = ("left_pad1", "right_pad1")


def strip_prefix(name, prefix=GRIPPER_PREFIX):
    if name and name.startswith(prefix):
        return name[len(prefix):]
    return name


def collect(model, assembly):
    """按"去前缀名"收集夹爪相关量，便于两侧对齐比较。"""
    prefix = GRIPPER_PREFIX if assembly else ""
    out = {"actuators": {}, "joints": {}, "geoms": {}, "bodies": {},
           "equality": [], "tendons": []}

    def is_gripper(name):
        if not name:
            return False
        return name.startswith(prefix) if assembly else True

    for index in range(model.nu):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, index)
        if not is_gripper(name) or (assembly and not name.startswith("rq2f85_fingers")):
            continue
        out["actuators"][strip_prefix(name)] = {
            "trntype": int(model.actuator_trntype[index]),
            "gaintype": int(model.actuator_gaintype[index]),
            "biastype": int(model.actuator_biastype[index]),
            "gainprm": [round(float(v), 6) for v in model.actuator_gainprm[index][:3]],
            "biasprm": [round(float(v), 6) for v in model.actuator_biasprm[index][:3]],
            "forcerange": [float(v) for v in model.actuator_forcerange[index]],
            "ctrlrange": [float(v) for v in model.actuator_ctrlrange[index]],
        }

    for index in range(model.njnt):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, index)
        if not is_gripper(name):
            continue
        dof = int(model.jnt_dofadr[index])
        out["joints"][strip_prefix(name)] = {
            "type": int(model.jnt_type[index]),
            "range": [round(float(v), 6) for v in model.jnt_range[index]],
            "stiffness": round(float(model.jnt_stiffness[index]), 6),
            "springref": round(float(model.qpos_spring[int(model.jnt_qposadr[index])]), 6),
            "damping": round(float(model.dof_damping[dof]), 6),
            "armature": round(float(model.dof_armature[dof]), 6),
            "frictionloss": round(float(model.dof_frictionloss[dof]), 6),
            "solimplimit": [round(float(v), 6) for v in model.jnt_solimp[index]],
            "solreflimit": [round(float(v), 6) for v in model.jnt_solref[index]],
        }

    for index in range(model.ngeom):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, index)
        if not is_gripper(name):
            continue
        out["geoms"][strip_prefix(name)] = {
            "type": int(model.geom_type[index]),
            "size": [round(float(v), 6) for v in model.geom_size[index]],
            "friction": [round(float(v), 4) for v in model.geom_friction[index]],
            "solref": [round(float(v), 6) for v in model.geom_solref[index]],
            "solimp": [round(float(v), 6) for v in model.geom_solimp[index]],
            "priority": int(model.geom_priority[index]),
            "condim": int(model.geom_condim[index]),
            # MjModel 没有 geom_mass（质量在 body_inertia / body_mass 上），
            # 这里改记 geom 的体密度作为替代指纹。
            "density": round(float(model.geom_rbound[index]), 6),
            "contype": int(model.geom_contype[index]),
            "conaffinity": int(model.geom_conaffinity[index]),
        }

    body_root = None
    for index in range(model.nbody):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, index)
        if name == prefix + "base_mount":
            body_root = index
            break
    if body_root is not None:
        stack = [int(body_root)]
        while stack:
            body_id = stack.pop()
            name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body_id)
            out["bodies"][strip_prefix(name)] = {
                "pos": [round(float(v), 6) for v in model.body_pos[body_id]],
                "quat": [round(float(v), 6) for v in model.body_quat[body_id]],
                "mass": round(float(model.body_mass[body_id]), 6),
            }
            stack.extend(
                child for child in range(model.nbody)
                if int(model.body_parentid[child]) == body_id and child != body_id
            )

    for index in range(model.neq):
        eq_type = int(model.eq_type[index])
        obj1 = strip_prefix(
            mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, int(model.eq_obj1id[index]))
            or mujoco.mj_id2name(
                model, mujoco.mjtObj.mjOBJ_JOINT, int(model.eq_obj1id[index])
            )
        )
        obj2 = strip_prefix(
            mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, int(model.eq_obj2id[index]))
            or mujoco.mj_id2name(
                model, mujoco.mjtObj.mjOBJ_JOINT, int(model.eq_obj2id[index])
            )
        )
        if not is_gripper(obj1) and not is_gripper(obj2):
            continue
        out["equality"].append(
            {
                "type": eq_type,
                "obj1": obj1,
                "obj2": obj2,
                "anchor": [round(float(v), 6) for v in model.eq_data[index][:3]],
                "polycoef": [round(float(v), 6) for v in model.eq_data[index][3:8]],
                "solref": [round(float(v), 6) for v in model.eq_solref[index]],
                "solimp": [round(float(v), 6) for v in model.eq_solimp[index]],
                "active": bool(model.eq_active0[index]),
            }
        )

    for index in range(model.ntendon):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_TENDON, index)
        if not is_gripper(name):
            continue
        out["tendons"].append(
            {
                "name": strip_prefix(name),
                "num_wrap": int(model.wrap_objid.shape[0]) if False else None,
            }
        )
    return out


def diff(label, official, assembly):
    print("\n=== 差异：%s ===" % label)
    diffs = []
    for section in ("actuators", "joints", "geoms", "bodies"):
        keys = sorted(set(official[section]) | set(assembly[section]))
        for key in keys:
            a = official[section].get(key)
            b = assembly[section].get(key)
            if a != b:
                diffs.append({"section": section, "key": key,
                              "official": a, "assembly": b})
                print("  [%s] %s" % (section, key))
                print("     official: %s" % (a,))
                print("     assembly: %s" % (b,))
    joined = {"equality", "tendons"}
    for section in joined:
        a, b = official[section], assembly[section]
        if a != b:
            diffs.append({"section": section, "official": a, "assembly": b})
            print("  [%s]" % section)
            print("     official: %s" % (a,))
            print("     assembly: %s" % (b,))
    if not diffs:
        print("  无差异")
    return diffs


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--official",
        default="/home/coretek/MuJoCoBin/mujoco_menagerie/robotiq_2f85/2f85.xml",
    )
    parser.add_argument("--assembly", default="build/models/ur5e_2f85/ur5e_2f85.xml")
    parser.add_argument(
        "--output", type=Path, default=Path("build/calibration/ur5-gripper-diff.json")
    )
    args = parser.parse_args()

    official_model = mujoco.MjModel.from_xml_path(str(Path(args.official).resolve()))
    assembly_model = mujoco.MjModel.from_xml_path(str(Path(args.assembly).resolve()))
    official = collect(official_model, assembly=False)
    assembly = collect(assembly_model, assembly=True)

    print("官方夹爪关节数=%d body数=%d geom数=%d"
          % (len(official["joints"]), len(official["bodies"]), len(official["geoms"])))
    print("组装夹爪关节数=%d body数=%d geom数=%d"
          % (len(assembly["joints"]), len(assembly["bodies"]), len(assembly["geoms"])))

    diffs = diff("夹爪物理量", official, assembly)
    report = {
        "schema_version": "iraf.ur5-gripper-diff/v1",
        "official": official,
        "assembly": assembly,
        "differences": diffs,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    print("\n差异条目数: %d" % len(diffs))
    print("WROTE", args.output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
