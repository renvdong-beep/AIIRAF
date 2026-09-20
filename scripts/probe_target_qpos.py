"""定位目标方块位置错乱的根因：freejoint qpos 是否被初始化为场景声明的位姿。

现象（probe_grasp_alignment.py）：
- 场景声明的目标位置是 [-0.55, -0.134, 0.025]；
- 实测 box_01 在 [-2.92, -1.47, 1.03]，且位置随机械臂姿态变化；
- 场景里目标带 freejoint，但**没有任何地方把它的 qpos 设成声明位姿**。

MuJoCo 的自由关节默认 qpos 是**全零**（位置 0、四元数 (1,0,0,0)），
所以方块会出现在世界原点附近并受重力下落，
随后被机械臂扫到并带走——这解释了"位置随姿态变化"。

本脚本做三件事：
1. 打印模型里目标 freejoint 的 qposadr，确认它存在；
2. 打印 MjData 初始化后的 qpos（应为零），证明未初始化；
3. 打印模型自带 keyframe 的内容，看它是否本应承载目标位姿。
"""

import argparse
from pathlib import Path

import mujoco
import numpy as np


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--scene", default="build/models/ur5-pick-scene.xml")
    parser.add_argument("--target", default="box_01")
    args = parser.parse_args()

    root = Path(__file__).resolve().parents[1]
    model = mujoco.MjModel.from_xml_path(str(root / args.scene))
    data = mujoco.MjData(model)

    print("nq=%d nkey=%d nmocap=%d" % (model.nq, model.nkey, model.nmocap))
    print("key names:", [
        mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_KEY, i)
        for i in range(model.nkey)
    ])

    for name in (args.target, args.target + "_free"):
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        print("joint %-16s id=%d" % (name, joint_id))
        if joint_id >= 0:
            adr = int(model.jnt_qposadr[joint_id])
            print("   type=%d qposadr=%d" % (int(model.jnt_type[joint_id]), adr))

    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, args.target)
    print("\nbody %s id=%d" % (args.target, body_id))
    free_joint = int(model.body_jntadr[body_id])
    print("   first joint =", free_joint,
          mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, free_joint))
    adr = int(model.jnt_qposadr[free_joint])
    dof = int(model.jnt_dofadr[free_joint])
    print("   qposadr=%d dofadr=%d" % (adr, dof))

    print("\nMjData 初始化后的目标 qpos（应为全零，即未初始化）:")
    print("   qpos[%d:%d] = %s" % (adr, adr + 7, np.round(data.qpos[adr:adr + 7], 6)))
    print("   body xpos =", np.round(data.xpos[body_id], 6))

    if model.nkey:
        key = model.key_qpos[0]
        print("\nkeyframe[0] qpos:", np.round(key, 6))
        print("   keyframe 里目标段 =", np.round(key[adr:adr + 7], 6))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
