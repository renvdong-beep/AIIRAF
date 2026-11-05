"""解析机器人模型（MJCF）→ 事实 JSON（inspect-robot 脚本化）。用法：python3 scripts/inspect_robot.py <model.xml>"""
import sys, json
import mujoco

def inspect(path):
    m = mujoco.MjModel.from_xml_path(path)
    joints = []
    for i in range(m.njnt):
        jn = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, i)
        if jn:
            r = m.jnt_range[i]
            joints.append({"name": jn, "limit": [round(float(r[0]),4), round(float(r[1]),4)]})
    actuators = []
    for i in range(m.nu):
        an = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
        actuators.append({"index": i, "name": an, "type": int(m.actuator_trntype[i]),
                          "ctrlrange": [round(float(m.actuator_ctrlrange[i,0]),4), round(float(m.actuator_ctrlrange[i,1]),4)]})
    bodies = [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, i) for i in range(m.nbody)
              if mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, i)]
    return {"nq": m.nq, "nv": m.nv, "nu": m.nu, "joints": joints, "actuators": actuators, "bodies": bodies}

if __name__ == "__main__":
    print(json.dumps(inspect(sys.argv[1]), ensure_ascii=False, indent=2))
