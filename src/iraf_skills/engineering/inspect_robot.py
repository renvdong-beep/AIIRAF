"""inspect-robot：读厂商 MJCF 模型 → 关节/限位/执行器事实（构建期工具，不驱动机器人）。"""
import mujoco

class InspectRobotProvider:
    def __init__(self, profile, backend):
        self.profile = profile
        self.backend = backend

    def execute(self, inputs, lease):
        path = inputs["model_path"]
        m = mujoco.MjModel.from_xml_path(path)
        joints = []
        for i in range(m.njnt):
            jn = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, i)
            if jn:
                r = m.jnt_range[i]
                joints.append({"name": jn, "limit": [round(float(r[0]), 4), round(float(r[1]), 4)]})
        actuators = []
        for i in range(m.nu):
            an = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
            actuators.append({"index": i, "name": an, "type": int(m.actuator_trntype[i]),
                              "ctrlrange": [round(float(m.actuator_ctrlrange[i, 0]), 4),
                                            round(float(m.actuator_ctrlrange[i, 1]), 4)]})
        bodies = [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, i) for i in range(m.nbody)
                  if mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, i)]
        evidence = {"nq": int(m.nq), "nv": int(m.nv), "nu": int(m.nu),
                    "joints": joints, "actuators": actuators, "bodies": bodies}
        return {"skill": "engineering.inspect_robot", "accepted": True, "evidence": evidence}
