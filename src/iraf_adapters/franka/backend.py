"""Franka Emika Panda 仿真 Backend（move_joint/stop/step 实测；pick/visual_pick 待接）。"""
from pathlib import Path
import numpy as np
from iraf_core.kinematics import solve_position_ik

class FrankaPandaBackend:
    ARM_JOINTS = ['joint1','joint2','joint3','joint4','joint5','joint6','joint7']
    GRIPPER_DRIVE_JOINTS = ['finger_joint1','finger_joint2']
    WRIST_BODY = 'link7'
    FINGER_GEOMS = {'left':'left_finger','right':'right_finger'}

    def __init__(self, profile, authority, config=None):
        self.profile = profile
        self.authority = authority
        self.config = config or {}
        import mujoco
        self.mujoco = mujoco
        root = Path(self.config.get("root", "/home/coretek/AIIRAF"))
        self.model = mujoco.MjModel.from_xml_path(str(root / self.config["model_file"]))
        self.data = mujoco.MjData(self.model)
        self.control_hz = float(self.config.get("control_frequency_hz", 100.0))
        self.bindings = {}
        for name in profile.joints:
            jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            acts = [i for i in range(self.model.nu)
                    if int(self.model.actuator_trntype[i]) == int(mujoco.mjtTrn.mjTRN_JOINT)
                    and int(self.model.actuator_trnid[i, 0]) == int(jid)]
            self.bindings[name] = {"joint_id": int(jid),
                                   "qpos_adr": int(self.model.jnt_qposadr[jid]),
                                   "dof_adr": int(self.model.jnt_dofadr[jid]),
                                   "actuator_id": int(acts[0]) if acts else None}

    @classmethod
    def from_config(cls, config, profile, authority):
        return cls(profile, authority, config)

    def move_joint(self, positions, duration_ms, lease):
        self.authority.validate(lease)
        ctrl = np.zeros(self.model.nu)
        for name, b in self.bindings.items():
            if b["actuator_id"] is not None:
                ctrl[b["actuator_id"]] = float(self.data.qpos[b["qpos_adr"]])
        for name, value in positions.items():
            if name in self.bindings and self.bindings[name]["actuator_id"] is not None:
                ctrl[self.bindings[name]["actuator_id"]] = float(value)
        steps = max(1, int(duration_ms / 1000.0 * self.control_hz))
        for _ in range(steps):
            self.data.ctrl[:] = ctrl
            self.mujoco.mj_step(self.model, self.data)
        achieved = {n: float(self.data.qpos[self.bindings[n]["qpos_adr"]]) for n in positions if n in self.bindings}
        return {"simulation": True, "capability": "move_joint", "duration_ms": duration_ms,
                "positions": dict(positions), "achieved_positions_rad": achieved}

    def stop(self, lease):
        self.authority.validate(lease)
        return {"simulation": True, "capability": "stop"}

    def step(self, count=1):
        for _ in range(count):
            self.mujoco.mj_step(self.model, self.data)

    def runtime_inventory(self):
        return {"safety": {"estop": False}, "manipulation": {"target_visible": False}}

    def pick_object(self, target_id, grasp_pose, duration_ms, lease):
        self.authority.validate(lease)
        raise NotImplementedError("抓取闭环待接")
    def visual_pick(self, inputs, lease):
        self.authority.validate(lease)
        raise NotImplementedError("视觉位姿估计待接")
