"""Headless MuJoCo 3 capability backend for IRAF simulation."""
from pathlib import Path
import mujoco
class MujocoBackend:
    @classmethod
    def from_config(cls,config,profile,authority): return cls(config["model_path"],profile,authority)
    def __init__(self,model_path,profile,authority):
        self.profile=profile; self.authority=authority; self.model=mujoco.MjModel.from_xml_path(str(Path(model_path))); self.data=mujoco.MjData(self.model); self._actuators={mujoco.mj_id2name(self.model,mujoco.mjtObj.mjOBJ_ACTUATOR,i):i for i in range(self.model.nu)}; self.last_positions={joint:0.0 for joint in profile.joints}; self.stopped=False
    def runtime_inventory(self): return {"safety":{"estop":False},"mode":"simulation"}
    def move_joint(self,positions,duration_ms,lease):
        self.authority.validate(lease)
        for joint,value in positions.items():
            if joint not in self._actuators: raise ValueError("未找到 actuator: "+joint)
            self.data.ctrl[self._actuators[joint]]=float(value); self.last_positions[joint]=float(value)
        self.stopped=False; return self.step(max(1,int(duration_ms/10)))
    def stop(self,lease): self.authority.validate(lease); self.data.ctrl[:]=0.0; self.stopped=True
    def step(self,count=1):
        for _ in range(count): mujoco.mj_step(self.model,self.data)
        return {joint:float(self.last_positions[joint]) for joint in self.profile.joints}