"""通用关节运动 Provider；机器人差异由 RobotProfile 和 Backend 提供。"""
class SkillRejected(ValueError): pass

class MoveJointProvider:
    def __init__(self,profile,backend): self.profile=profile; self.backend=backend
    def execute(self,inputs,lease):
        positions=inputs["positions"]; duration_ms=int(inputs.get("duration_ms",1000))
        unknown=set(positions)-set(self.profile.joints)
        if unknown: raise SkillRejected("未知关节: "+str(sorted(unknown)))
        for joint,value in positions.items():
            low,high=self.profile.joint_limits[joint]
            if not low<=float(value)<=high: raise SkillRejected("关节超限: "+joint)
        self.backend.move_joint({k:float(v) for k,v in positions.items()},duration_ms,lease)
        return {"skill":"move_joint","accepted":True}

class StopProvider:
    def __init__(self,profile,backend): self.backend=backend
    def execute(self,inputs,lease):
        self.backend.stop(lease)
        return {"skill":"stop","accepted":True}