from dataclasses import dataclass
from enum import Enum
class TaskStatus(str,Enum):
    PENDING='PENDING'; VALIDATING='VALIDATING'; RUNNING='RUNNING'; SUCCEEDED='SUCCEEDED'; FAILED='FAILED'; ABORTED='ABORTED'; CANCELLED='CANCELLED'; SAFETY_STOP='SAFETY_STOP'
@dataclass(frozen=True)
class RobotProfile:
    name:str; version:str; simulation:bool; control_frequency_hz:float; joints:tuple=(); joint_limits:dict=None; capabilities:frozenset=frozenset(); verification:str='development'; digest:str=''
    # 结构化差异声明（全部可选）。缺失时使用方回退到既有平铺语义，
    # 保证既有 profile 与按位置参数构造的调用完全不受影响。
    joint_roles:dict=None; gripper:dict=None; home:dict=None; camera:dict=None; manipulation:dict=None
@dataclass(frozen=True)
class SafetyPolicy:
    name:str; version:str; verification:str; simulation_only:bool; allowed_skills:frozenset; max_duration_ms:int; digest:str=''