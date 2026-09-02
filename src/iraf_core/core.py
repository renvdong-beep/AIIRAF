from dataclasses import dataclass
from enum import Enum
class TaskStatus(str,Enum):
    PENDING='PENDING'; VALIDATING='VALIDATING'; RUNNING='RUNNING'; SUCCEEDED='SUCCEEDED'; FAILED='FAILED'; ABORTED='ABORTED'; CANCELLED='CANCELLED'; SAFETY_STOP='SAFETY_STOP'
@dataclass(frozen=True)
class RobotProfile:
    name:str; version:str; simulation:bool; control_frequency_hz:float; joints:tuple=(); joint_limits:dict=None; capabilities:frozenset=frozenset(); verification:str='development'; digest:str=''
@dataclass(frozen=True)
class SafetyPolicy:
    name:str; version:str; verification:str; simulation_only:bool; allowed_skills:frozenset; max_duration_ms:int; digest:str=''