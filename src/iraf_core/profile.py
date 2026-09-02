from pathlib import Path
import hashlib,yaml
from .core import RobotProfile,SafetyPolicy
class ProfileError(ValueError): pass
def load_robot_profile(path):
    raw=Path(path).read_bytes(); data=yaml.safe_load(raw) or {}; meta=data.get('metadata') or {}; spec=data.get('spec') or {}; joints=tuple(spec.get('joints') or ()); limits=spec.get('joint_limits') or {}
    if data.get('kind')!='RobotProfile' or not meta.get('name'): raise ProfileError('无效 RobotProfile')
    if not joints or set(joints)!=set(limits): raise ProfileError('关节与限制不匹配')
    parsed={}
    for joint in joints:
        pair=limits[joint]
        if len(pair)!=2 or float(pair[0])>=float(pair[1]): raise ProfileError('无效关节限制')
        parsed[joint]=(float(pair[0]),float(pair[1]))
    hz=float(spec.get('control_frequency_hz',0))
    if hz<=0: raise ProfileError('无效控制频率')
    return RobotProfile(meta['name'],str(meta.get('version','0.0.0')),bool(spec.get('simulation')),hz,joints,parsed,frozenset(spec.get('capabilities') or ()),str(spec.get('verification','unverified')),hashlib.sha256(raw).hexdigest())
def load_safety_policy(path):
    raw=Path(path).read_bytes(); data=yaml.safe_load(raw) or {}; meta=data.get('metadata') or {}; spec=data.get('spec') or {}
    if data.get('kind')!='SafetyPolicy' or not meta.get('name'): raise ProfileError('无效 SafetyPolicy')
    maximum=int(spec.get('max_duration_ms',0))
    if maximum<=0: raise ProfileError('SafetyPolicy max_duration_ms 必须为正数')
    return SafetyPolicy(meta['name'],str(meta.get('version','0.0.0')),str(spec.get('verification','unverified')),bool(spec.get('simulation_only',False)),frozenset(spec.get('allowed_skills') or ()),maximum,hashlib.sha256(raw).hexdigest())