from pathlib import Path
import hashlib,yaml
from .core import RobotProfile,SafetyPolicy
class ProfileError(ValueError): pass
def _parse_joint_roles(spec, joints):
    """解析关节语义；未声明时返回 None，调用方回退平铺语义。"""
    raw = spec.get('joint_roles')
    if raw is None:
        return None
    if not isinstance(raw, dict) or not raw:
        raise ProfileError('joint_roles 必须是非空对象')
    allowed = {'arm', 'gripper_drive_left', 'gripper_drive_right', 'gripper_drive'}
    parsed_roles = {}
    for joint, role in raw.items():
        if str(joint) not in set(joints):
            raise ProfileError('joint_roles 引用了未声明的关节: ' + str(joint))
        if str(role) not in allowed:
            raise ProfileError('未知关节角色: ' + str(role))
        parsed_roles[str(joint)] = str(role)
    return parsed_roles


def _parse_gripper(spec, joints, joint_roles):
    """解析夹爪结构；未声明时返回 None。"""
    raw = spec.get('gripper')
    if raw is None:
        return None
    if not isinstance(raw, dict):
        raise ProfileError('gripper 必须是对象')
    drive = raw.get('drive_joints')
    if not isinstance(drive, (list, tuple)) or len(drive) != 2:
        raise ProfileError('gripper.drive_joints 必须恰好声明两个驱动关节')
    drive = [str(item) for item in drive]
    declared = set(joints)
    for joint in drive:
        if joint not in declared:
            raise ProfileError('gripper.drive_joints 引用了未声明的关节: ' + joint)
    if drive[0] == drive[1]:
        raise ProfileError('gripper.drive_joints 的两个关节不能相同')
    for key in ('open_positions', 'closed_positions'):
        block = raw.get(key)
        if not isinstance(block, dict) or set(str(k) for k in block) != set(drive):
            raise ProfileError(
                'gripper.' + key + ' 必须与 drive_joints 完全对应'
            )
    pad = raw.get('pad_offset_m')
    if pad is not None:
        pad = float(pad)
        if not pad >= 0:
            raise ProfileError('gripper.pad_offset_m 必须是非负数')
    extra = {}
    for key in ('pad_offset_axis', 'max_tilt_deg'):
        if key not in raw or raw[key] is None:
            continue
        if key == 'pad_offset_axis':
            axis = list(raw[key])
            if len(axis) != 3:
                raise ProfileError('gripper.pad_offset_axis 必须是 3 个数值')
            extra[key] = [float(v) for v in axis]
        else:
            value = float(raw[key])
            if not 0.0 <= value <= 90.0:
                raise ProfileError('gripper.max_tilt_deg 必须在 0..90 之间')
            extra[key] = value
    return {
        'drive_joints': drive,
        'left_index': int(raw.get('left_index', 0)),
        'right_index': int(raw.get('right_index', 1)),
        'open_positions': {
            str(k): float(v) for k, v in raw['open_positions'].items()
        },
        'closed_positions': {
            str(k): float(v) for k, v in raw['closed_positions'].items()
        },
        'pad_offset_m': pad,
        **extra,
    }


def _parse_home(spec, joints):
    """解析 Home 位姿；未声明时返回 None。"""
    raw = spec.get('home')
    if raw is None:
        return None
    if not isinstance(raw, dict) or not raw:
        raise ProfileError('home 必须是非空对象')
    declared = set(joints)
    parsed_home = {}
    for joint, value in raw.items():
        if str(joint) not in declared:
            raise ProfileError('home 引用了未声明的关节: ' + str(joint))
        parsed_home[str(joint)] = float(value)
    return parsed_home


def _parse_camera(spec):
    """解析相机初始视角；未声明时返回 None。"""
    raw = spec.get('camera')
    if raw is None:
        return None
    if not isinstance(raw, dict):
        raise ProfileError('camera 必须是对象')
    lookat = raw.get('lookat_m')
    if not isinstance(lookat, (list, tuple)) or len(lookat) != 3:
        raise ProfileError('camera.lookat_m 必须是 3 个数值')
    return {
        'lookat_m': [float(v) for v in lookat],
        'distance_m': float(raw.get('distance_m', 1.0)),
        'azimuth_deg': float(raw.get('azimuth_deg', 180.0)),
        'elevation_deg': float(raw.get('elevation_deg', -8.0)),
    }


def _parse_manipulation(spec):
    """解析目标与容差；未声明时返回 None。"""
    raw = spec.get('manipulation')
    if raw is None:
        return None
    if not isinstance(raw, dict):
        raise ProfileError('manipulation 必须是对象')
    return {str(k): v for k, v in raw.items()}


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
    return RobotProfile(
        meta['name'],
        str(meta.get('version', '0.0.0')),
        bool(spec.get('simulation')),
        hz,
        joints,
        parsed,
        frozenset(spec.get('capabilities') or ()),
        str(spec.get('verification', 'unverified')),
        hashlib.sha256(raw).hexdigest(),
        _parse_joint_roles(spec, joints),
        _parse_gripper(spec, joints, spec.get('joint_roles')),
        _parse_home(spec, joints),
        _parse_camera(spec),
        _parse_manipulation(spec),
    )
def load_safety_policy(path):
    raw=Path(path).read_bytes(); data=yaml.safe_load(raw) or {}; meta=data.get('metadata') or {}; spec=data.get('spec') or {}
    if data.get('kind')!='SafetyPolicy' or not meta.get('name'): raise ProfileError('无效 SafetyPolicy')
    maximum=int(spec.get('max_duration_ms',0))
    if maximum<=0: raise ProfileError('SafetyPolicy max_duration_ms 必须为正数')
    return SafetyPolicy(meta['name'],str(meta.get('version','0.0.0')),str(spec.get('verification','unverified')),bool(spec.get('simulation_only',False)),frozenset(spec.get('allowed_skills') or ()),maximum,hashlib.sha256(raw).hexdigest())