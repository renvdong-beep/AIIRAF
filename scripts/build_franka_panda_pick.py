import json
from pathlib import Path
import yaml
import mujoco

repo = Path('/home/coretek/AIIRAF')
model_path = str(repo / 'vendor/franka_emika_panda/mjx_single_cube.xml')
m = mujoco.MjModel.from_xml_path(model_path)
key = {mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_KEY, i): i for i in range(m.nkey)}
home = [float(x) for x in m.key_qpos[key['home']]]
pick = [float(x) for x in m.key_qpos[key['pickup']]]
pick1 = [float(x) for x in m.key_qpos[key['pickup1']]]
joints = [f'joint{i}' for i in range(1, 8)]
arm = {n: pick[i] for i, n in enumerate(joints)}
lift = {n: pick1[i] for i, n in enumerate(joints)}
lift['joint2'] -= 0.15
homep = {n: home[i] for i, n in enumerate(joints)}
boxq = pick[9:16]

report = {
    'schema': 'iraf.scene-report/v1',
    'output': model_path,
    'pose_tolerance_m': 0.03,
    'target_id': 'box',
    'targets': [{'id': 'box', 'body': 'box', 'geom': 'box',
                 'position_m': boxq[:3]}],
    'gripper': {
        'wrist_body': 'hand',
        'left_finger_body': 'left_finger', 'right_finger_body': 'right_finger',
        'left_finger_geom': 'left_finger_pad', 'right_finger_geom': 'right_finger_pad',
        'open_positions': {'actuator8': 0.04},
        'closed_positions': {'actuator8': 0.0},
        'home_positions': homep,
        'approach_positions': arm,
        'grasp_positions': arm,
        'lift_positions': dict(lift, actuator8=0.0),
    },
}
outdir = repo / 'build/models/franka_panda_pick'
outdir.mkdir(parents=True, exist_ok=True)
rp = outdir / 'panda_mjx_pick.json'
rp.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
profile = {
    'apiVersion': 'iraf.intewell.io/v1', 'kind': 'RobotProfile',
    'metadata': {'name': 'franka_panda_mujoco', 'version': '0.1.0'},
    'spec': {'simulation': True, 'verification': 'development',
             'control_frequency_hz': 100,
             'joints': joints,
             'joint_limits': {n: [float(m.jnt_range[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)][0]),
                                  float(m.jnt_range[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)][1])]
                              for n in joints},
             'capabilities': ['move_joint', 'pick_object', 'gripper', 'stop', 'reset_scene']},
}
(repo / 'profiles/franka_panda_mujoco.yaml').write_text(
    yaml.safe_dump(profile, allow_unicode=True, sort_keys=False), encoding='utf-8')
print('report', rp, 'boxq', boxq)
