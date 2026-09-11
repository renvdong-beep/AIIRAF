"""使用 MuJoCo 位置雅可比求解 Piper link6 预抓取位置 IK。"""
import argparse, json
from pathlib import Path
import mujoco
import numpy as np

def solve(model_path, target_path, output, iterations=200, body_name="link6"):
    model = mujoco.MjModel.from_xml_path(str(Path(model_path).resolve()))
    data = mujoco.MjData(model)
    target_data = json.loads(Path(target_path).read_text())
    target_value = next((target_data[key] for key in ("ik_target_position_m", "ik_link6_pregrasp_position_m", "target_position_m", "grasp_position_m", "pregrasp_position_m") if key in target_data), None)
    if target_value is None:
        raise ValueError("IK 输入缺少目标位置")
    target = np.asarray(target_value, dtype=float)
    body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
    if body < 0:
        raise ValueError("MJCF 缺少 link6")
    joint_ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"joint{i}") for i in range(1, 7)]
    qadr = [int(model.jnt_qposadr[j]) for j in joint_ids]
    for _ in range(iterations):
        mujoco.mj_forward(model, data)
        err = target - data.xpos[body]
        if float(np.linalg.norm(err)) < 1e-4:
            break
        jac = np.zeros((3, model.nv)); mujoco.mj_jacBody(model, data, jac, np.zeros((3, model.nv)), body)
        delta = 0.45 * np.linalg.pinv(jac[:, [int(model.jnt_dofadr[j]) for j in joint_ids]]) @ err
        for joint_id, adr, value in zip(joint_ids, qadr, delta):
            data.qpos[adr] += float(value)
            if model.jnt_limited[joint_id]:
                lo, hi = model.jnt_range[joint_id]
                data.qpos[adr] = float(np.clip(data.qpos[adr], lo, hi))
        mujoco.mj_normalizeQuat(model, data.qpos)
    mujoco.mj_forward(model, data)
    result = {"schema_version":"iraf.piper-ik/v1","body":body_name,"target_position_m":target.tolist(),"solved_position_m":data.xpos[body].tolist(),"position_error_m":float(np.linalg.norm(target-data.xpos[body])),"iterations":_+1,"joint_positions":{"joint%d"%(i+1):float(data.qpos[qadr[i]]) for i in range(6)}}
    Path(output).parent.mkdir(parents=True, exist_ok=True); Path(output).write_text(json.dumps(result,ensure_ascii=True,indent=2)+"\n"); print(json.dumps(result,ensure_ascii=True,indent=2))

if __name__ == "__main__":
    p=argparse.ArgumentParser(description=__doc__); p.add_argument("--model",required=True); p.add_argument("--target",required=True); p.add_argument("--output",default="build/calibration/piper-ik.json"); p.add_argument("--body",default="link6"); a=p.parse_args(); solve(a.model,a.target,a.output,body_name=a.body)
