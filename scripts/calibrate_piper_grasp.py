"""从 Piper MJCF 自动计算末端和双指夹持标定参数。"""

import argparse
import json
from pathlib import Path

import mujoco


def calibrate(model_path, output):
    model = mujoco.MjModel.from_xml_path(str(Path(model_path).resolve()))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    def body(name):
        ident = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
        if ident < 0:
            raise ValueError("Piper MJCF 缺少 body: " + name)
        return int(ident)

    link6, left, right = body("link6"), body("link7"), body("link8")
    wrist = data.xpos[link6].copy()
    def geom_position(name, fallback_body):
        ident = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)
        return data.geom_xpos[ident].copy() if ident >= 0 else data.xpos[fallback_body].copy()

    left_pos = geom_position("piper_left_finger", left)
    right_pos = geom_position("piper_right_finger", right)
    midpoint = (left_pos + right_pos) / 2.0
    approach = midpoint - wrist
    norm = float((approach @ approach) ** 0.5)
    if norm < 1e-9:
        raise ValueError("link6 到指尖中点距离过小，无法标定")
    approach /= norm
    result = {
        "schema_version": "iraf.piper-grasp-calibration/v1",
        "model": str(Path(model_path).resolve()),
        "bodies": {"wrist": "link6", "left_finger": "link7", "right_finger": "link8"},
        "wrist_position_m": [round(float(v), 9) for v in wrist],
        "left_finger_position_m": [round(float(v), 9) for v in left_pos],
        "right_finger_position_m": [round(float(v), 9) for v in right_pos],
        "grasp_center_m": [round(float(v), 9) for v in midpoint],
        "finger_separation_m": round(float(((left_pos - right_pos) ** 2).sum() ** 0.5), 9),
        "position_source": "geom_xpos",
        # 通用字段名（腕部 body 由配置声明）；旧名 tcp_offset_from_link6_m 已移除。
        "tcp_offset_from_wrist_m": [round(float(v), 9) for v in (midpoint - wrist)],
        "approach_axis_world": [round(float(v), 9) for v in approach],
        "recommended_pregrasp_offset_m": 0.04,
    }
    Path(output).parent.mkdir(parents=True, exist_ok=True)
    Path(output).write_text(json.dumps(result, ensure_ascii=True, indent=2) + "\n")
    print(json.dumps(result, ensure_ascii=True, indent=2))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--output", type=Path, default=Path("build/calibration/piper-grasp.json"))
    args = parser.parse_args()
    calibrate(args.model, args.output)
