"""根据 Piper 抓取标定结果生成预抓取位姿。"""
import argparse
import json
from pathlib import Path


def plan(calibration, output):
    data = json.loads(Path(calibration).read_text())
    center = data["grasp_center_m"]
    axis = data["approach_axis_world"]
    tcp = data["tcp_offset_from_link6_m"]
    offset = float(data["recommended_pregrasp_offset_m"])
    pregrasp = [float(center[i]) - float(axis[i]) * offset for i in range(3)]
    ik_target = [pregrasp[i] - float(tcp[i]) for i in range(3)]
    result = {
        "schema_version": "iraf.piper-pregrasp/v1",
        "frame_id": "world",
        "grasp_position_m": center,
        "pregrasp_position_m": pregrasp,
        "ik_link6_pregrasp_position_m": ik_target,
        "approach_axis_world": axis,
        "offset_m": offset,
        "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
        "ik_required": True,
    }
    Path(output).parent.mkdir(parents=True, exist_ok=True)
    Path(output).write_text(json.dumps(result, ensure_ascii=True, indent=2) + "\n")
    print(json.dumps(result, ensure_ascii=True, indent=2))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--calibration", type=Path, required=True)
    parser.add_argument("--output", type=Path, default=Path("build/calibration/piper-pregrasp.json"))
    args = parser.parse_args()
    plan(args.calibration, args.output)
