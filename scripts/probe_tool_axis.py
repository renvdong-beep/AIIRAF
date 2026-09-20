"""判定 UR5e + 2F-85 组装的"工具轴"在世界系中的指向。

动机（`scripts/probe_contact_geometry.py` 的实测）：
- APPROACH 段 t=1278ms 首次非台面接触是 `box_01_geom <-> wrist_2_link 的胶囊碰撞体`，
  接触点在方块顶面（z≈0.0498）；
- 该瞬间夹持区中点（4 个 pad geom 中心均值）仍在 z≈0.344，pad 完全没有碰到方块；
- 也就是说**腕部 link 比 pad 更早到达方块**，物理上等价于"工具轴朝上"：
  法兰指向 +z（背离台面），于是要把 pad 送到方块高度，必须先让腕部穿到台面以下。

本脚本加载基线姿态 JSON 里的 home / approach / grasp / lift 四组关节角，
逐组做正向运动学，直接输出：
  - 法兰 site 的局部 z 轴在世界系中的方向（工具轴）；
  - 腕部 link、夹爪基座、pad 的 z 高度排序；
  - "pad 位于法兰之上还是之下"这一判定；
  - 若要把 pad 降到方块中心高度，法兰需要到达的 z（可行性判据）。

用法：
  python3 scripts/probe_tool_axis.py --scene build/models/ur5-pick-scene.xml \
      --poses build/calibration/ur5-baseline-pose.json \
      --flange-site attachment_site
"""

import argparse
import json
from pathlib import Path

import mujoco
import numpy as np


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--scene", default="build/models/ur5-pick-scene.xml")
    parser.add_argument("--poses", default="build/calibration/ur5-baseline-pose.json")
    parser.add_argument("--flange-site", default="attachment_site")
    parser.add_argument("--pads", nargs="+", default=None)
    args = parser.parse_args()

    root = Path(__file__).resolve().parents[1]
    scene_xml = root / args.scene
    scene = json.loads(scene_xml.with_suffix(".json").read_text(encoding="utf-8"))
    poses = json.loads((root / args.poses).read_text(encoding="utf-8"))

    pad_names = args.pads or list(scene["finger_geoms"]["pad_boxes"])
    model = mujoco.MjModel.from_xml_path(str(scene_xml))
    data = mujoco.MjData(model)

    def jid(name):
        return int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name))

    def bid(name):
        return int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name))

    def gid(name):
        return int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name))

    site_id = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, args.flange_site))
    if site_id < 0:
        raise SystemExit("找不到法兰 site: " + args.flange_site)

    wrist_2 = bid("wrist_2_link")
    wrist_3 = bid("wrist_3_link")
    base_mount = bid("rq2f85_base_mount")
    box_body = bid(scene["target_id"])
    pad_geoms = [gid(n) for n in pad_names]
    target_z = float(scene["target_position"]["z"])

    print("方块中心 z = %.5f m，方块顶面 z = %.5f m" % (target_z, target_z + scene["target_half_size_m"]))
    print()
    header = "%-9s %-25s %-11s %-11s %-11s %-11s %-11s %-6s"
    print(
        header
        % (
            "姿态",
            "法兰局部 z 轴(世界系)",
            "法兰 z",
            "wrist_2 z",
            "wrist_3 z",
            "基座 z",
            "pad 中点 z",
            "轴指向",
        )
    )
    rows = []
    for name in ("home", "approach", "grasp", "lift", "home_solved"):
        entry = poses.get(name)
        if not entry:
            continue
        # 两种结构都兼容：扁平 {关节: 角} 与 {joint_positions: {关节: 角}, ...}
        positions = entry.get("joint_positions", entry)
        qpos = np.array(data.qpos, dtype=float)
        for joint_name, value in positions.items():
            index = jid(joint_name)
            if index < 0:
                # 执行器名（如夹爪通道）跳过：本探针只关心臂关节几何。
                continue
            qpos[int(model.jnt_qposadr[index])] = float(value)
        data.qpos[:] = qpos
        mujoco.mj_forward(model, data)

        site_pos = np.asarray(data.site_xpos[site_id], dtype=float)
        site_z_axis = np.asarray(data.site_xmat[site_id].reshape(3, 3)[:, 2], dtype=float)
        pad_center = np.mean([np.asarray(data.geom_xpos[g]) for g in pad_geoms], axis=0)
        wrist_2_z = float(data.xpos[wrist_2][2])
        wrist_3_z = float(data.xpos[wrist_3][2])
        mount_z = float(data.xpos[base_mount][2])
        tool_axis_up = float(pad_center[2] - site_pos[2]) > 0
        # 工具轴（法兰 z）朝上时，pad 在法兰之上；朝下时在法兰之下。
        print(
            "%-9s [%+.4f %+.4f %+.4f] %-11.5f %-11.5f %-11.5f %-11.5f %-11.5f %-6s"
            % (
                name,
                site_z_axis[0],
                site_z_axis[1],
                site_z_axis[2],
                site_pos[2],
                wrist_2_z,
                wrist_3_z,
                mount_z,
                pad_center[2],
                "朝上" if tool_axis_up else "朝下",
            )
        )
        # 要把 pad 中点送到方块中心高度，法兰必须到哪个 z：
        # pad 相对法兰的 z 偏置在刚体假设下是常量。
        offset_pad_from_flange = float(pad_center[2] - site_pos[2])
        flange_z_needed = target_z - offset_pad_from_flange
        rows.append(
            {
                "pose": name,
                "flange_z_axis_world": [round(float(v), 6) for v in site_z_axis],
                "flange_position_m": [round(float(v), 6) for v in site_pos],
                "wrist_2_z_m": round(wrist_2_z, 6),
                "wrist_3_z_m": round(wrist_3_z, 6),
                "base_mount_z_m": round(mount_z, 6),
                "pad_center_m": [round(float(v), 6) for v in pad_center],
                "pad_offset_from_flange_z_m": round(offset_pad_from_flange, 6),
                "tool_axis_points": "up" if tool_axis_up else "down",
                "flange_z_to_place_pad_at_target_m": round(flange_z_needed, 6),
            }
        )
    print()
    for row in rows:
        print(
            "%-9s pad 相对法兰 z 偏置 = %+.5f m  →  pad 落到方块中心(z=%.4f)需法兰 z=%.5f"
            % (
                row["pose"],
                row["pad_offset_from_flange_z_m"],
                target_z,
                row["flange_z_to_place_pad_at_target_m"],
            )
        )
    print()
    print(json.dumps(rows, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
