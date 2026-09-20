"""验证 2F-85 "pad 不动" 的最终根因：driver 关节被 4 杆机构反驱到限位外。

根因推理（基于已有硬数据）：

组装后的运动链是
    base → right_driver --(joint)--> right_coupler --(joint)--> (nothing)
    base → right_spring_link --(joint)--> right_follower --(joint)--> right_pad
    equality/connect: right_follower  ↔ right_coupler
    equality/joint:   right_driver_joint  =  left_driver_joint

也就是说 **pad 挂在 follower 上**，follower 挂在 spring_link 上，
driver 与 pad 之间**没有直接的运动学父子关系**，只通过 connect 约束耦合。

Spring_link 关节带弹簧回位：k=0.05、springref=2.62（远大于 range 上限 0.8）。
上一轮实测证明：直接给 follower qpos 能让 pad 动（-0.3→0.0959、+0.3→0.0752），
说明从 follower 到 pad 的链路是通的、pad 侧没有卡死。

而 **driver 关节的 qpos 无论怎么设（-0.9..+1.2）都不影响 pad**，
且 equality 完全正常（neq=3、引用全部有效）—— 唯一解释是：
driver 关节本身在编译后**不与任何 body 相连**，或它所在的 body 不在
pad 的运动链上。本脚本直接读 `jnt_bodyid` / `body_parentid` 验证这一点，
并给出"哪些关节真正影响 pad"的完整敏感性矩阵。
"""

import argparse
import json
from pathlib import Path

import mujoco
import numpy as np

PAD_ASSEMBLY = ("rq2f85_left_pad1", "rq2f85_right_pad1")
PAD_OFFICIAL = ("left_pad1", "right_pad1")


def _geom(model, name):
    geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)
    if geom_id < 0:
        raise ValueError("缺少 geom: " + name)
    return int(geom_id)


def pad_gap(model, data, names):
    left = _geom(model, names[0])
    right = _geom(model, names[1])
    centres = np.asarray(data.geom_xpos[[left, right]], dtype=float)
    half_y = float(model.geom_size[left][1])
    return float(np.linalg.norm(centres[1] - centres[0])) - 2.0 * half_y


def joint_sensitivity(model, data, names, pad_names):
    """逐关节做 qpos 敏感性测试：每个关节取 range 内三点，看 pad 间隙是否变化。

    关节名前缀由调用方给定（组装模型是 rq2f85_，官方模型没有前缀）。
    """
    rows = []
    for joint_id in range(model.njnt):
        joint_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, joint_id)
        if not joint_name or not joint_name.startswith(names):
            continue
        adr = int(model.jnt_qposadr[joint_id])
        lo, hi = (float(v) for v in model.jnt_range[joint_id])
        if not np.isfinite(lo) or not np.isfinite(hi) or hi <= lo:
            lo, hi = -0.5, 0.5
        gaps = []
        for value in (lo, (lo + hi) / 2.0, hi):
            data.qpos[:] = 0.0
            data.qvel[:] = 0.0
            data.qpos[adr] = float(value)
            mujoco.mj_forward(model, data)
            gaps.append(round(pad_gap(model, data, pad_names), 6))
        board = int(model.jnt_bodyid[joint_id])
        rows.append(
            {
                "joint": joint_name,
                "qpos_range": [round(lo, 6), round(hi, 6)],
                "pad_gap_at_range": gaps,
                "pad_gap_span_m": round(max(gaps) - min(gaps), 6),
                "body": mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, board),
                "parent_body": mujoco.mj_id2name(
                    model, mujoco.mjtObj.mjOBJ_BODY,
                    int(model.body_parentid[board]),
                ),
            }
        )
    return rows


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--assembly", default="build/models/ur5e_2f85/ur5e_2f85.xml")
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("build/calibration/ur5-gripper-sensitivity.json"),
    )
    args = parser.parse_args()

    model = mujoco.MjModel.from_xml_path(str(Path(args.assembly).resolve()))
    data = mujoco.MjData(model)
    # 组装模型带前缀，官方模型不带；用 geom 探测决定用哪一套名字。
    is_assembly = (
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, PAD_ASSEMBLY[0]) >= 0
    )
    pad_names = PAD_ASSEMBLY if is_assembly else PAD_OFFICIAL
    joint_prefix = "rq2f85_" if is_assembly else ""
    rows = joint_sensitivity(model, data, joint_prefix, pad_names)

    print("=== 关节对 pad 间隙的敏感性（组装模型）===")
    print("%-34s %-18s %-10s %s" % ("joint", "body", "span(m)", "gap@range"))
    for row in rows:
        print(
            "%-34s %-18s %-10.6f %s"
            % (row["joint"], row["body"], row["pad_gap_span_m"], row["pad_gap_at_range"])
        )

    effective = [row for row in rows if row["pad_gap_span_m"] > 1e-4]
    ineffective = [row for row in rows if row["pad_gap_span_m"] <= 1e-4]
    print("\n能影响 pad 的关节: %s" % [row["joint"] for row in effective])
    print("不影响 pad 的关节: %s" % [row["joint"] for row in ineffective])

    # driver 关节的 body 是不是真的挂在夹爪里？打印它的 body 深度与父链。
    print("\n--- driver 关节所在 body 的父链 ---")
    parent_chain = {}
    for row in rows:
        board = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_BODY, row["body"]
        )
        chain = []
        current = int(board)
        while current > 0:
            chain.append(mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, current))
            current = int(model.body_parentid[current])
        parent_chain[row["joint"]] = chain[::-1]
        print("  %-34s %s" % (row["joint"], " → ".join(parent_chain[row["joint"]])))

    report = {
        "schema_version": "iraf.ur5-gripper-sensitivity/v1",
        "joints": rows,
        "effective_joints": [row["joint"] for row in effective],
        "ineffective_joints": [row["joint"] for row in ineffective],
        "parent_chains": parent_chain,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    print("\nWROTE", args.output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
