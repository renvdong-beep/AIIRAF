"""验证"重力前馈"能否把纯 PD 执行器的稳态静差压到验收容差以内。

背景（本次实测）：
- Menagerie 的 UR5e 执行器是 `<general gainprm=2000 biasprm=[0,-2000,-400]>`，
  即**纯 PD、没有真实 UR 控制器的重力/惯量前馈**；
- 于是在重力矩不为零的位形下必须靠稳态位置误差平衡：Δq = τ_g / gain；
- 工具轴翻正后三段跟踪误差已从 0.017 / 0.191 / 0.484 rad 降到
  0.014 / 0.014 / 0.015 rad（方块全程未位移、无碰撞），
  但 DESCEND 末端的 0.015 rad 仍折算成末端 16.6mm 偏差，超过 5mm 验收容差。

本探针读取 `build/calibration/ur5-baseline-pose.json` 里由构建器算好的
`gravity_feedforward`（四姿态各自的 ctrl 增量），在同一场景上做两组仿真：
  A) 基线：ctrl = 目标关节角（当前实现）
  B) 前馈：ctrl = 目标关节角 + 该姿态的重力前馈增量
逐段比较"夹持区中点 → 目标点"的实际偏差，给出是否进入容差的结论。

注意：`gravity_feedforward` 的键是**关节名**，而场景里 `*_positions` 的键是
**执行器名**（UR5e 两者不同名），必须按执行器 → 驱动关节反查后取用，
否则前馈会静默失效（本探针第一版就踩了这个坑，两组结果完全一致）。

用法：
  python3 scripts/probe_gravity_feedforward.py \
      --scene build/models/ur5-pick-scene.xml \
      --poses build/calibration/ur5-baseline-pose.json \
      --duration-ms 12000
"""

import argparse
import json
import math
from pathlib import Path

import mujoco
import numpy as np


def quintic(start, end, duration_s, t):
    if t >= duration_s:
        return end
    x = t / duration_s
    return start + (end - start) * (10 * x**3 - 15 * x**4 + 6 * x**5)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--scene", default="build/models/ur5-pick-scene.xml")
    parser.add_argument("--poses", default="build/calibration/ur5-baseline-pose.json")
    parser.add_argument("--duration-ms", type=int, default=12000)
    parser.add_argument("--tolerance-m", type=float, default=0.005)
    parser.add_argument(
        "--output", default="build/calibration/ur5-gravity-feedforward.json"
    )
    args = parser.parse_args()

    root = Path(__file__).resolve().parents[1]
    scene_path = root / args.scene
    scene = json.loads(scene_path.with_suffix(".json").read_text(encoding="utf-8"))
    poses = json.loads((root / args.poses).read_text(encoding="utf-8"))
    gripper = scene["gripper"]
    feedforward = poses["gravity_feedforward"]

    model = mujoco.MjModel.from_xml_path(str(scene_path))
    pad_geoms = [
        int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name))
        for name in scene["finger_geoms"]["pad_boxes"]
    ]
    box_body = int(
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, scene["target_id"])
    )

    def actuator_index(name):
        index = int(
            mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, str(name))
        )
        if index < 0:
            raise SystemExit("执行器不存在: " + str(name))
        return index

    def driven_joint_name(name):
        index = actuator_index(name)
        joint_id = int(model.actuator_trnid[index, 0])
        joint = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, joint_id)
        return joint if joint else ""

    def run(apply_feedforward):
        data = mujoco.MjData(model)
        if int(model.nkey) > 0:
            mujoco.mj_resetDataKeyframe(model, data, 0)
        mujoco.mj_forward(model, data)
        segment_ms = max(1, args.duration_ms // 5)
        dt = float(model.opt.timestep)
        results = {}

        def grip_center():
            return np.mean(
                [np.asarray(data.geom_xpos[g]) for g in pad_geoms], axis=0
            )

        for phase, key in (
            ("HOME_HOLD", "home"),
            ("APPROACH", "approach"),
            ("DESCEND", "grasp"),
        ):
            targets = gripper[key + "_positions"]
            names = list(targets)
            aids = {name: actuator_index(name) for name in names}
            # 关节名 → 执行器名 的映射：前馈表的键是关节名
            offsets = {}
            if apply_feedforward:
                for name in names:
                    joint = driven_joint_name(name)
                    offsets[name] = float(feedforward[key].get(joint, 0.0))
            starts = [
                float(data.qpos[int(model.jnt_qposadr[int(model.actuator_trnid[aids[n], 0])])])
                for n in names
            ]
            steps = max(1, int(math.ceil(segment_ms / 1000.0 / dt)))
            for step in range(1, steps + 1):
                elapsed = step * dt
                for i, name in enumerate(names):
                    value = quintic(
                        starts[i], float(targets[name]), segment_ms / 1000.0, elapsed
                    )
                    data.ctrl[aids[name]] = value + offsets.get(name, 0.0)
                mujoco.mj_step(model, data)
            settle_ms = max(250, min(16000, segment_ms * 4))
            for _ in range(max(1, int(math.ceil(settle_ms / 1000.0 / dt)))):
                for name in names:
                    data.ctrl[aids[name]] = float(targets[name]) + offsets.get(name, 0.0)
                mujoco.mj_step(model, data)
            mujoco.mj_forward(model, data)
            entry = poses[key]
            target = entry.get("target_m") or poses[key + "_solved"]["target_m"]
            target = np.asarray(target, dtype=float)
            center = grip_center()
            delta = center - target
            results[phase] = {
                "grip_center_m": [round(float(v), 6) for v in center],
                "target_m": [round(float(v), 6) for v in target],
                "error_m": round(float(np.linalg.norm(delta)), 6),
                "delta_m": [round(float(v), 6) for v in delta],
                "box_position_m": [
                    round(float(v), 6) for v in data.xpos[box_body]
                ],
                "passed": bool(np.linalg.norm(delta) <= args.tolerance_m),
            }
        return results

    baseline = run(False)
    corrected = run(True)

    print("重力前馈（关节名 → ctrl 增量 rad，仅列非零项）:")
    for pose, offsets in feedforward.items():
        nonzero = {n: round(float(v), 5) for n, v in offsets.items() if abs(v) > 1e-6}
        print("  %-9s %s" % (pose, nonzero))
    print()
    header = "%-10s %-16s %-16s %-10s"
    print(header % ("段", "无前馈误差(m)", "有前馈误差(m)", "容差内"))
    for phase in baseline:
        print(
            header
            % (
                phase,
                "%.6f" % baseline[phase]["error_m"],
                "%.6f" % corrected[phase]["error_m"],
                "是" if corrected[phase]["passed"] else "否",
            )
        )
    print()
    print("有前馈时的目标点偏差分量:")
    for phase, entry in corrected.items():
        print(
            "  %-10s delta=%s  box=%s"
            % (phase, entry["delta_m"], entry["box_position_m"])
        )

    output = root / args.output
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(
        json.dumps(
            {
                "schema_version": "iraf.ur5-gravity-feedforward/v1",
                "scene": str(scene_path),
                "poses": str(root / args.poses),
                "tolerance_m": args.tolerance_m,
                "feedforward": feedforward,
                "feedforward_evidence": poses.get("gravity_feedforward_evidence"),
                "baseline": baseline,
                "corrected": corrected,
            },
            ensure_ascii=False,
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )
    worst = max(entry["error_m"] for entry in corrected.values())
    print()
    print("证据: %s" % output)
    print(
        "结论: 有前馈时三段最大末端偏差 %.6f m（容差 %.6f m）→ %s"
        % (worst, args.tolerance_m, "通过" if worst <= args.tolerance_m else "未通过")
    )
    return 0 if worst <= args.tolerance_m else 1


if __name__ == "__main__":
    raise SystemExit(main())
