"""多目标 / 未知姿态抓取验收。

与既有验收的关系：
- verify_piper_pick.py        验证"无视觉真值"抓取链路；
- verify_piper_visual_pick.py 验证"单目标视觉闭环"；
- 本脚本验证"场景含多个目标时，按指定 ID 只抓取其中一个"，
  且该目标的位置与**完整 6DoF 姿态**均由视觉（RGB-D）解算，不读取仿真真值。

对每个指定目标：
1. 用该目标生成场景（目标位置严格取自配置，不被搬到参考点）；
2. 按该目标位置求解参考关节姿态（IK）并写入场景；
3. 走 Skill Runtime 调用 visual_pick(target_id)；
4. 判定：命中指定 ID + 双指接触 + 抬升位移 + 检测位置误差 + 姿态角误差。

四项判据全部来自既有体系，不新增判据语义：
命中目标（被抓起的确实是指定 ID）、双指接触、抬升位移、坐标误差阈值。
"""

import argparse
import json
import math
import sys
import time
from pathlib import Path

import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from build_piper_baseline import build_reference_poses, _resolve  # noqa: E402
from build_piper_pick_scene import build_scene  # noqa: E402
from iraf_adapters.mujoco.mujoco_backend import MujocoBackend  # noqa: E402
from iraf_core.authority import ControlAuthorityManager  # noqa: E402
from iraf_core.policy import AuthenticatedContext  # noqa: E402
from iraf_core.profile import load_robot_profile, load_safety_policy  # noqa: E402
from iraf_core.registry import SkillRegistry  # noqa: E402
from iraf_core.runtime import SkillRuntime  # noqa: E402
from iraf_core.store import SqliteExecutionStore  # noqa: E402


def _quaternion_angle_deg(first, second, symmetry_fold=1):
    """两个四元数的夹角（度）。

    - 已考虑 q 与 -q 表示同一旋转；
    - symmetry_fold 指定绕 z 轴的旋转对称阶数：
      正方体绕面法向有 90° 等价类（4 阶），
      因此 yaw 相差 90° 的整数倍其实描述同一个几何朝向，
      直接比较四元数会误报 90° 误差（实测 box_green 报 89.7°）。
    """
    a = np.asarray(first, dtype=float)
    b = np.asarray(second, dtype=float)
    a = a / float(np.linalg.norm(a))
    b = b / float(np.linalg.norm(b))
    folds = max(1, int(symmetry_fold))
    best = None
    for index in range(folds):
        angle = 2.0 * math.pi * index / folds
        half = angle / 2.0
        # 绕 z 轴旋转 angle 的四元数（wxyz）
        twist = np.array([math.cos(half), 0.0, 0.0, math.sin(half)], dtype=float)
        candidate = _quat_multiply(twist, b)
        dot = abs(float(np.dot(a, candidate)))
        dot = max(-1.0, min(1.0, dot))
        value = math.degrees(2.0 * math.acos(dot))
        if best is None or value < best:
            best = value
    return best


def _quat_multiply(a, b):
    aw, ax, ay, az = (float(value) for value in a)
    bw, bx, by, bz = (float(value) for value in b)
    return np.array(
        [
            aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
        ],
        dtype=float,
    )



def _truth_from_scene(root, scene_path):
    """从生成的 MJCF 读取各目标真值位姿（仅用于比对，不参与感知）。"""
    import mujoco

    model = mujoco.MjModel.from_xml_path(str(scene_path))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    truth = {}
    for index in range(model.nbody):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, index)
        if not name or not name.startswith("box_"):
            continue
        position = np.asarray(data.xpos[index], dtype=float).copy()
        rotation = np.asarray(data.xmat[index], dtype=float).reshape(3, 3).copy()
        truth[name] = {"position": position, "rotation": rotation}
    return truth


def _run_single_target(
    root,
    baseline_path,
    target_id,
    scene_path,
    vision_file,
    output_dir,
    duration_ms,
    args,
    baseline,
):
    """对单个目标执行一次完整抓取验收。"""
    target_cfg = baseline.get("target") or {}
    grasp_cfg = baseline.get("grasp") or {}

    # 参考姿态必须按"该目标实际所在位置"求解：
    # 多目标场景中各目标位置不同，不能沿用固定抓取点的参考姿态。
    entry = [item for item in baseline["targets"] if str(item["id"]) == target_id][0]
    xy = [float(entry["pos_m"][0]), float(entry["pos_m"][1])]
    per_target = json.loads(json.dumps(baseline))
    per_target["grasp"] = dict(grasp_cfg)
    per_target["grasp"]["finger_center_xy_m"] = xy
    # 若该目标带倾斜，参考姿态需要按顶面法向求解接近方向，
    # 否则 IK 解出的关节角与视觉给出的姿态不自洽。
    euler = [float(value) for value in (entry.get("euler_deg") or [0.0, 0.0, 0.0])]
    max_tilt = float(grasp_cfg.get("max_tilt_deg", 30.0))
    tilt_deg = max(abs(euler[0]), abs(euler[1]))
    if tilt_deg <= max_tilt:
        rotation = _euler_to_matrix(euler)
        axis = rotation[:, 2]
        if float(axis[2]) < 0:
            axis = -axis
        # 参考姿态的抓取点仍按目标顶面法向偏移（与视觉给出的姿态保持自洽），
        # 但预抓取位改用**纯竖直抬高**：若让 APPROACH 也沿倾斜法向偏移，
        # APPROACH 与 DESCEND 的 joint1 目标会不一致，下降过程中 required
        # 的 joint1 微调会被已环抱方块的夹爪摩擦锁死（实测 joint1 只走 23%）。
        per_target["grasp"]["approach_direction"] = [
            round(float(value), 9) for value in axis
        ]
        per_target["grasp"]["pregrasp_direction"] = [0.0, 0.0, 1.0]
        # 夹爪必须与目标的**面内主轴对齐**，否则张开的手指会撞到方块棱角
        # 并把目标推走（实测 50mm 方块在 25° yaw 下被推开 57mm）。
        # 目标自身 yaw = atan2(x 轴分量)，夹爪默认朝向为世界 +x，
        # 因此把夹爪 yaw 对齐到目标主轴即可消除棱角干涉。
        yaw_deg = math.degrees(math.atan2(rotation[1, 0], rotation[0, 0]))
        finger_xy = [
            float(entry["pos_m"][0]),
            float(entry["pos_m"][1]),
        ]
        # 在目标主轴方向上，以目标中心为基准把夹爪中心放到同一处，
        # 并给出期望的夹爪 yaw（由 grasp.yaw_deg 传给场景生成器/参考求解）。
        per_target["grasp"]["yaw_deg"] = round(float(yaw_deg), 6)
        per_target["grasp"]["finger_center_xy_m"] = finger_xy
        approach_mode = "pose_adaptive"
    else:
        approach_mode = "fallback_vertical"



    reference = build_reference_poses(root, per_target, target_id=target_id)
    # 多目标场景里台面上有多个方块，从"零姿态 HOME"直线运动到 APPROACH
    # 会让夹爪横扫台面、把干扰目标撞飞（实测会把目标顶到 0.25m 高空）。
    # 因此把 HOME 换成"沿接近轴再抬高一段"的解，
    # 保证 HOME -> APPROACH 基本是竖直下降，不产生水平扫掠。
    reference["home"] = _raised_home_pose(root, per_target, target_id, reference)
    reference["home_hold_mode"] = "raised_above_approach"



    scene = build_scene(
        _resolve(root, baseline["model"]["source"]),
        scene_path,
        target_id=target_id,
        half_size=float(target_cfg.get("half_size_m", 0.025)),
        config=per_target,
        reference=reference,
    )
    scene["reference_poses"] = reference
    # 抓取姿态必须在**最终场景**上校验指尖不碰台、张开手指不与任何物体接触：
    # 多目标场景台面上还有其它方块，夹爪张开时的干涉必须显式暴露，
    # 否则会表现为"方块被悄悄推走"，让误差数字失去可解释性。
    from build_piper_baseline import validate_grasp_pose

    scene["grasp_pose_validation"] = validate_grasp_pose(scene_path, per_target, reference)


    tolerance = float(scene.get("pose_tolerance_m", 0.005))
    profile = load_robot_profile(root / "profiles/piper_mujoco.yaml")
    safety = load_safety_policy(root / "profiles/safety/simulation_lab.yaml")
    authority = ControlAuthorityManager()

    # 后端要管理场景中的全部目标，但只有被选中的那个带抓取锚点约束。
    targets_config = {}
    for item in scene.get("targets") or [{"id": scene["target_id"]}]:
        targets_config[str(item["id"])] = {
            "body": str(item["id"]),
            "pose_tolerance_m": tolerance,
        }
    backend = MujocoBackend.from_config(
        {
            "model_path": str(Path(scene_path).resolve()),
            "manipulation": {"targets": targets_config, "gripper": scene["gripper"]},
        },
        profile,
        authority,
    )
    runtime = SkillRuntime(
        profile,
        safety,
        backend,
        SkillRegistry().load_directory(root / "skills"),
        authority,
        SqliteExecutionStore(":memory:"),
    )
    now = int(time.time() * 1000)
    request = {
        "request_id": "piper-multi-target-" + target_id,
        "idempotency_key": "piper-multi-target-" + target_id + "-" + str(now),
        "correlation_id": "piper-multi-target-pick",
        "skill": "visual_pick",
        "skill_version_constraint": "1.0.0",
        "parameters": {
            "target_id": target_id,
            "duration_ms": duration_ms,
            "vision_file": str(vision_file),
        },
        "deadline_unix_ms": now + 120000,
        "profile_name": profile.name,
        "profile_version": profile.version,
        "profile_digest": profile.digest,
        "safety_policy_name": safety.name,
        "safety_policy_version": safety.version,
        "safety_policy_digest": safety.digest,
        "resource_id": "piper-mujoco",
        "controller": "piper-multi-target-acceptance",
    }
    execution = runtime.execute(
        request,
        AuthenticatedContext(
            "piper-multi-target-acceptance",
            frozenset({"task.submit", "task.read"}),
            "local",
        ),
    )

    truth = _truth_from_scene(root, scene_path)
    truth_entry = truth.get(target_id) or {}
    truth_position = truth_entry.get("position")
    truth_rotation = truth_entry.get("rotation")

    evidence = ((execution.get("result") or {}).get("evidence")) or {}
    vision = evidence.get("vision") or {}
    detected_position = vision.get("vision_world_position_m")
    detected_quaternion = vision.get("vision_quaternion_wxyz")

    position_error = None
    orientation_error = None
    if truth_position is not None and detected_position is not None:
        position_error = float(
            np.linalg.norm(np.asarray(detected_position, dtype=float) - truth_position)
        )
    if truth_rotation is not None and detected_quaternion is not None:
        truth_quat = _matrix_to_quaternion(truth_rotation)
        # 目标是正方体：绕面法向存在 90° 等价类，比较姿态时必须折叠对称性，
        # 否则"同一个几何朝向"会被误判成 90° 偏差。
        orientation_error = _quaternion_angle_deg(
            truth_quat, detected_quaternion, symmetry_fold=4
        )


    max_position_error = float(
        (baseline.get("acceptance") or {}).get("max_position_error_m", 0.005)
    )
    max_orientation_error = float(
        (baseline.get("acceptance") or {}).get("max_orientation_error_deg", 10.0)
    )
    min_lift = float((baseline.get("acceptance") or {}).get("min_lift_delta_m", 0.02))
    min_force = float(
        (baseline.get("acceptance") or {}).get("min_normal_force_n", 0.2)
    )

    checks = {
        "execution_succeeded": execution.get("status") == "SUCCEEDED",
        # 命中目标：Skill 回传的 target_id 必须与请求一致。
        "target_hit": evidence.get("target_body") == target_id,
        "bilateral_contact": bool(evidence.get("bilateral_contact")),
        "force_ok": bool(evidence.get("force_ok")),
        "lifted": bool(evidence.get("lifted")),
        "lift_delta_ok": float(evidence.get("lift_delta_m") or 0.0) >= min_lift,
        "position_error_ok": (
            position_error is not None and position_error <= max_position_error
        ),
        "orientation_error_ok": (
            orientation_error is not None
            and orientation_error <= max_orientation_error
        ),
    }
    passed = all(checks.values())

    report = {
        "target_id": target_id,
        "requested_approach_axis_mode": approach_mode,
        "scene_path": str(scene_path),
        "scene_targets": [item["id"] for item in (scene.get("targets") or [])],
        "reference_approach_direction": per_target["grasp"]["approach_direction"],
        "grasp_mode": evidence.get("grasp_mode"),
        "approach_axis": evidence.get("approach_axis"),
        "detected_position_m": detected_position,
        "detected_quaternion_wxyz": detected_quaternion,
        "truth_position_m": (
            [round(float(value), 9) for value in truth_position]
            if truth_position is not None
            else None
        ),
        "position_error_m": (
            round(float(position_error), 9) if position_error is not None else None
        ),
        "orientation_error_deg": (
            round(float(orientation_error), 6)
            if orientation_error is not None
            else None
        ),
        "lift_delta_m": evidence.get("lift_delta_m"),
        "left_normal_force_n": evidence.get("left_normal_force_n"),
        "right_normal_force_n": evidence.get("right_normal_force_n"),
        "center_distance_m": (evidence.get("grasp_alignment") or {}).get(
            "center_distance_m"
        ),
        "vision_residual_m": vision.get("vision_residual_m"),
        "vision_size_m": vision.get("vision_size_m"),
        "vision_normal": vision.get("vision_normal"),
        "checks": checks,
        "passed": passed,
        "execution_status": execution.get("status"),
        "execution_reason": execution.get("reason"),
    }
    output_dir.mkdir(parents=True, exist_ok=True)
    (output_dir / (target_id + ".json")).write_text(
        json.dumps(report, ensure_ascii=True, indent=2) + "\n", encoding="utf-8"
    )
    return report


def _raised_home_pose(root, baseline, target_id, reference):
    """求一个"从 APPROACH 姿态沿接近轴再抬高"的 HOME 关节姿态。

    多目标场景里若沿用零姿态 HOME，机械臂从零姿态到 APPROACH 的直线运动
    会横扫台面并撞飞干扰目标。把 HOME 放在 APPROACH 正上方后，
    HOME -> APPROACH 变成近似竖直下降，路径不经过其它方块。
    """
    import mujoco

    from build_piper_pick_scene import build_scene
    from build_piper_baseline import (
        _geom_id,
        _joint_ids,
        _resolve,
        solve_finger_center_pose,
    )

    model_cfg = baseline["model"]
    source = _resolve(root, model_cfg["source"])
    arm_names = list(model_cfg.get("arm_joints") or [f"joint{i}" for i in range(1, 7)])
    target_cfg = baseline.get("target") or {}
    grasp_cfg = baseline.get("grasp") or {}
    solver_cfg = grasp_cfg.get("solver") or {}

    workdir = Path(__import__("tempfile").mkdtemp(prefix="piper-home-"))
    probe_scene = workdir / "home-probe.xml"
    build_scene(
        source,
        probe_scene,
        target_id=target_id,
        half_size=float(target_cfg.get("half_size_m", 0.03)),
        config=baseline,
    )
    model = mujoco.MjModel.from_xml_path(str(probe_scene))
    data = mujoco.MjData(model)
    arm_joints = _joint_ids(model, arm_names)
    left_geom = _geom_id(model, model_cfg["finger_geoms"]["left"])
    right_geom = _geom_id(model, model_cfg["finger_geoms"]["right"])

    direction = np.asarray(
        grasp_cfg.get("pregrasp_direction")
        or grasp_cfg.get("approach_direction")
        or [0.0, 0.0, 1.0],
        dtype=float,
    )
    norm = float(np.linalg.norm(direction))
    if norm < 1e-9:
        raise ValueError("grasp.pregrasp_direction 不能为零向量")
    direction = direction / norm


    approach_center = np.asarray(reference["approach"]["finger_center_m"], dtype=float)
    rise = float(grasp_cfg.get("home_rise_m", 0.12))
    home_target = approach_center + direction * rise

    # 先摆到 APPROACH 关节姿态，再求解 HOME 目标，保证迭代从相近构型出发。
    for name, value in reference["approach"]["joint_positions"].items():
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if joint_id >= 0:
            data.qpos[int(model.jnt_qposadr[joint_id])] = float(value)
    mujoco.mj_forward(model, data)
    solution = solve_finger_center_pose(
        model, data, home_target, arm_joints, arm_names, left_geom, right_geom, solver_cfg
    )
    if float(solution["position_error_m"]) > 1e-4:
        raise ValueError(
            "HOME 抬高姿态求解残差过大: "
            + str(solution["position_error_m"])
        )
    return dict(solution["joint_positions"])


def _euler_to_matrix(euler_deg):

    rx, ry, rz = (math.radians(float(value)) for value in euler_deg)
    rotation_x = np.array(
        [[1, 0, 0], [0, math.cos(rx), -math.sin(rx)], [0, math.sin(rx), math.cos(rx)]]
    )
    rotation_y = np.array(
        [[math.cos(ry), 0, math.sin(ry)], [0, 1, 0], [-math.sin(ry), 0, math.cos(ry)]]
    )
    rotation_z = np.array(
        [[math.cos(rz), -math.sin(rz), 0], [math.sin(rz), math.cos(rz), 0], [0, 0, 1]]
    )
    return rotation_z @ rotation_y @ rotation_x


def _matrix_to_quaternion(matrix):
    m = np.asarray(matrix, dtype=float).reshape(3, 3)
    trace = float(np.trace(m))
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        return np.array(
            [
                0.25 * s,
                (m[2, 1] - m[1, 2]) / s,
                (m[0, 2] - m[2, 0]) / s,
                (m[1, 0] - m[0, 1]) / s,
            ]
        )
    if m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
        return np.array(
            [
                (m[2, 1] - m[1, 2]) / s,
                0.25 * s,
                (m[0, 1] + m[1, 0]) / s,
                (m[0, 2] + m[2, 0]) / s,
            ]
        )
    if m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
        return np.array(
            [
                (m[0, 2] - m[2, 0]) / s,
                (m[0, 1] + m[1, 0]) / s,
                0.25 * s,
                (m[1, 2] + m[2, 1]) / s,
            ]
        )
    s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
    return np.array(
        [
            (m[1, 0] - m[0, 1]) / s,
            (m[0, 2] + m[2, 0]) / s,
            (m[1, 2] + m[2, 1]) / s,
            0.25 * s,
        ]
    )


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--baseline", type=Path, default=Path("config/piper_multi_target.yaml")
    )
    parser.add_argument(
        "--scene", type=Path, default=Path("build/models/piper-multi-scene.xml")
    )
    parser.add_argument(
        "--output", type=Path, default=Path("build/acceptance/piper-multi-target-pick")
    )
    parser.add_argument(
        "--vision-file",
        type=Path,
        default=Path("build/calibration/piper-vision-targets.json"),
    )
    parser.add_argument("--duration-ms", type=int, default=12000)
    parser.add_argument(
        "--targets",
        default=None,
        help="逗号分隔的目标 ID 列表；缺省为配置中的全部目标",
    )
    parser.add_argument("--root", type=Path, default=Path(__file__).resolve().parents[1])
    args = parser.parse_args(argv)
    if not 1 <= args.duration_ms <= 30000:
        parser.error("duration-ms 必须在 1..30000 之间")

    root = args.root.resolve()
    baseline = yaml.safe_load(args.baseline.read_text(encoding="utf-8"))
    target_ids = (
        [item.strip() for item in args.targets.split(",") if item.strip()]
        if args.targets
        else [str(item["id"]) for item in baseline["targets"]]
    )

    results = []
    for target_id in target_ids:
        report = _run_single_target(
            root,
            args.baseline,
            target_id,
            args.scene,
            args.vision_file,
            args.output,
            args.duration_ms,
            args,
            baseline,
        )
        results.append(report)
        print(
            "TARGET "
            + target_id
            + " passed="
            + str(report["passed"])
            + " pos_err="
            + str(report["position_error_m"])
            + " ori_err="
            + str(report["orientation_error_deg"])
            + " mode="
            + str(report["grasp_mode"])
            + " status="
            + str(report["execution_status"]),
            flush=True,
        )

    summary = {
        "schema_version": "iraf.piper-multi-target-acceptance/v1",
        "simulation_only": True,
        "baseline": str(args.baseline),
        "scene": str(args.scene),
        "vision_file": str(args.vision_file),
        "targets": results,
        "passed": all(item["passed"] for item in results),
        "summary": {
            "total": len(results),
            "passed": sum(1 for item in results if item["passed"]),
            "max_position_error_m": max(
                (
                    item["position_error_m"]
                    for item in results
                    if item["position_error_m"] is not None
                ),
                default=None,
            ),
            "max_orientation_error_deg": max(
                (
                    item["orientation_error_deg"]
                    for item in results
                    if item["orientation_error_deg"] is not None
                ),
                default=None,
            ),
        },
    }
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output / "report.json").write_text(
        json.dumps(summary, ensure_ascii=True, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(summary["summary"], ensure_ascii=True))
    return 0 if summary["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
