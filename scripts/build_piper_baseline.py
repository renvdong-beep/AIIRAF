"""按受控基线配置求解 Piper 参考关节姿态并构建抓取场景。

模型来源、参考姿态、工作台几何和验收条件全部来自
config/piper_simulation_baseline.yaml，避免在脚本里散落硬编码。
"""

import argparse
import hashlib
import json
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np
import yaml

from build_piper_pick_scene import build_scene
from build_robot_baseline import (  # noqa: E402  通用编排：与机型无关，只按声明调用
    derive_grasp_for_target,
    raised_home_pose,
)
from iraf_core.kinematics import (
    balance_tip_clearance,
    gravity_hold_ctrl,
    lowest_mesh_point_z,
    solve_position_ik,
)

DEFAULT_BASELINE = "config/piper_simulation_baseline.yaml"

#: 四个参考相位 → 场景报告里对应的**位置指令键**（两处必须枚举一致：
#: 相位名用于参考姿态字典，位置键用于 `gripper.*_positions`）。
REFERENCE_PHASE_KEYS = (
    ("home", "home_positions"),
    ("approach", "approach_positions"),
    ("grasp", "grasp_positions"),
    ("lift", "lift_positions"),
)


def build_reference_feedforward(model, reference, baseline, prefix="", gripper_positions=None):
    """在**给定模型**上重算四个参考姿态的重力前馈（相位 → {模型内关节名: ctrl 增量}）。

    为什么单独成函数（2026-09-24，联合世界）：增量 = τ_g / kp，对**执行器增益**极敏感，
    而"臂自己场景"与"联合模型里的臂"是两个模型：

    | 关节 | 联合模型（Profile 声明的厂商 MJCF） | 臂自己场景（注入声明增益） |
    | --- | --- | --- |
    | joint1 | 10000 | 450 |
    | joint2 | 2000 | 200 |
    | joint3 | 500 | 200 |
    | joint4 | 50 | 200 |
    | joint5 | 20 | 200 |
    | joint6 | 5 | 200 |

    ⇒ 直接继承臂侧前馈会按增益比例失真（实测最大差 0.016225157 rad，在 joint5）。
    把本函数暴露成**公开入口**，使联合构建器能按声明
    （`scenes/<id>/scene.yaml: robots[].reference_solver.feedforward_entry`）在**运行期同款模型**上重算，
    与 pad_offset 的重解同一个理由：场景专属数据不得静默继承。

    契约（声明侧写 `feedforward_entry: build_reference_feedforward`）：
      输入 `model`     —— **运行期同款模型**（执行器增益必须与它一致；本场景即联合产物）
           `gripper_positions` ——（可选）{相位: {**模型内**夹爪关节名: 值}}：**必须**给出。
             为什么（2026-09-24 实测踩坑）：只设定臂关节时，夹爪 joint7/8 停在模型默认/关键帧状态，
             闭合的指腹会**卡进 50 mm 方块**（实测指腹张开向量模长 0.020378284 m vs 张开 0.090362481 m，
             指腹与方块重叠 −0.033339/−0.032634 m）⇒ 接触力把臂顶离姿态，量出来的是"手指卡住的动力学"，
             不是"重力下的保持"。判别法就是量张开向量模长。
           `reference` —— `build_reference_poses` 的返回值（同一份参考姿态）
           `baseline`  —— `reference_solver.baseline` 指向的声明文档（读 hold_ms/tolerance_rad）
           `prefix`    —— 联合模型里附加本体的名字前缀（关节名 = prefix + 声明关节名）
      输出 ({相位: {**模型内**关节名: 增量}}, 证据字典)。键必须是模型内关节名：
      后端按报告键直接寻址执行器，前缀漏掉会让前馈静默失效（表现为"精度莫名不达标"）。
    """
    ff_cfg = baseline.get("gravity_feedforward") or {}
    hold_ms = ff_cfg.get("hold_ms")
    tolerance_rad = ff_cfg.get("tolerance_rad")
    if not isinstance(hold_ms, int) or isinstance(hold_ms, bool) or hold_ms <= 0:
        raise ValueError("基线必须声明 gravity_feedforward.hold_ms（正整数）；实现层不写默认值")
    if not isinstance(tolerance_rad, (int, float)) or isinstance(tolerance_rad, bool) or tolerance_rad <= 0:
        raise ValueError("基线必须声明 gravity_feedforward.tolerance_rad（正数）；实现层不写默认值")
    prefix = str(prefix or "")
    feedforward = {}
    feedforward_evidence = {}
    for phase, _key in REFERENCE_PHASE_KEYS:
        pose = reference.get(phase)
        if pose is None:
            continue
        # ⚠ 两种形状：`home` 是扁平 {关节名: 值}，其余是 {"joint_positions": {...}}
        raw = pose if phase == "home" else (pose.get("joint_positions") or {})
        arm_here = {prefix + str(name): float(value) for name, value in (raw or {}).items()}
        if not arm_here:
            continue
        # `hold_positions`（要被**设定状态**的关节）＝ 臂关节 + 该相位的夹爪指令；
        # 只对 `arm_here` 算补偿（夹爪通道的补偿口径由机型自己决定，不在这里静默加通道）。
        hold_positions = dict(arm_here)
        for name, value in ((gripper_positions or {}).get(phase) or {}).items():
            hold_positions.setdefault(str(name), float(value))
        compensation, evidence = gravity_hold_ctrl(
            model, list(arm_here), hold_positions,
            hold_ms=int(hold_ms), tolerance_rad=float(tolerance_rad),
        )
        feedforward[phase] = compensation
        feedforward_evidence[phase] = evidence
    return feedforward, feedforward_evidence


def build_place_reference_poses(root, baseline, target_local_m, payload_half_m, pad_offset_m,
                                clearance_m):
    """按**臂基座系**里的接收体目标解出放置四段（关节空间），供后端**回放**。

    为什么是构建期解而不是运行期 IK（2026-09-28，docs/debug/2026-09-24-joint-model-dog-arm.md §11.16）：
    已验证的 `pick_object` 之所以能把方块抬 88 mm，是因为它的每一段位形都由**构建期求解器**
    按目标几何解出（`grasp_positions → lift_positions`），后端只做 `_move_trajectory` 回放；
    而运行时"现解 IK"（`pad_mid + Δ`）会落到别的分支、轨迹中途把方块打掉。放置段照同一条路做。

    输入（都由声明/报告给出，函数内不写死任何数字）：
      `target_local_m` —— 接收体**承载面中心**在臂基座系里的坐标；
      `payload_half_m` —— 载荷的半高（方块半边长，来自场景/基线声明）；
      `pad_offset_m`   —— 指腹中点相对抓取点的高度（= 本场景已解出的 `finger_height_correction_m`）；
      `clearance_m`    —— 接近/抬离的净间隙（取声明的 `grasp.pregrasp_offset_m`）。
    输出：`{"above": {...}, "descend": {...}, "retreat": {...}}`，每段是与
    `build_reference_poses` 同形状的 `_pack_pose` 结果（`joint_positions` 为臂关节解）。
    """
    model_cfg = baseline["model"]
    source = _resolve(root, model_cfg["source"])
    arm_names = list(model_cfg.get("arm_joints") or ["joint%d" % i for i in range(1, 7)])
    target_cfg = baseline.get("target") or {}
    grasp_cfg = baseline.get("grasp") or {}
    workdir = Path(tempfile.mkdtemp(prefix="piper-place-"))
    probe_scene = workdir / "probe-scene.xml"
    build_scene(source, probe_scene, target_id=target_cfg.get("id", "box_01"),
                half_size=float(target_cfg.get("half_size_m", 0.03)), config=baseline)
    model = mujoco.MjModel.from_xml_path(str(probe_scene))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    arm_joints = _joint_ids(model, arm_names)
    left_geom = _geom_id(model, model_cfg["finger_geoms"]["left"])
    right_geom = _geom_id(model, model_cfg["finger_geoms"]["right"])
    solver_cfg = grasp_cfg.get("solver") or {}
    base = np.asarray(target_local_m, dtype=float)
    if base.shape != (3,):
        raise ValueError("target_local_m 必须是 3 个数值（臂基座系）")
    height = float(pad_offset_m) + float(payload_half_m)      # 指腹中点相对承载面的高度
    plan = {
        "above": base + np.asarray([0.0, 0.0, height + float(clearance_m)]),
        "descend": base + np.asarray([0.0, 0.0, height]),
        "retreat": base + np.asarray([0.0, 0.0, height + float(clearance_m)]),
    }
    out = {"schema_version": "iraf.piper-reference-pose/v1",
           "place_mode": "declared_offset",
           "target_local_m": [round(float(v), 9) for v in base],
           "payload_half_m": float(payload_half_m), "pad_offset_m": float(pad_offset_m),
           "clearance_m": float(clearance_m), "solver": dict(solver_cfg),
           "poses": {}}
    for phase, target in plan.items():
        result = solve_finger_center_ik(model, data, target, arm_joints, left_geom, right_geom,
                                        solver_cfg)
        if float(result.position_error_m) > float(solver_cfg.get("tolerance_m", 1e-5)) * 100.0:
            raise ValueError("放置段 %s 的 IK 未收敛: error=%.9f m target=%s"
                             % (phase, float(result.position_error_m),
                                [round(float(v), 6) for v in target]))
        out["poses"][phase] = _pack_pose(result, arm_names)
    return out


def load_baseline(path):
    data = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError("基线配置格式无效: " + str(path))
    return data


def _resolve(root, value):
    candidate = Path(value)
    if not candidate.is_absolute():
        candidate = (root / candidate).resolve()
    return candidate


def _sha256(path):
    digest = hashlib.sha256()
    with open(str(path), "rb") as handle:
        for chunk in iter(lambda: handle.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _collect_model_files(source):
    files = [source]
    root = ET.parse(str(source)).getroot()
    for mesh in root.findall("./asset/mesh"):
        file_value = mesh.get("file")
        if not file_value:
            continue
        candidate = Path(file_value)
        if not candidate.is_absolute():
            candidate = (source.parent / candidate).resolve()
        files.append(candidate)
    return files


def verify_source_lock(root, baseline):
    """校验 Piper 模型来源并维护 source-lock.json。"""
    model_cfg = baseline.get("model") or {}
    source = _resolve(root, model_cfg.get("source"))
    if not source.is_file():
        raise FileNotFoundError("Piper MJCF 不存在: " + str(source))

    files = _collect_model_files(source)
    missing = [str(path) for path in files if not path.is_file()]
    if missing:
        raise FileNotFoundError("Piper 模型文件缺失: " + ", ".join(missing))

    lock_path = _resolve(root, model_cfg.get("source_lock") or "vendor/agilex_piper/source-lock.json")
    entries = []
    for path in files:
        entries.append(
            {
                "path": str(path),
                "relative_path": str(Path(path).relative_to(source.parent.parent)) if str(path).startswith(str(source.parent.parent)) else Path(path).name,
                "bytes": int(path.stat().st_size),
                "sha256": _sha256(path),
            }
        )
    lock = {
        "schema_version": "iraf.piper-model-source-lock/v1",
        "name": "agilex_piper",
        "source": str(source),
        "file_count": len(entries),
        "files": entries,
    }

    mismatches = []
    if lock_path.is_file():
        try:
            previous = json.loads(lock_path.read_text(encoding="utf-8"))
        except (ValueError, OSError):
            previous = {}
        known = {
            item.get("relative_path"): item.get("sha256")
            for item in (previous.get("files") or [])
        }
        for entry in entries:
            old = known.get(entry["relative_path"])
            if old is not None and old != entry["sha256"]:
                mismatches.append(entry["relative_path"])
    lock_path.parent.mkdir(parents=True, exist_ok=True)
    lock_path.write_text(json.dumps(lock, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return {
        "source": str(source),
        "lock": str(lock_path),
        "file_count": len(entries),
        "mismatches": mismatches,
    }


def _joint_ids(model, names):
    ids = []
    for name in names:
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if joint_id < 0:
            raise ValueError("MJCF 缺少关节: " + name)
        ids.append(int(joint_id))
    return ids


def _geom_id(model, name):
    geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)
    if geom_id < 0:
        raise ValueError("MJCF 缺少 geom: " + name)
    return int(geom_id)


def _body_id(model, name):
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
    if body_id < 0:
        raise ValueError("MJCF 缺少 body: " + name)
    return int(body_id)


def _finger_center(model, data, left_geom, right_geom):
    return (data.geom_xpos[left_geom] + data.geom_xpos[right_geom]) / 2.0


def _contact_pairs(model, data):
    pairs = []
    for index in range(int(data.ncon)):
        contact = data.contact[index]
        pairs.append(
            (
                mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom1) or "?",
                mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom2) or "?",
                round(float(contact.dist), 6),
            )
        )
    return pairs


def solve_finger_center_ik(
    model, data, target, arm_joints, left_geom, right_geom, solver_cfg
):
    """以双指指尖 geom 中点为目标求解臂关节姿态，返回契约层 `IkResult`。

    IK 算法在机器人无关的 `iraf_core.kinematics.solve_position_ik`；
    本函数只把 Piper 侧的关节 id / geom id 与 solver 配置翻译成契约参数。
    等价性由 `scripts/verify_ik_equivalence.py` 逐位验证
    （同初值同目标同参数下 max_joint_abs_diff_rad = 0.0）。

    **内部一律用本函数**：`balance_tip_clearance` 等 core 能力消费的是
    IkResult（属性访问），dict 形状只在写参考姿态 JSON 时由
    `_pack_pose()` 一次性构造。
    """
    solver = {
        "iterations": int(solver_cfg.get("iterations", 800)),
        "step": float(solver_cfg.get("step", 0.5)),
        "tolerance_m": float(solver_cfg.get("tolerance_m", 1e-5)),
    }
    return solve_position_ik(
        model,
        data,
        target,
        arm_joints,
        [
            {"kind": "geom", "id": int(left_geom)},
            {"kind": "geom", "id": int(right_geom)},
        ],
        **solver
    )


def _pack_pose(result, joint_names):
    """把 IkResult 装配成参考姿态 JSON 的字段形状（对外契约，保持不变）。"""
    return {
        "joint_positions": {
            name: float(result.joint_positions[name]) for name in joint_names
        },
        "finger_center_m": [float(value) for value in result.solved_position_m],
        "target_m": [float(value) for value in result.target_position_m],
        "position_error_m": float(result.position_error_m),
        "iterations": int(result.iterations),
    }


def solve_finger_center_pose(
    model, data, target, arm_joints, joint_names, left_geom, right_geom, solver_cfg
):
    """兼容入口：返回参考姿态 JSON 的 dict 形状。

    保留给仓库外/历史脚本调用（如 `scripts/verify_ik_equivalence.py`）；
    仓库内新代码请直接用 `solve_finger_center_ik` + `_pack_pose`，
    避免在算法层与 JSON 契约层之间来回转换。
    """
    return _pack_pose(
        solve_finger_center_ik(
            model, data, target, arm_joints, left_geom, right_geom, solver_cfg
        ),
        joint_names,
    )

def build_reference_poses(root, baseline, target_id=None, target_xy_override_m=None,
                          target_z_override_m=None):
    """求解 home/approach/grasp 三个参考关节姿态。

    target_id 用于多目标场景：显式指定本次求解针对哪个目标，
    缺省沿用配置 target.id，保持单目标场景行为不变。

    `target_z_override_m`（可选，2026-09-24）＝抓取点 z（**世界系**，即目标中心高度）。
    为什么这两个覆盖都要：本函数按"臂自己场景"的目标几何（`top_z + half_size`）求解；联合场景若
    方块尺寸/台面声明不同（本场景半边长 0.025 ≠ 基线 0.03），只覆盖 xy 会留下一个纯 z 的常数偏差
    （实测 0.018389447 m）。缺省 None ⇒ 行为与改动前逐位一致。

    `target_xy_override_m`（可选，2026-09-24）＝**抓取点相对臂基座的 xy**（臂基座系）。
    为什么需要：本函数在臂自己场景里求解，臂基座在原点 ⇒ 解出来的是"目标相对基座"的关节解；
    联合场景里臂被 `placement` 挪走并转了 90°（`scenes/handoff_lab/scene.yaml`）⇒ 直接照搬会让
    指腹落到别处（实测残差 0.367696068 m）。把目标先换算到臂基座系再重解即可复用同一套配方与模型。
    缺省 None ⇒ 行为与改动前逐位一致。
    """
    model_cfg = baseline["model"]
    source = _resolve(root, model_cfg["source"])
    arm_names = list(model_cfg.get("arm_joints") or [f"joint{i}" for i in range(1, 7)])
    target_cfg = baseline.get("target") or {}
    # 指尖 geom 名称、摩擦、kp 与重力由场景生成器统一写入，
    # 因此参考姿态必须在生成后的探测场景上求解，保证与验收模型一致。
    workdir = Path(tempfile.mkdtemp(prefix="piper-baseline-"))
    probe_scene = workdir / "probe-scene.xml"
    build_scene(
        source,
        probe_scene,
        target_id=target_id or target_cfg.get("id", "box_01"),
        half_size=float(target_cfg.get("half_size_m", 0.03)),
        config=baseline,
    )

    model = mujoco.MjModel.from_xml_path(str(probe_scene))
    data = mujoco.MjData(model)

    arm_joints = _joint_ids(model, arm_names)
    left_geom = _geom_id(model, model_cfg["finger_geoms"]["left"])
    right_geom = _geom_id(model, model_cfg["finger_geoms"]["right"])
    wrist_body = _body_id(model, model_cfg["bodies"]["wrist"])

    workbench = baseline.get("workbench") or {}
    target_cfg = baseline.get("target") or {}
    grasp_cfg = baseline.get("grasp") or {}
    half_size = float(target_cfg.get("half_size_m", 0.03))
    top_z = float(workbench.get("top_z_m", 0.0))
    finger_xy = (list(target_xy_override_m) if target_xy_override_m is not None
                 else grasp_cfg.get("finger_center_xy_m"))
    if not finger_xy or len(finger_xy) != 2:
        raise ValueError("基线配置缺少 grasp.finger_center_xy_m")
    grasp_target = [
        float(finger_xy[0]), float(finger_xy[1]),
        (float(target_z_override_m) if target_z_override_m is not None else top_z + half_size),
    ]

    solver_cfg = grasp_cfg.get("solver") or {}
    offset = float(grasp_cfg.get("pregrasp_offset_m", 0.04))
    direction = np.asarray(grasp_cfg.get("approach_direction") or [0.0, 0.0, 1.0], dtype=float)
    direction_norm = float(np.linalg.norm(direction))
    if direction_norm < 1e-9:
        raise ValueError("grasp.approach_direction 不能为零向量")
    direction = direction / direction_norm
    tip_clearance = float(grasp_cfg.get("tip_clearance_m", 0.005))
    base_target = np.asarray(grasp_target, dtype=float)

    mujoco.mj_forward(model, data)
    # 指腹 geom 中心并不是抓取点：指尖比它再低约 50mm。
    # 直接用方块中心作为指尖中心会让指尖扎进工作台，因此按指尖离台间隙自动配平抓取高度。
    # 配平算法本体在 core（与机型无关，UR5e 侧同一份实现）。
    def solve_for_clearance(target):
        return solve_finger_center_ik(
            model, data, target, arm_joints, left_geom, right_geom, solver_cfg
        )

    grasp_ik, height_correction, clearance_trace, cleared = balance_tip_clearance(
        model,
        data,
        solve_for_clearance,
        base_target,
        direction,
        (left_geom, right_geom),
        top_z,
        tip_clearance,
        iterations=int(grasp_cfg.get("clearance_iterations", 8)),
    )
    if not cleared:
        last = clearance_trace[-1] if clearance_trace else {}
        raise ValueError(
            "指尖离台间隙配平未收敛: tip_z=%.9f required=%.9f（轨迹 %d 步）"
            % (
                float(last.get("tip_z_m", float("nan"))),
                top_z + tip_clearance,
                len(clearance_trace),
            )
        )
    grasp_target_corrected = base_target + direction * height_correction
    # 边界处一次性装配成参考姿态 JSON 的字段形状（对外契约）。
    grasp = _pack_pose(grasp_ik, arm_names)

    wrist = data.xpos[wrist_body].copy()
    axis = np.asarray(grasp["finger_center_m"], dtype=float) - np.asarray(wrist, dtype=float)
    axis_norm = float(np.linalg.norm(axis))
    if axis_norm < 1e-9:
        raise ValueError("腕部到指尖中点距离过小，无法计算夹爪朝向")
    axis = axis / axis_norm

    # 预抓取方向可与接近方向不同：
    # 多目标场景中让 APPROACH 沿纯竖直抬高，保证 APPROACH -> DESCEND 是竖直下降，
    # 避免下降时还需要 joint1 微调（夹爪已环抱目标，微调会被摩擦锁死）。
    pregrasp_direction = np.asarray(
        grasp_cfg.get("pregrasp_direction") or direction, dtype=float
    )
    pregrasp_norm = float(np.linalg.norm(pregrasp_direction))
    if pregrasp_norm < 1e-9:
        raise ValueError("grasp.pregrasp_direction 不能为零向量")
    pregrasp_direction = pregrasp_direction / pregrasp_norm

    approach_target = grasp_target_corrected + pregrasp_direction * offset
    if float(approach_target[2]) <= top_z:
        raise ValueError(
            "预抓取位置未离开工作台: "
            f"approach_z={float(approach_target[2]):.9f} workbench_top_z={top_z:.9f}"
        )
    approach = solve_finger_center_pose(
        model, data, approach_target, arm_joints, arm_names, left_geom, right_geom, solver_cfg
    )


    lift_offset = float(grasp_cfg.get("lift_offset_m", 0.08))
    # 抬升同样沿预抓取方向（多目标场景下即竖直）：夹爪已夹住目标，
    # 沿倾斜方向抬升会让目标产生水平拖拽，而竖直抬升只考验摩擦力。
    lift_target = grasp_target_corrected + pregrasp_direction * lift_offset
    lift = solve_finger_center_pose(
        model, data, lift_target, arm_joints, arm_names, left_geom, right_geom, solver_cfg
    )


    home_qpos = {name: 0.0 for name in grasp["joint_positions"]}
    reference = {
        "schema_version": "iraf.piper-reference-pose/v1",
        "source": str(source),
        "gripper_axis_world": [round(float(value), 9) for value in axis],
        # 抓取点沿该方向从指尖中点回退，场景生成器据此写入 pad_offset_axis。
        "approach_direction_world": [round(float(value), 9) for value in direction],
        # 预抓取/抬升的偏移方向（多目标场景为纯竖直，保证竖直进近与抬升）。
        "pregrasp_direction_world": [
            round(float(value), 9) for value in pregrasp_direction
        ],
        "pregrasp_offset_m": offset,

        "tip_clearance_m": tip_clearance,
        # 指尖最低点取自 core 配平轨迹的最后一步（同一口径、可追溯）。
        "finger_tip_z_m": round(float(clearance_trace[-1]["tip_z_m"]), 9),
        "finger_height_correction_m": round(float(height_correction), 9),
        "clearance_trace": clearance_trace,
        "grasp_point_m": [round(float(value), 9) for value in base_target],
        "finger_center_m": grasp["finger_center_m"],
        "target_z_m": top_z + half_size,
        "lift_offset_m": lift_offset,
        "home": home_qpos,
        "approach": approach,
        "grasp": grasp,
        "lift": lift,
    }
    return reference


def validate_grasp_pose(scene_path, baseline, reference):
    """在最终场景上校验参考抓取姿态：指尖不碰台、张开时手指不与任何物体接触。"""
    model_cfg = baseline["model"]
    arm_names = list(model_cfg.get("arm_joints") or [f"joint{i}" for i in range(1, 7)])
    finger_names = (
        model_cfg["finger_geoms"]["left"],
        model_cfg["finger_geoms"]["right"],
    )
    gripper_cfg = baseline.get("gripper") or {}
    open_positions = dict(gripper_cfg.get("open") or {"joint7": 0.035, "joint8": -0.035})
    workbench = baseline.get("workbench") or {}
    grasp_cfg = baseline.get("grasp") or {}
    top_z = float(workbench.get("top_z_m", 0.0))
    tip_clearance = float(grasp_cfg.get("tip_clearance_m", 0.005))

    model = mujoco.MjModel.from_xml_path(str(scene_path))
    data = mujoco.MjData(model)
    for name in arm_names:
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        data.qpos[int(model.jnt_qposadr[joint_id])] = float(
            reference["grasp"]["joint_positions"][name]
        )
    for name, value in open_positions.items():
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        data.qpos[int(model.jnt_qposadr[joint_id])] = float(value)
    mujoco.mj_forward(model, data)

    geom_ids = tuple(_geom_id(model, name) for name in finger_names)
    tip_z = lowest_mesh_point_z(model, data, geom_ids)
    pairs = _contact_pairs(model, data)
    touching = [pair for pair in pairs if pair[0] in finger_names or pair[1] in finger_names]
    if tip_z < top_z + tip_clearance - 1e-4:
        raise ValueError(
            "参考抓取姿态指尖扎入工作台: "
            f"tip_z={tip_z:.9f} required={top_z + tip_clearance:.9f}"
        )
    if touching:
        raise ValueError(
            "参考抓取姿态下手指与场景发生接触: "
            + ", ".join(f"{pair[0]}|{pair[1]}({pair[2]})" for pair in touching)
        )
    return {
        "finger_tip_z_m": round(float(tip_z), 9),
        "tip_clearance_m": tip_clearance,
        "contact_count": len(pairs),
        "finger_contacts": [],
    }


def build(root, baseline_path, scene_path, calibration_path=None, target_id=None):
    """校验模型来源、求解参考姿态并生成受控场景。

    `target_id` 用于多目标基线：只影响"哪个目标承载搬运约束/参考姿态"，
    缺省取基线声明的 target.id（单目标场景行为不变）。
    """
    root = Path(root).resolve()
    baseline = load_baseline(_resolve(root, baseline_path))
    lock_report = verify_source_lock(root, baseline)
    if lock_report["mismatches"]:
        raise ValueError(
            "Piper 模型与来源清单不一致: " + ", ".join(lock_report["mismatches"])
        )

    # 多目标编排（按目标推导抓取参数 + 抬高 HOME）实现在通用构建器里，
    # 这里只按声明调用，避免同一套逻辑在两处各写一遍。
    grasp_cfg = baseline.get("grasp") or {}
    target_mode = None
    if grasp_cfg.get("derive_from_target"):
        if not target_id:
            raise ValueError(
                "grasp.derive_from_target=true 时必须指定 target_id（多目标基线）"
            )
        baseline, target_mode = derive_grasp_for_target(baseline, target_id)

    reference = build_reference_poses(root, baseline, target_id=target_id)
    if (baseline.get("grasp") or {}).get("raised_home"):
        reference["home"] = raised_home_pose(
            root, baseline, target_id, reference, scene_builder=build_scene
        )
        reference["home_hold_mode"] = "raised_above_approach"
    if target_mode:
        reference["target_mode"] = target_mode
    acceptance = baseline.get("acceptance") or {}
    tolerance = float(acceptance.get("pose_tolerance_m", 0.005))
    factor = float(acceptance.get("solver_error_factor", 0.1))
    error = float(reference["grasp"]["position_error_m"])
    if error > tolerance * factor:
        raise ValueError(
            "参考抓取姿态残差超出门禁容差: "
            f"error={error:.9f}m limit={tolerance * factor:.9f}m"
        )

    target_cfg = baseline.get("target") or {}
    scene = build_scene(
        _resolve(root, baseline["model"]["source"]),
        _resolve(root, scene_path),
        # 多目标基线：调用方指定的 target_id 优先（顶层 target.id 只是历史缺省）
        target_id=target_id or target_cfg.get("id", "box_01"),
        half_size=float(target_cfg.get("half_size_m", 0.03)),
        config=baseline,
        reference=reference,
    )

    scene["grasp_pose_validation"] = validate_grasp_pose(
        _resolve(root, scene_path), baseline, reference
    )

    if calibration_path is not None:
        pose_path = _resolve(root, calibration_path)
        pose_path.parent.mkdir(parents=True, exist_ok=True)
        pose_path.write_text(
            json.dumps(reference, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
        )
        reference["pose_evidence"] = str(pose_path)
    # ---- 重力前馈（逐相位 ctrl 增量）：见 config 里的 gravity_feedforward 段说明 ----
    # ⚠ 必须用**注入增益后**的场景模型（`scene.arm_position_kp`）算增量：增益错，增量就错。
    # 实现在公开入口 `build_reference_feedforward`（联合构建器按声明复用同一个入口，
    # 避免"同一套前馈算法两处各写一遍"）；本处传 prefix="" ⇒ 键就是模型内关节名，行为逐位不变。
    ff_model_path = _resolve(root, scene_path)
    ff_model = mujoco.MjModel.from_xml_path(str(ff_model_path))
    feedforward, feedforward_evidence = build_reference_feedforward(ff_model, reference, baseline)
    for phase, key in REFERENCE_PHASE_KEYS:
        compensation = feedforward.get(phase)
        if not compensation:
            continue
        # 键必须与该段位置指令一一对应：错位会让前馈静默失效（表现为"精度莫名不达标"）
        declared_keys = set((scene.get("gripper") or {}).get(key) or {})
        unknown = sorted(set(compensation) - declared_keys)
        if unknown:
            raise ValueError(
                "重力前馈 %s 含该段位置指令里不存在的通道: %s（前馈键与位置指令必须一一对应）"
                % (phase, unknown))
    reference["gravity_feedforward"] = feedforward
    reference["gravity_feedforward_evidence"] = feedforward_evidence
    gripper_block = scene.get("gripper")
    if not isinstance(gripper_block, dict):
        raise ValueError("场景缺少 gripper 段，无法写入重力前馈（后端从 gripper.gravity_feedforward 取）")
    gripper_block["gravity_feedforward"] = {
        phase: {name: float(value) for name, value in offsets.items()}
        for phase, offsets in feedforward.items()
    }
    scene["model_source_lock"] = lock_report
    scene["reference_poses"] = reference
    return scene


def build_baseline_scene(root, baseline_path, source, scene_path):
    """供 verify_piper_pick.py 调用：忽略 --source，统一使用基线声明的模型来源。"""
    del source
    return build(root, baseline_path, scene_path, calibration_path="build/calibration/piper-baseline-pose.json")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--baseline", type=Path, default=Path(DEFAULT_BASELINE))
    parser.add_argument("--scene", type=Path, default=Path("build/models/piper-pick-scene.xml"))
    parser.add_argument(
        "--pose-evidence",
        type=Path,
        default=Path("build/calibration/piper-baseline-pose.json"),
    )
    parser.add_argument("--root", type=Path, default=Path(__file__).resolve().parents[1])
    args = parser.parse_args(argv)
    scene = build(args.root, args.baseline, args.scene, calibration_path=args.pose_evidence)
    print(json.dumps(scene, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
