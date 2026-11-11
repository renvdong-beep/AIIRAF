"""端侧运动学**计划**计算（kinematic planning）：只算数与几何，不跑物理、不渲染。

为什么不是 pick_object 的后端（2026-11-11 设计修正）：
现有 `PickObjectProvider` 要求 `grasped=true` 且 evidence `bilateral_contact=true`（物理接触证据）。
运动学没有接触，**不能**也不该产出这些字段；否则就是伪造成功（IRAF 铁律 5）。
因此本模块输出的是 **plan**（可达性/关节解/限位余量/时序/夹爪开口），并强制标注：
    planned=true, physics=false, backend="kinematic"

依赖：mujoco（只需模型加载与 mj_forward）+ numpy；`iraf_core.kinematics` 的 IK 契约。
模型来源：优先仓库 `build/`（开发机），其次 bundle 自带 `deploy/sdk/prebuilt/scenes/`（端侧）。
"""
from __future__ import annotations

import hashlib
import json
import os
import time
from pathlib import Path
from typing import Optional

import numpy as np

from iraf_core.kinematics import KinematicsError, body_id, joint_ids, solve_position_ik

SCENE_ALIASES = {
    "ur5-pick-planned": "ur5-pick-scene",
    "piper-pick-planned": "piper-pick-scene",
    "ur5-pick": "ur5-pick-scene",
    "piper-pick": "piper-pick-scene",
}
CANDIDATE_DIRS = ("build/models", "build/scenes/handoff_lab",
                  "deploy/sdk/prebuilt/scenes")
IK_TOLERANCE_M = 1e-3


class PlanError(RuntimeError):
    def __init__(self, code: str, detail: str):
        super().__init__("%s: %s" % (code, detail))
        self.code = code
        self.detail = detail


def resolve_scene(scene_id: str, repo_root: Optional[Path] = None) -> tuple[Path, Path]:
    """返回 (model.xml, report.json)。找不到就显式失败（不猜路径）。"""
    stem = SCENE_ALIASES.get((scene_id or "").strip(), (scene_id or "").strip())
    root = Path(repo_root or os.environ.get("IRAF_REPO_ROOT") or ".")
    tried = []
    for rel in CANDIDATE_DIRS:
        base = root / rel
        for xml in (base / (stem + ".xml"), base / stem / (stem + ".xml")):
            tried.append(str(xml))
            if xml.is_file() and xml.with_suffix(".json").is_file():
                return xml, xml.with_suffix(".json")
    raise PlanError("SCENE-NOT-FOUND",
                    "找不到场景模型 %r（尝试：%s）" % (stem, ", ".join(tried[:4])))


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _seed_qpos(model, data, report: dict, arm_names: list) -> None:
    """用报告里的 lift_positions 作为 IK 初值（与 MuJoCo 抓取前的姿态一致）。"""
    lift = ((report.get("gripper") or {}).get("lift_positions") or {})
    for name in arm_names:
        if name not in lift:
            continue
        jid = joint_ids(model, [name])[0]
        adr = int(model.jnt_qposadr[jid])
        value = float(lift[name])
        if model.jnt_limited[jid]:
            lo, hi = model.jnt_range[jid]
            value = min(max(value, float(lo)), float(hi))
        data.qpos[adr] = value


def plan_pick(scene_id: str = "ur5-pick-planned", target_id: str = "box_01",
              approach_m: float = 0.10, lift_m: float = 0.05,
              duration_ms: int = 3000, iterations: int = 800,
              repo_root: Optional[Path] = None) -> dict:
    if not 0.0 <= float(approach_m) <= 0.5:
        raise PlanError("INPUT-INVALID", "approach_m 必须在 0..0.5 m")
    if not 0.0 <= float(lift_m) <= 0.5:
        raise PlanError("INPUT-INVALID", "lift_m 必须在 0..0.5 m")
    if not 100 <= int(duration_ms) <= 60000:
        raise PlanError("INPUT-INVALID", "duration_ms 必须在 100..60000")

    xml_path, report_path = resolve_scene(scene_id, repo_root)
    report = json.loads(report_path.read_text(encoding="utf-8"))
    targets = {t["id"]: t for t in (report.get("targets") or []) if isinstance(t, dict)}
    target = targets.get(target_id)
    if target is None:
        raise PlanError("TARGET-NOT-FOUND",
                        "场景 %s 没有目标 %r（可用：%s）" % (xml_path.name, target_id, sorted(targets)))
    gripper = report.get("gripper") or {}
    arm_names = list(report.get("arm_joints") or [])
    if not arm_names:
        raise PlanError("SCENE-INVALID", "场景报告缺少 arm_joints")

    import mujoco

    t0 = time.time()
    model = mujoco.MjModel.from_xml_path(str(xml_path))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    load_ms = (time.time() - t0) * 1000.0
    _seed_qpos(model, data, report, arm_names)
    mujoco.mj_forward(model, data)

    target_body = body_id(model, target["body"])
    left = body_id(model, gripper["left_finger_body"])
    right = body_id(model, gripper["right_finger_body"])
    center = np.array(data.xpos[target_body], dtype=float)
    quat = [float(v) for v in data.xquat[target_body]]
    approach_point = center + np.array([0.0, 0.0, float(approach_m)])

    t1 = time.time()
    try:
        ik = solve_position_ik(
            model, data, approach_point, joint_ids(model, arm_names),
            [{"kind": "body", "id": left}, {"kind": "body", "id": right}],
            iterations=int(iterations))
    except KinematicsError as exc:
        raise PlanError("IK-FAILED", str(exc))
    ik_ms = (time.time() - t1) * 1000.0

    limits = {}
    margins = []
    for name in arm_names:
        jid = joint_ids(model, [name])[0]
        adr = int(model.jnt_qposadr[jid])
        value = float(data.qpos[adr])
        if model.jnt_limited[jid]:
            lo, hi = (float(model.jnt_range[jid][0]), float(model.jnt_range[jid][1]))
            margin = min(value - lo, hi - value)
            limits[name] = {"q": round(value, 6), "lo": round(lo, 6), "hi": round(hi, 6),
                            "margin_deg": round(float(np.degrees(margin)), 3)}
            margins.append(float(np.degrees(margin)))
        else:
            limits[name] = {"q": round(value, 6), "limited": False}
    if not margins:
        raise PlanError("SCENE-INVALID", "臂关节没有任何限位声明，无法给出限位余量")

    converged = ik.position_error_m <= IK_TOLERANCE_M
    result = {
        "schema": "iraf.kinematic_plan/v1",
        "status": "SUCCEEDED" if converged else "FAILED",
        "planned": True,
        "physics": False,
        "backend": "kinematic",
        "scene": str(xml_path),
        "target": {"id": target_id, "position_m": [round(float(v), 6) for v in center],
                   "quat_wxyz": [round(v, 6) for v in quat]},
        "approach": {"point_m": [round(float(v), 6) for v in approach_point],
                     "offset_m": float(approach_m), "lift_m": float(lift_m)},
        "ik": {"iterations": int(ik.iterations), "error_m": round(float(ik.position_error_m), 9),
               "tolerance_m": IK_TOLERANCE_M, "converged": converged,
               "joints_rad": {k: round(float(v), 6) for k, v in ik.joint_positions.items()}},
        "limits": {"min_margin_deg": round(min(margins), 3), "per_joint": limits},
        "gripper": {"left_finger_body": gripper.get("left_finger_body"),
                    "right_finger_body": gripper.get("right_finger_body"),
                    "open_positions": gripper.get("open_positions"),
                    "closed_positions": gripper.get("closed_positions"),
                    "target_half_size_m": target.get("target_half_size_m")},
        "timing": {"duration_ms": int(duration_ms), "load_ms": round(load_ms, 1),
                   "ik_ms": round(ik_ms, 1)},
        "model": {"path": str(xml_path), "sha256": _sha256(xml_path),
                  "report": str(report_path)},
        "disclaimer": "计划计算（运动学），无物理/接触；不得表述为抓取成功或物理验收",
    }
    if not converged:
        result["error_code"] = "IK-NOT-CONVERGED"
        result["reason"] = "位置残差 %.6f m 超过容差 %.6f m" % (ik.position_error_m, IK_TOLERANCE_M)
    result["report"] = _write_report(result)
    return result


def _write_report(result: dict) -> dict:
    base = Path(os.environ.get("IRAF_PLAN_REPORT_DIR") or "build/acceptance-planned")
    base.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d-%H%M%S")
    target = (result.get("target") or {}).get("id") or "plan"
    json_path = base / ("plan_%s_%s.json" % (target, stamp))
    json_path.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
    lines = ["# 运动学计划报告（无物理仿真）", "",
             "- 场景：`%s`" % result["scene"],
             "- 目标：`%s` @ %s" % (target, result["target"]["position_m"]),
             "- IK：收敛=%s，误差=%.6f m，迭代=%d" % (result["ik"]["converged"],
                                                  result["ik"]["error_m"], result["ik"]["iterations"]),
             "- 限位最小余量：%.1f°" % result["limits"]["min_margin_deg"],
             "- 耗时：模型加载 %.0f ms，IK %.0f ms" % (result["timing"]["load_ms"], result["timing"]["ik_ms"]),
             "", "> 计划计算（运动学），**无物理/接触**；不构成抓取成功或物理验收。"]
    md_path = base / ("plan_%s_%s.md" % (target, stamp))
    md_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return {"json": str(json_path), "markdown": str(md_path)}