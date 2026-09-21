"""支撑集可行性探针（只读）：把「换一种支撑集判定语义后还站不站得住」从**推论**变成**数字**。

用途（战役 iraf-24h-2 步骤 02b 的决策包）
------------------------------------------
B1（ADR-0008 决策 1d，支撑腿走力矩级力控）落地后，wave/trot 双复评在同一批 9 项判据上失败，
实测根因是**支撑集按实测接触力判定 ⇒ 自锁**（声明摆动窗口内 900/900 采样四腿全接触，探针
`scripts/probe_go2_support_classification.py`）。人类要做的决策是**支撑集判定语义**：
是否引入「声明相位 ∧ 实测接触」（摆动窗口内的腿不参与 mg 分摊、不承受力控）。

本探针回答该决策的**静力学前提**：把声明摆动腿从支撑集里去掉（三腿支撑）后，
**生产环境同一个分配器** `iraf_adapters.unitree.balance.allocate_foot_forces`
还能否兑现期望力旋量（静态 = `(0, 0, mg)`、力矩 0）。

- 四腿支撑集（现状 B1 的实测接触判定）是**正例对照**：期望「无截断、残差 ≈ 0」。
  若对照组都不成立，说明本探针自己坏了，**不得据此判读实验组**。
- 四个三腿支撑集是待判假设（对应 wave 的四个单腿摆动相位）。

判读口径（**不引入任何新阈值**）
--------------------------------
分配器对每腿法向力取 `[normal_force_floor_n, max_normal_force_n]`、水平向取 ±`max_horizontal_force_n`
截断（全部来自声明）。因此判据是两条**事实**而非调出来的阈值：

1. `clamped_legs` 是否非空 —— 非空即表示「该腿需要被向下拉住/超出摩擦预算才能兑现期望力旋量」，
   物理上不可能 ⇒ 该支撑集**不可行**（最小二乘的精确解落在物理约束之外）。
2. 截断**之后**的力旋量残差 `residual` 相对 `mg` 的比值 —— 只登记比值，不设通过线。

边界（诚实声明）
----------------
- 只读：不改声明、不跑控制回路、不写仓库（`--json` 只在证据区）。
- 位形取**关键帧中立位形**（`mj_resetDataKeyframe(0)` ⇒ 「命令抬腿的瞬间」），因此是**静力学**结论，
  不含冲击、摩擦滑移与阻尼的动力学效果；三腿支撑下其余三足的足端位置仍取该中立位形。
- 结论属于仿真（`simulation=true`）；真机/目标端不适用。
- 本探针**不选路**：它只报告「某假设在静力学上是否可行」，不实现任何候选机制、不改配置。

用法
----
    /usr/bin/python3 scripts/probe_go2_support_set_feasibility.py \
        --baseline config/go2_loopback.yaml \
        [--model build/scenes/handoff_lab/handoff_lab.xml] \
        [--json build/iraf-24h-2/02b/support-set-feasibility.json]

退出码
------
    0 成功 / 1 用法错误 / 2 声明非法（缺 gait.legs / balance / profile.spec.model.trunk_body）
    3 引用完整性失败（模型编译、关键帧、足端几何、躯干 body 缺失）/ 4 力分配不可解（显式抛出）
"""

import argparse
import json
import sys
from pathlib import Path

import numpy as np
import yaml

_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_ROOT / "src"))
sys.path.insert(0, str(_ROOT / "scripts"))

import mujoco  # noqa: E402

from iraf_adapters.unitree import balance as balance_module  # noqa: E402
from probe_quadruped_support_margin import margins_and_normals  # noqa: E402

DEFAULT_MODEL = "build/scenes/handoff_lab/handoff_lab.xml"


class ProbeError(Exception):
    """带退出码的显式失败（不吞错、不给默认值）。"""

    def __init__(self, message, code=2):
        super().__init__(message)
        self.code = int(code)


def load_declarations(baseline_path):
    """读基线：`gait.legs` 提供足端几何与腿代号，`balance` 段交给生产解析器。"""
    path = Path(baseline_path)
    if not path.is_file():
        raise ProbeError("基线不存在: %s" % baseline_path, code=3)
    baseline = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    legs_section = (baseline.get("gait") or {}).get("legs") or {}
    if not legs_section:
        raise ProbeError("基线 %s 未声明 gait.legs：支撑集无依据" % baseline_path, code=2)
    legs = sorted(legs_section)
    contact_geoms = {code: str(legs_section[code]["contact_geom"]) for code in legs}
    if "balance" not in baseline:
        raise ProbeError("基线 %s 未声明 balance 段：力分配无依据" % baseline_path, code=2)
    params = balance_module.load_balance_declaration(baseline)
    return baseline, legs, contact_geoms, params


def resolve_trunk_body(baseline, profile_path=None):
    """躯干 body 名来自 Profile 的构建期声明 `spec.model.trunk_body`（缺即显式失败）。"""
    if profile_path is None:
        robot_section = baseline.get("robot")
        if not isinstance(robot_section, dict):
            raise ProbeError("基线未声明 robot.profile：无法解析躯干 body", code=2)
        profile_path = str(robot_section.get("profile") or "")
    if not profile_path:
        raise ProbeError("基线未声明 robot.profile（不得猜测 Profile 路径）", code=2)
    profile_file = Path(profile_path)
    if not profile_file.is_file():
        raise ProbeError("Profile 不存在: %s" % profile_path, code=3)
    profile = yaml.safe_load(profile_file.read_text(encoding="utf-8")) or {}
    model_section = (profile.get("spec") or {}).get("model") or {}
    trunk = model_section.get("trunk_body")
    if not trunk:
        raise ProbeError("Profile %s 未声明 spec.model.trunk_body" % profile_path, code=2)
    return str(trunk), str(profile_path)


def neutral_state(model_path, legs, contact_geoms, trunk_name):
    """中立位形下的足端世界坐标、躯干参考点与整机重心投影（全部实测，不猜）。"""
    if not Path(model_path).is_file():
        raise ProbeError("模型不存在: %s（不得退化成一次空跑）" % model_path, code=3)
    try:
        model = mujoco.MjModel.from_xml_path(model_path)
    except Exception as exc:  # 编译失败也是引用完整性失败，映射成退出码而不是 traceback
        raise ProbeError("模型编译失败: %s" % exc, code=3)
    data = mujoco.MjData(model)
    if model.nkey == 0:
        raise ProbeError("模型 %s 没有关键帧：无法取中立位形" % model_path, code=3)
    trunk_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, trunk_name)
    if trunk_id < 0:
        raise ProbeError("躯干 body %s 不在模型里" % trunk_name, code=3)
    mujoco.mj_resetDataKeyframe(model, data, 0)
    mujoco.mj_forward(model, data)
    feet = {}
    for code in legs:
        gid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, contact_geoms[code])
        if gid < 0:
            raise ProbeError("足端几何 %s 不在模型里" % contact_geoms[code], code=3)
        feet[code] = np.asarray(data.geom_xpos[gid], dtype=float).copy()
    com = np.asarray(data.subtree_com[0], dtype=float).copy()
    center = np.asarray(data.xpos[int(trunk_id)], dtype=float).copy()
    return model, data, feet, com, center


def allocate_for_support(params, wrench, feet, center, support):
    """按给定支撑集调用**生产**分配器；足端点与参考点口径与适配器一致（geom_xpos / xpos[trunk]）。"""
    stance_points = {code: np.asarray(feet[code], dtype=float).copy() for code in support}
    return balance_module.allocate_foot_forces(params, wrench, stance_points, center)


def summarize(allocation, support, mass_kg, gravity_mps2):
    """把分配结果折成可比数字：逐腿法向力、截断腿、残差（力/力矩 + 相对 mg 比值）。"""
    forces = allocation["forces"]
    normals = {code: float(forces[code][2]) for code in support}
    residual = allocation["residual"]
    residual_force = float(np.linalg.norm(np.asarray(residual["force_n"], dtype=float)))
    residual_torque = float(np.linalg.norm(np.asarray(residual["torque_nm"], dtype=float)))
    weight = float(mass_kg) * abs(float(gravity_mps2))
    min_normal = min(normals.values()) if normals else None
    return {
        "support_legs": list(support),
        "support_leg_count": int(len(support)),
        "normal_force_n": normals,
        "normal_sum_n": float(sum(normals.values())),
        "min_normal_force_n": min_normal,
        "min_normal_over_mg": (min_normal / weight) if (min_normal is not None and weight) else None,
        "clamped_legs": list(allocation["clamped_legs"]),
        "residual_force_n": residual_force,
        "residual_torque_nm": residual_torque,
        "residual_force_over_mg": (residual_force / weight) if weight else None,
        "residual_force_vector_n": [float(v) for v in residual["force_n"]],
        "residual_torque_vector_nm": [float(v) for v in residual["torque_nm"]],
    }


def main(argv=None):
    parser = argparse.ArgumentParser(description="四足支撑集可行性探针（只读）")
    parser.add_argument("--baseline", default="config/go2_loopback.yaml",
                        help="基线声明（只读 gait.legs / balance / robots.<id>.profile）")
    parser.add_argument("--model", default=DEFAULT_MODEL, help="编译后的 MJCF/XML")
    parser.add_argument("--profile", default=None, help="可选：覆盖 Profile 路径")
    parser.add_argument("--json", default=None, help="可选的 JSON 证据输出路径")
    args = parser.parse_args(argv)

    try:
        baseline, legs, contact_geoms, params = load_declarations(args.baseline)
        trunk_name, profile_path = resolve_trunk_body(baseline, args.profile)
        model, data, feet, com, center = neutral_state(
            args.model, legs, contact_geoms, trunk_name
        )
    except ProbeError as exc:
        print("[支撑集可行性探针] %s" % exc, file=sys.stderr)
        return exc.code

    mass = float(model.body_mass.sum())
    # 口径与适配器逐项对齐（`unitree_go2.gravity_mps2` 取绝对值；高度目标取声明的
    # `balance.verification.height_target_m`，不是「把当前高度当目标」）——否则符号/基准错一位，
    # 四腿正例对照会整片被截断（首跑实测：重力取负号 ⇒ 期望力旋量 F_z = −mg、四腿全部被
    # 法向下界 0 截断、残差 = 1.000 × mg，正例对照当场不成立）。
    gravity = abs(float(model.opt.gravity[2]))
    height = float(data.qpos[2])
    gait_verification = (baseline.get("gait") or {}).get("verification") or {}
    if "height_target_m" not in gait_verification:
        print("[支撑集可行性探针] 基线未声明 gait.verification.height_target_m："
              "高度目标无依据（不得用当前高度兜底）", file=sys.stderr)
        return 2
    height_target = float(gait_verification["height_target_m"])
    wrench = balance_module.desired_wrench(
        params,
        mass_kg=mass,
        gravity_mps2=gravity,
        height_m=height,
        height_target_m=height_target,
        vertical_velocity_mps=float(data.qvel[2]),
        roll_rad=0.0,
        pitch_rad=0.0,
        horizontal_velocity_world_mps=np.asarray(data.qvel[0:2], dtype=float),
        angular_velocity_world_rad_s=np.asarray(data.qvel[3:6], dtype=float),
    )
    com_xy = np.array([float(com[0]), float(com[1])], dtype=float)
    feet_xy = {code: np.array([float(feet[code][0]), float(feet[code][1])], dtype=float)
               for code in legs}
    margins, _normals = margins_and_normals(feet_xy, com_xy, legs)

    print("model=%s  keyframe=%s  simulation=true" % (
        args.model, mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_KEY, 0)))
    print("baseline=%s  profile=%s  trunk_body=%s" % (args.baseline, profile_path, trunk_name))
    print("整机质量 = %.6f kg  重力 = %.6f m/s²  mg = %.9f N  声明支撑腿数下限 = %d"
          % (mass, gravity, mass * abs(gravity), int(params["allocation"]["min_stance_legs"])))
    print("期望力旋量（静态）= 力 %s N、力矩 %s N·m（mg 支撑来自声明 include_gravity_support=%s）"
          % ([round(float(v), 9) for v in wrench["force_n"]],
             [round(float(v), 9) for v in wrench["torque_nm"]],
             params["include_gravity_support"]))

    cases = []
    try:
        # 正例对照：四腿支撑集（B1 的实测接触判定：探针实测直方图恒为 4）
        reference = summarize(
            allocate_for_support(params, wrench, feet, center, list(legs)),
            list(legs), mass, gravity,
        )
    except ProbeError as exc:
        print("[支撑集可行性探针] %s" % exc, file=sys.stderr)
        return exc.code
    except Exception as exc:  # 分配器显式失败（秩不足等）——如实报出，不吞
        print("[支撑集可行性探针] 分配器失败: %s" % exc, file=sys.stderr)
        return 4

    print("")
    print("【正例对照】四腿支撑集（现状：实测接触判定 ⇒ 四腿全接触）")
    print("  逐腿法向力 = %s N；合计 = %.9f N（对照 mg = %.9f N）"
          % ({c: round(v, 6) for c, v in reference["normal_force_n"].items()},
             reference["normal_sum_n"], mass * abs(gravity)))
    print("  截断腿 = %s；残差力 = %.3e N（%.3e × mg）、残差力矩 = %.3e N·m"
          % (reference["clamped_legs"] or "无", reference["residual_force_n"],
             reference["residual_force_over_mg"] or 0.0, reference["residual_torque_nm"]))
    ok_reference = (not reference["clamped_legs"]
                    and reference["residual_force_n"] <= 1.0e-9 * mass * abs(gravity)
                    and reference["residual_torque_nm"] <= 1.0e-9 * mass * abs(gravity))
    print("  对照结论：%s" % ("无截断且残差 ≈ 0（分配器可兑现期望力旋量）" if ok_reference
                              else "**对照不成立** ⇒ 本探针或分配器异常，实验组结果不得据此判读"))
    cases.append({"case_id": "reference_four_leg_measured_contact", "role": "positive_control",
                  "realizable": bool(ok_reference), **reference})

    print("")
    print("【实验组】三腿支撑集（对应「声明相位 ∧ 实测接触」在该腿摆动窗口内）")
    for code in legs:
        support = [name for name in legs if name != code]
        try:
            allocation = allocate_for_support(params, wrench, feet, center, support)
        except Exception as exc:  # 分配器显式失败 ⇒ 该假设不可行，如实记录而不是伪造数值
            print("  抬 %-3s（支撑 %s）：分配器显式失败 → %s" % (code, support, exc))
            cases.append({"case_id": "three_leg_excluding_%s" % code, "role": "hypothesis",
                          "excluded_leg": code, "support_legs": support,
                          "realizable": False, "allocator_error": str(exc)})
            continue
        row = summarize(allocation, support, mass, gravity)
        # 判据只取**事实**：`clamped_legs` 非空 = 该腿的物理约束被违反（需要被向下拉住，或超出摩擦预算）。
        # 这里**不引入残差阈值**——本探针首版曾用 `residual_force_n <= 0.0` 判可行，
        # 结果被浮点噪声（1e-15 N）判成"不可兑现"，把两次可兑现相位误报为不可行（自伤型判据）。
        realizable = not row["clamped_legs"]
        cases.append({"case_id": "three_leg_excluding_%s" % code, "role": "hypothesis",
                      "excluded_leg": code, "margin_m": float(margins[code]),
                      "realizable": bool(realizable), **row})
        print("  排除 %-3s：几何余量 = %+.9f m；法向力 = %s N（合计 %.6f N）"
              % (code, margins[code],
                 {c: round(v, 6) for c, v in row["normal_force_n"].items()},
                 row["normal_sum_n"]))
        print("           截断腿 = %s；最小法向力 = %.6f N（%.4f %% mg）；力旋量残差 = 力 %.6f N（%.4f %% mg）"
              " 方向 %s、力矩 %.6f N·m ⇒ %s"
              % (row["clamped_legs"] or "无", row["min_normal_force_n"],
                 100.0 * (row["min_normal_over_mg"] or 0.0), row["residual_force_n"],
                 100.0 * (row["residual_force_over_mg"] or 0.0),
                 [round(v, 4) for v in row["residual_force_vector_n"]],
                 row["residual_torque_nm"],
                 "**不可兑现**（有腿被物理约束截断：非负法向力下无法实现期望力旋量）"
                 if not realizable else
                 "可兑现（无截断）——但余量与最小法向力见本行：零余量特征下不足以抵抗扰动"))

    not_realizable = [item["case_id"] for item in cases
                      if item["role"] == "hypothesis" and not item["realizable"]]
    print("")
    print("汇总：正例对照 %s；三腿假设 %d 组中**不可兑现** %d 组（其余可兑现但余量见上表）"
          % ("成立" if ok_reference else "不成立", len(legs), len(not_realizable)))
    print("说明：本探针**只做静力学可行性判定**，不实现任何候选机制、不改声明/判据；")
    print("      不可行 ≠ 整条路线不可行（重心转移/落点变化会改变位形，需另测）。")

    if args.json:
        out = Path(args.json)
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(json.dumps({
            "simulation": True,
            "probe": "go2_support_set_feasibility",
            "model": args.model,
            "baseline": args.baseline,
            "profile": profile_path,
            "trunk_body": trunk_name,
            "keyframe": mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_KEY, 0),
            "mass_kg": mass,
            "gravity_mps2": gravity,
            "weight_n": mass * abs(gravity),
            "include_gravity_support": bool(params["include_gravity_support"]),
            "min_stance_legs": int(params["allocation"]["min_stance_legs"]),
            "allocation_limits": {
                "normal_force_floor_n": float(params["allocation"]["normal_force_floor_n"]),
                "max_normal_force_n": float(params["allocation"]["max_normal_force_n"]),
                "max_horizontal_force_n": float(params["allocation"]["max_horizontal_force_n"]),
            },
            "desired_wrench": {
                "force_n": [float(v) for v in wrench["force_n"]],
                "torque_nm": [float(v) for v in wrench["torque_nm"]],
            },
            "com_xy_m": [float(com_xy[0]), float(com_xy[1])],
            "foot_xy_m": {code: [float(feet_xy[code][0]), float(feet_xy[code][1])] for code in legs},
            "reference_ok": bool(ok_reference),
            "cases": cases,
            "not_realizable_cases": not_realizable,
            "height_m": height,
            "height_target_m": height_target,
            "probe_scope": "statics_at_neutral_keyframe",
            "not_proved": [
                "三腿支撑在动力学下是否真的站得住（本探针只判静力学可行性）",
                "重心转移/落点变化后的位形是否可行（位形改变则需重跑）",
            ],
        }, ensure_ascii=False, indent=1), encoding="utf-8")
        print("written %s" % out)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
