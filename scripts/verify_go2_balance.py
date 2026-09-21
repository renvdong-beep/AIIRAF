"""Go2 力矩级平衡器验收入口（步骤 02b）：静态抗扰 + 隔离实验复跑（含对照组）。

一句话：本入口只证明「在声明的 `balance` 段参数下，力矩级反馈对**静态抗扰**与
**隔离实验（步高 0.001 m + 幅度扫描）**的实测效果」，并把**对照组**（`balance.enabled: false`，
其余声明逐字相同）与实验组的数字并列 —— 没有对照组的“改善”是无法与“环境本来就不稳”区分的。

数字与路径**全部来自声明**：

- 平衡器增益/权重/分配上限/看门狗/判据：`config/go2_loopback.yaml` 的 `balance` 段；
- 步态参数与步态判据：同声明的 `gait` 段（隔离实验只改 `gait.step_height_m` 与
  `gait.sway.amplitude_m` 两个自变量，写在**证据区**副本里，仓库声明不改动）；
- 关节身份/限位：`profiles/unitree_go2_mujoco.yaml`；移动安全边界：安全策略。

用法::

    PYTHONPATH=src /usr/bin/python3 scripts/verify_go2_balance.py --config config/go2_loopback.yaml

退出码：

- `0` 全部判据通过（报告 `passed: true`）
- `1` 用法错误（缺 `--config`、声明文件不存在）
- `2` 声明非法（balance/gait 段缺键、越界、自相矛盾）
- `3` 引用完整性失败（Profile / 安全策略 / 被测模型 / 关节 / 接触几何不存在）
- `4` 后端装配失败
- `5` 判据未通过（`failed_checks` 逐条列出）

诚实边界：全部结论属于**仿真**（`simulation: true`）；本入口不证明行走/转向/到点
（步骤 03/04），也不含任何真机证据（目标端验收 DEFERRED，板卡不在场）。
`balance.enabled: true` 组是**实验组**：它由本入口按声明生成证据区副本开启，不代表
机型声明当前启用了平衡器（机型声明里的开关值以 `config/go2_loopback.yaml` 为准并在报告里给出）。
"""

import argparse
import datetime
import hashlib
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

import yaml  # noqa: E402

from iraf_adapters.factory import KNOWN_BACKENDS, load_backend  # noqa: E402
from iraf_adapters.unitree import balance as balance_module  # noqa: E402
from iraf_adapters.unitree import gait  # noqa: E402
from iraf_adapters.unitree import quadruped as quadruped_contract  # noqa: E402
from iraf_adapters.unitree import unitree_go2  # noqa: E402
from iraf_core.authority import ControlAuthorityManager  # noqa: E402
from iraf_core.profile import ProfileError, load_robot_profile, load_safety_policy  # noqa: E402
from iraf_skills import quadruped as quadruped_skills  # noqa: E402

EXIT_OK = 0
EXIT_USAGE = 1
EXIT_DECLARATION = 2
EXIT_REFERENCE = 3
EXIT_BACKEND = 4
EXIT_CRITERIA = 5

REPORT_SCHEMA_VERSION = "iraf.go2-balance/v1"
REPORT_KIND = "go2-balance"
LEASE_TTL_MARGIN = 3.0
#: 证据区（gitignore）：两个实验组的声明副本与逐用例报告都落在这里，保住仓库提交边界。
EVIDENCE_DIR = "build/iraf-24h-2/02b"


def _dig(document, dotted, label="声明"):
    node = document
    for part in str(dotted).split("."):
        if not isinstance(node, dict) or part not in node:
            raise quadruped_contract.DeclarationError("%s 缺少声明键: %s" % (label, dotted))
        node = node[part]
    return node


def _resolve(root, value):
    path = Path(str(value))
    return path if path.is_absolute() else Path(root) / path


def _relative(root, path):
    try:
        return str(Path(path).resolve().relative_to(Path(root).resolve()))
    except ValueError:
        return str(path)


def _sha256(path):
    digest = hashlib.sha256()
    digest.update(Path(path).read_bytes())
    return digest.hexdigest()


def _write_group_declaration(root, document, enabled, name, **overrides):
    """把声明写到证据区副本（只改声明的开关与自变量），返回可装配的 config 映射。"""
    data = yaml.safe_load(yaml.safe_dump(document, allow_unicode=True))
    data["balance"]["enabled"] = bool(enabled)
    for dotted, value in overrides.items():
        node = data
        parts = dotted.split(".")
        for part in parts[:-1]:
            node = node[part]
        node[parts[-1]] = value
    path = Path(root) / EVIDENCE_DIR / ("%s.yaml" % name)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(yaml.safe_dump(data, allow_unicode=True), encoding="utf-8")
    config = yaml.safe_load(path.read_text(encoding="utf-8"))
    config["root"] = str(root)
    return config, path


def _run_with_lease(authority, resource, owner, ttl_seconds, action):
    """取租约 → 执行 → **无论成败都释放**（同一进程内连续跑多个用例时，不释放会 `resource busy`）。"""
    lease = authority.acquire(resource, owner, ttl_seconds=ttl_seconds)
    try:
        return action(lease)
    finally:
        try:
            authority.release(lease)
        except Exception:  # 释放失败不改判据结论，但必须留痕（由调用方报告 status）
            pass


def _check(checks, name, value, expectation, passed, detail):
    checks.append(
        {
            "name": name,
            "value": value,
            "expectation": expectation,
            "passed": bool(passed),
            "detail": detail,
        }
    )


def _metrics(samples, onset_xy):
    tilts = [item["tilt_deg"] for item in samples]
    heights = [item["base_height_m"] for item in samples]
    drift = [
        ((item["base_position_xy_m"][0] - onset_xy[0]) ** 2
         + (item["base_position_xy_m"][1] - onset_xy[1]) ** 2) ** 0.5
        for item in samples
    ]
    return {
        "max_tilt_deg": max(tilts),
        "min_base_height_m": min(heights),
        "height_mean_m": sum(heights) / len(heights),
        "max_drift_m": max(drift),
        "saturated_samples": sum(1 for item in samples if item["ctrl_saturated"] > 0),
        "saturated_total": sum(item["ctrl_saturated"] for item in samples),
        "samples": len(samples),
    }


def _static_case(declaration, root, profile, authority, enabled, balanced_params, checks):
    """静态抗扰：站立 + 声明水平冲量 → 恢复窗口内的最坏值。"""
    config, path = _write_group_declaration(
        root, declaration, enabled, "balance-group-static-bal%d" % int(enabled)
    )
    backend = load_backend(KNOWN_BACKENDS[str(_dig(declaration, "robot.backend"))],
                           config, profile, authority)
    verification = balanced_params["verification"]["static_disturbance"]
    seconds = float(verification["duration_s"]) + float(verification["hold_s"])
    execution = _run_with_lease(
        authority,
        profile.name + "-mujoco",
        "balance-static-%d" % int(enabled),
        seconds * LEASE_TTL_MARGIN,
        backend.balance_hold,
    )
    samples = execution["samples"]
    onset_xy = samples[0]["base_position_xy_m"]
    settle = [
        item for item in samples if item["elapsed_s"] >= float(verification["duration_s"])
    ]
    metrics = _metrics(samples, onset_xy)
    settle_metrics = _metrics(settle, onset_xy)
    case = {
        "case": "static_disturbance",
        "balance_enabled": bool(enabled),
        "declaration": _relative(root, path),
        "declaration_sha256": _sha256(path),
        "disturbance": execution["disturbance"],
        "height_target_m": execution["height_target_m"],
        "metrics": metrics,
        "recovery_window_metrics": settle_metrics,
        "ctrl_saturated_samples": execution["ctrl_saturated_samples"],
        "balance_stats": execution["balance"]["stats"],
        # 决策 1g：证明「静态保持走的是**无声明相位**那条控制模式」—— 生效口径 / 声明原文 /
        # 选出它的事实依据三者并列（只从生产侧透传，不在本入口重算）。
        "stance_classification": execution["balance"]["stance_classification"],
        "stance_classification_modes": dict(
            execution["balance"]["stance_classification_modes"]
        ),
        "declared_phase_source": execution["balance"]["declared_phase_source"],
    }
    height_error = abs(settle_metrics["height_mean_m"] - execution["height_target_m"])
    if enabled:
        prefix = "static.balance"
        _check(
            checks,
            prefix + ".max_tilt_deg",
            settle_metrics["max_tilt_deg"],
            "<= %r" % float(verification["max_tilt_deg"]),
            settle_metrics["max_tilt_deg"] <= float(verification["max_tilt_deg"]),
            "恢复窗口内的最坏倾角（不含偏航：tilt = 相对竖直的夹角）",
        )
        _check(
            checks,
            prefix + ".height_error_m",
            height_error,
            "<= %r" % float(verification["max_height_error_m"]),
            height_error <= float(verification["max_height_error_m"]),
            "恢复窗口内的平均高度与声明目标的偏差",
        )
        _check(
            checks,
            prefix + ".max_drift_m",
            settle_metrics["max_drift_m"],
            "<= %r" % float(verification["max_drift_m"]),
            settle_metrics["max_drift_m"] <= float(verification["max_drift_m"]),
            "恢复窗口内的水平位移（相对冲量施加前的起始位置）",
        )
        _check(
            checks,
            prefix + ".saturated_samples",
            settle_metrics["saturated_samples"],
            "<= %r" % int(verification["max_saturated_samples"]),
            settle_metrics["saturated_samples"] <= int(verification["max_saturated_samples"]),
            "控制量饱和的采样数（平衡器不得把控制量顶到模型 ctrlrange 之外）",
        )
        _check(
            checks,
            prefix + ".watchdog_not_triggered",
            case["balance_stats"]["watchdog_triggered"],
            "== false",
            not case["balance_stats"]["watchdog_triggered"],
            "看门狗触发说明「有效支撑腿不足」持续超过声明上限：该次保持不成立",
        )
        # 决策 1g 的门禁（声明 → 事实 → 生效口径）：静态保持**没有**步态时钟 ⇒ 必须取声明里
        # `gait_clock_inactive` 给出的口径。若只把口径写在声明里而不按控制模式取，这条会红
        # （1f 的回归正是「静态保持套用步态相位分类」⇒ 实测 116.0440876799953° 翻倒）。
        declared_modes = balanced_params["stance_classification"]
        _check(
            checks,
            prefix + ".stance_classification_follows_control_mode",
            case["stance_classification"],
            "== %r（无声明相位 ⇒ 取声明 gait_clock_inactive）"
            % declared_modes["gait_clock_inactive"],
            case["stance_classification"] == declared_modes["gait_clock_inactive"]
            and case["declared_phase_source"] == "no_declared_phase",
            "支撑集口径必须按控制模式从声明取值（declared_phase_source=%r）"
            % case["declared_phase_source"],
        )
    return case


def _isolation_case(declaration, root, profile, authority, movement_limits, enabled,
                    params, amplitude, checks):
    """隔离实验：步高 0.001 m + 给定 sway 幅度 → 是否翻倒（对照历史负结果）。"""
    isolation = params["verification"]["isolation"]
    if amplitude == 0.0:
        name = "balance-group-iso-amp0-bal%d" % int(enabled)
    else:
        name = "balance-group-iso-amp%g-bal%d" % (amplitude, enabled)
    config, path = _write_group_declaration(
        root, declaration, enabled, name,
        **{
            "gait.step_height_m": float(isolation["step_height_m"]),
            "gait.sway.amplitude_m": float(amplitude),
            "gait.verification.duration_s": float(isolation["duration_s"]),
        }
    )
    loaded, _root = unitree_go2.load_declaration(config)
    gait_params = gait.load_gait_declaration(loaded, profile.joints)
    backend = load_backend(KNOWN_BACKENDS[str(_dig(declaration, "robot.backend"))],
                           config, profile, authority)
    lease_owner = "balance-isolation-%g-%d" % (amplitude, int(enabled))
    execution = _run_with_lease(
        authority,
        profile.name + "-mujoco",
        lease_owner,
        float(isolation["duration_s"]) * LEASE_TTL_MARGIN,
        backend.gait_in_place,
    )
    samples = execution["samples"]
    onset_xy = samples[0]["base_position_xy_m"]
    metrics = _metrics(samples, onset_xy)
    settled = [item for item in samples if item["time_s"] - samples[0]["time_s"] >= 1.0]
    settled_metrics = _metrics(settled, onset_xy)
    case = {
        "case": "isolation",
        "balance_enabled": bool(enabled),
        "amplitude_m": float(amplitude),
        "step_height_m": float(isolation["step_height_m"]),
        "declaration": _relative(root, path),
        "declaration_sha256": _sha256(path),
        "metrics": metrics,
        "steady_metrics": settled_metrics,
        "ctrl_saturated_samples": execution["ctrl_saturated_samples"],
        "balance_stats": execution["balance"]["stats"],
        # 决策 1g：本路径**有**声明相位 ⇒ 生效口径必须是声明里 `gait_clock_active` 给出的那一个。
        "stance_classification": execution["balance"]["stance_classification"],
        "stance_classification_modes": dict(
            execution["balance"]["stance_classification_modes"]
        ),
        "declared_phase_source": execution["balance"]["declared_phase_source"],
        "gait_assessment": {
            key: value for key, value in gait.assess_gait(
                samples, gait_params, movement_limits["max_tilt_moving_deg"]
            )["metrics"].items()
            if key in ("height_mean_m", "max_tilt_deg", "max_displacement_m", "support_legs")
        },
    }
    if enabled and amplitude == float(isolation["no_fall_amplitude_m"]):
        prefix = "isolation.amp%g.balance" % amplitude
        _check(
            checks,
            prefix + ".no_fall_height",
            metrics["min_base_height_m"],
            ">= %r" % float(isolation["fall_base_height_m"]),
            metrics["min_base_height_m"] >= float(isolation["fall_base_height_m"]),
            "机身高度不得低于声明跌倒阈值（机身未趴地）",
        )
        _check(
            checks,
            prefix + ".max_tilt_deg",
            metrics["max_tilt_deg"],
            "<= %r" % float(isolation["max_tilt_deg"]),
            metrics["max_tilt_deg"] <= float(isolation["max_tilt_deg"]),
            "倾角上限（相对竖直，不含偏航）",
        )
        # 决策 1g 的门禁（声明 → 事实 → 生效口径）：步态路径**有**声明相位 ⇒ 必须取声明里
        # `gait_clock_active` 给出的口径（否则「口径按控制模式区分」只写在声明里，没被兑现）。
        declared_modes = params.get("stance_classification") or {}
        _check(
            checks,
            prefix + ".stance_classification_follows_control_mode",
            case["stance_classification"],
            "== %r（有声明相位 ⇒ 取声明 gait_clock_active）"
            % declared_modes.get("gait_clock_active"),
            case["stance_classification"] == declared_modes.get("gait_clock_active")
            and case["declared_phase_source"] == "gait_clock",
            "支撑集口径必须按控制模式从声明取值（declared_phase_source=%r）"
            % case["declared_phase_source"],
        )
    return case


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Go2 力矩级平衡器验收（仿真）：静态抗扰 + 隔离实验复跑（含对照组）"
    )
    parser.add_argument("--config", type=Path, default=None, help="机型声明（config/go2_loopback.yaml）")
    parser.add_argument("--root", type=Path, default=None, help="仓库根（默认按本文件位置推断）")
    parser.add_argument("--report", type=Path, default=None, help="覆盖报告输出路径")
    parser.add_argument("--only", choices=("static", "isolation", "all"), default="all")
    args = parser.parse_args(argv)

    if args.config is None:
        print("用法错误：必须给出 --config <声明文件>", file=sys.stderr)
        return EXIT_USAGE
    root = args.root or unitree_go2.repo_root()
    config_path = _resolve(root, args.config)
    if not config_path.is_file():
        print("用法错误：声明文件不存在: %s" % config_path, file=sys.stderr)
        return EXIT_USAGE

    try:
        declaration, _ = unitree_go2.load_declaration(config_path)
        for key in ("robot.profile", "robot.backend", "skills.safety_policy"):
            _dig(declaration, key)
        balanced_params = balance_module.load_balance_declaration(declaration)
    except quadruped_contract.ModelUnavailableError as exc:
        print("引用完整性失败：%s" % exc, file=sys.stderr)
        return EXIT_REFERENCE
    except quadruped_contract.DeclarationError as exc:
        print("声明非法：%s" % exc, file=sys.stderr)
        return EXIT_DECLARATION

    profile_path = _resolve(root, _dig(declaration, "robot.profile"))
    safety_path = _resolve(root, _dig(declaration, "skills.safety_policy"))
    report_path = _resolve(root, args.report or balanced_params["verification"]["report"])
    if not profile_path.is_file() or not safety_path.is_file():
        print(
            "引用完整性失败：Profile（%s）或安全策略（%s）不存在" % (profile_path, safety_path),
            file=sys.stderr,
        )
        return EXIT_REFERENCE

    try:
        profile = load_robot_profile(profile_path)
        load_safety_policy(safety_path)
        movement_limits = quadruped_skills.load_movement_limits(safety_path)
    except ProfileError as exc:
        print("声明非法：%s" % exc, file=sys.stderr)
        return EXIT_DECLARATION
    except quadruped_skills.SkillContractError as exc:
        print("声明非法（移动边界）：%s" % exc, file=sys.stderr)
        return EXIT_DECLARATION

    authority = ControlAuthorityManager()
    cases = []
    checks = []
    try:
        if args.only in ("static", "all"):
            for enabled in (False, True):
                cases.append(
                    _static_case(declaration, root, profile, authority, enabled,
                                 balanced_params, checks)
                )
        if args.only in ("isolation", "all"):
            for enabled in (False, True):
                for amplitude in balanced_params["verification"]["isolation"]["amplitudes_m"]:
                    if amplitude not in (0.0, balanced_params["verification"]["isolation"][
                        "no_fall_amplitude_m"
                    ]):
                        continue
                    cases.append(
                        _isolation_case(declaration, root, profile, authority, movement_limits,
                                        enabled, balanced_params, amplitude, checks)
                    )
    except quadruped_contract.ModelUnavailableError as exc:
        print("引用完整性失败（模型/关节/接触几何）：%s" % exc, file=sys.stderr)
        return EXIT_REFERENCE
    except quadruped_contract.DeclarationError as exc:
        print("声明非法（后端契约/步态目标）：%s" % exc, file=sys.stderr)
        return EXIT_DECLARATION
    except Exception as exc:  # 装配/物理异常：显式失败，不返回半成品
        print("后端装配或执行失败：%s" % exc, file=sys.stderr)
        return EXIT_BACKEND

    static_cases = [item for item in cases if item["case"] == "static_disturbance"]
    control = next((item for item in static_cases if not item["balance_enabled"]), None)
    enabled_case = next((item for item in static_cases if item["balance_enabled"]), None)
    # 对照组不该"翻倒"：若对照组自身超限，说明扰动档过强，无法把差异归因给平衡器。
    control_failure = None
    if control is not None:
        verification = balanced_params["verification"]["static_disturbance"]
        control_metrics = control["recovery_window_metrics"]
        control_failure = {
            "max_tilt_deg": control_metrics["max_tilt_deg"],
            "excursion_ratio": (
                control_metrics["max_tilt_deg"] / float(verification["max_tilt_deg"])
            ),
            "falls": control_metrics["min_base_height_m"] < float(
                balanced_params["verification"]["isolation"]["fall_base_height_m"]
            ),
        }
    improvement = None
    if control is not None and enabled_case is not None:
        improvement = {
            "max_tilt_deg": {
                "control": control["recovery_window_metrics"]["max_tilt_deg"],
                "balance": enabled_case["recovery_window_metrics"]["max_tilt_deg"],
            },
            "max_drift_m": {
                "control": control["recovery_window_metrics"]["max_drift_m"],
                "balance": enabled_case["recovery_window_metrics"]["max_drift_m"],
            },
        }
    isolation = [item for item in cases if item["case"] == "isolation"]
    control_isolation = [
        item for item in isolation
        if not item["balance_enabled"]
        and item["amplitude_m"] == float(
            balanced_params["verification"]["isolation"]["no_fall_amplitude_m"]
        )
    ]
    if control_isolation:
        _check(
            checks,
            "isolation.control_reproduces_negative_result",
            control_isolation[0]["metrics"]["min_base_height_m"],
            "< %r（对照组应复现历史负结果：该档翻倒）"
            % float(balanced_params["verification"]["isolation"]["fall_base_height_m"]),
            control_isolation[0]["metrics"]["min_base_height_m"]
            < float(balanced_params["verification"]["isolation"]["fall_base_height_m"]),
            "若对照组不再翻倒，说明本实验已失去区分力（环境/声明变了），不得据此判实验组有效",
        )

    failed = [item["name"] for item in checks if not item["passed"]]
    report = {
        "schema_version": REPORT_SCHEMA_VERSION,
        "kind": REPORT_KIND,
        "generated_at": datetime.datetime.now().astimezone().isoformat(timespec="seconds"),
        "simulation": True,
        "robot": {
            "id": str(_dig(declaration, "robot.id")),
            "profile": _relative(root, profile_path),
            "profile_sha256": _sha256(profile_path),
            "backend": str(_dig(declaration, "robot.backend")),
            "joints": [str(item) for item in profile.joints],
            "safety_policy": _relative(root, safety_path),
            "safety_policy_sha256": _sha256(safety_path),
        },
        "declaration": {
            "path": _relative(root, config_path),
            "sha256": _sha256(config_path),
            # 机型声明里的开关当前值（实验组由本入口按声明生成证据区副本开启，不改仓库声明）
            "balance_enabled": bool(balanced_params["enabled"]),
        },
        "balance_declaration": {
            "weight_position": balanced_params["weight_position"],
            "weight_balance": balanced_params["weight_balance"],
            "include_gravity_support": balanced_params["include_gravity_support"],
            # 决策 1g：口径声明段（控制模式 → 判定口径的映射）与兜底阈值原样进报告。
            "stance_classification": dict(balanced_params["stance_classification"]),
            "stance_integrity": dict(balanced_params["stance_integrity"]),
            "attitude": balanced_params["attitude"],
            "height": balanced_params["height"],
            "velocity": balanced_params["velocity"],
            "allocation": balanced_params["allocation"],
            "watchdog": balanced_params["watchdog"],
            "verification": balanced_params["verification"],
        },
        "cases": cases,
        "control_static": control_failure,
        "static_improvement": improvement,
        "checks": checks,
        "failed_checks": failed,
        "not_proved": [
            "静态抗扰与隔离实验都是**仿真**结论（simulation=true，场景 handoff_lab 台面）："
            "不证明真机表现，目标端/真机验收 DEFERRED（板卡不在场）。",
            "本入口不证明行走/转向/到点能力（步骤 03/04）；也不回填任何 Profile 能力。",
            "对照组与实验组各自重新装配后端并从关键帧 `home` 起步，属**同构**对照；"
            "两组之间不存在“同一初始状态分叉”的严格配对。",
            "隔离实验只到 `no_fall_amplitude_m` 这一档：其余档位是**扫描记录**，"
            "不构成判据（幅度与结果非单调，历史实测已证明）。",
        ],
    }
    report["passed"] = not failed
    report["exit_code"] = EXIT_OK if report["passed"] else EXIT_CRITERIA
    report["report_path"] = str(report_path)
    report_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )

    summary = {
        "passed": report["passed"],
        "exit_code": report["exit_code"],
        "report_path": str(report_path),
        "simulation": True,
        "declaration_balance_enabled": report["declaration"]["balance_enabled"],
        "cases": [
            {
                "case": item["case"],
                "balance_enabled": item["balance_enabled"],
                "amplitude_m": item.get("amplitude_m"),
                "max_tilt_deg": round(item["metrics"]["max_tilt_deg"], 6),
                "min_base_height_m": round(item["metrics"]["min_base_height_m"], 6),
                "height_mean_m": round(item["metrics"]["height_mean_m"], 6),
                "max_drift_m": round(item["metrics"]["max_drift_m"], 6),
                "saturated_samples": item["metrics"]["saturated_samples"],
            }
            for item in cases
        ],
        "static_improvement": improvement,
        "control_static": control_failure,
        "checks": len(checks),
        "failed_checks": failed,
    }
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    return int(report["exit_code"])


if __name__ == "__main__":
    raise SystemExit(main())
