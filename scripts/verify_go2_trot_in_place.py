"""Go2 参数化 trot 原地踏步验收入口（统一步态入口）：`--config <声明>` → 报告 + 退出码。

本文件只是**入口层**：解析参数、装配后端、取租约、调用步态路径、把判定委托给
`iraf_adapters.unitree.gait.assess_trot`、映射退出码。全部数字与路径来自声明：

- 步态参数与验收判据：`config/go2_loopback.yaml` 的 `gait` 段；
- 关节身份与限位：`profiles/unitree_go2_mujoco.yaml`；
- 移动姿态上限（倾角）：`profiles/safety/quadruped_lab.yaml` 的 `quadruped_limits`
  （倾角阈值**只在安全策略声明一次**，本脚本不写第二份）；
- 后端入口：机型声明的 `robot.backend`（经 `iraf_adapters.factory.KNOWN_BACKENDS` 解析）。

用法::

    PYTHONPATH=src /usr/bin/python3 scripts/verify_go2_trot_in_place.py --config config/go2_loopback.yaml

退出码：

- `0` 全部判据通过（报告 `passed: true`）
- `1` 用法错误（缺 `--config`、声明文件不存在）
- `2` 声明非法（步态段缺键/越界/相位不自洽、安全策略缺移动边界）
- `3` 引用完整性失败（Profile / 安全策略 / 被测模型 / 关节 / 接触几何不存在）
- `4` 后端装配失败（能力契约、模型编译）
- `5` 判据未通过（报告 `failed_checks` 逐条列出）

诚实边界：全部结论属于**仿真**（报告 `simulation: true`）；本入口证明的是「声明的步态参数下
原地踏步稳定且接触序列正确」，**不**证明行走/转向/到点能力（那是步骤 03/04），
也不含任何真机证据（目标端验收 DEFERRED，板卡不在场）。
"""

import argparse
import datetime
import hashlib
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from iraf_adapters.factory import KNOWN_BACKENDS, load_backend  # noqa: E402
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

REPORT_SCHEMA_VERSION = "iraf.go2-trot-in-place/v1"
REPORT_KIND = "go2-trot-in-place"

#: 租约 TTL 相对声明动作时长的倍数：TTL 必须覆盖一次完整动作，留 3 倍余量。
#: （技能路径的同一规则由 `profile_check --quadruped` 第 8 条门禁约束；本入口是验收路径。）
LEASE_TTL_MARGIN = 3.0


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


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Go2 参数化 trot 原地踏步验收（仿真）：稳定性 + 接触序列 + 控制量未饱和"
    )
    parser.add_argument("--config", type=Path, default=None, help="机型声明（config/go2_loopback.yaml）")
    parser.add_argument("--root", type=Path, default=None, help="仓库根（默认按本文件位置推断）")
    parser.add_argument("--report", type=Path, default=None, help="覆盖报告输出路径")
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
        for key in ("robot.profile", "robot.backend", "skills.safety_policy", "gait.verification.report"):
            _dig(declaration, key)
    except quadruped_contract.ModelUnavailableError as exc:
        print("引用完整性失败：%s" % exc, file=sys.stderr)
        return EXIT_REFERENCE
    except quadruped_contract.DeclarationError as exc:
        print("声明非法：%s" % exc, file=sys.stderr)
        return EXIT_DECLARATION

    profile_path = _resolve(root, _dig(declaration, "robot.profile"))
    safety_path = _resolve(root, _dig(declaration, "skills.safety_policy"))
    report_path = _resolve(root, args.report or _dig(declaration, "gait.verification.report"))
    if not profile_path.is_file() or not safety_path.is_file():
        print(
            "引用完整性失败：Profile（%s）或安全策略（%s）不存在" % (profile_path, safety_path),
            file=sys.stderr,
        )
        return EXIT_REFERENCE

    try:
        profile = load_robot_profile(profile_path)
        safety = load_safety_policy(safety_path)
        movement_limits = quadruped_skills.load_movement_limits(safety_path)
        locomotion = quadruped_skills.load_locomotion_declaration(
            declaration, movement_limits, quadruped_skills.load_quadruped_limits(safety_path)
        )
        params = gait.load_gait_declaration(declaration, profile.joints)
    except ProfileError as exc:
        print("声明非法：%s" % exc, file=sys.stderr)
        return EXIT_DECLARATION
    except quadruped_skills.SkillContractError as exc:
        print("声明非法（移动边界/停止语义）：%s" % exc, file=sys.stderr)
        return EXIT_DECLARATION
    except quadruped_contract.DeclarationError as exc:
        print("声明非法：%s" % exc, file=sys.stderr)
        return EXIT_DECLARATION

    # 跨声明自洽门禁：步态的控制周期必须等于 locomotion 声明的心跳周期
    # （同一事实的两条消费面：控制环频率由 control.frequency_hz 唯一来源给出）。
    control_hz = float(_dig(declaration, "control.frequency_hz"))
    period_ms = 1000.0 / control_hz
    heartbeat_ms = float(locomotion["heartbeat_period_ms"])
    cross_checks = [
        {
            "name": "heartbeat_matches_control_period",
            "value": {"control_period_ms": period_ms, "heartbeat_period_ms": heartbeat_ms},
            "expectation": "|control_period_ms - heartbeat_period_ms| <= 1e-9",
            "passed": abs(period_ms - heartbeat_ms) <= 1.0e-9,
            "detail": "步态每个控制周期一次心跳：频率来源是 control.frequency_hz，"
            "心跳周期声明在 locomotion.heartbeat_period_ms，两者必须一致",
        },
        {
            "name": "watchdog_covers_heartbeat",
            "value": {
                "watchdog_timeout_ms": movement_limits["watchdog_timeout_ms"],
                "heartbeat_period_ms": heartbeat_ms,
            },
            "expectation": "heartbeat_period_ms <= watchdog_timeout_ms",
            "passed": heartbeat_ms <= float(movement_limits["watchdog_timeout_ms"]),
            "detail": "心跳周期不得大于失联判定（阈值只在安全策略声明）",
        },
    ]

    authority = ControlAuthorityManager()
    backend_key = str(_dig(declaration, "robot.backend"))
    if backend_key not in KNOWN_BACKENDS:
        print(
            "后端装配失败：robot.backend=%s 未在 factory.KNOWN_BACKENDS 登记" % backend_key,
            file=sys.stderr,
        )
        return EXIT_BACKEND
    try:
        backend = load_backend(KNOWN_BACKENDS[backend_key], str(config_path), profile, authority)
    except quadruped_contract.ModelUnavailableError as exc:
        print("引用完整性失败（模型/关键帧/关节/接触几何不可用）：%s" % exc, file=sys.stderr)
        return EXIT_REFERENCE
    except quadruped_contract.DeclarationError as exc:
        print("声明非法（后端契约）：%s" % exc, file=sys.stderr)
        return EXIT_DECLARATION
    except Exception as exc:  # 模型编译、能力契约等：显式失败，不返回半成品
        print("后端装配失败：%s" % exc, file=sys.stderr)
        return EXIT_BACKEND

    duration_s = float(params["verification"]["duration_s"])
    lease = authority.acquire(
        profile.name + "-mujoco",
        "trot-in-place-acceptance",
        ttl_seconds=duration_s * LEASE_TTL_MARGIN,
    )
    try:
        execution = backend.trot_in_place(lease)
    except quadruped_contract.ModelUnavailableError as exc:
        print("引用完整性失败（步态几何实测）：%s" % exc, file=sys.stderr)
        return EXIT_REFERENCE
    except (quadruped_contract.DeclarationError, quadruped_contract.CommandRejectedError) as exc:
        print("声明非法（步态目标/几何）：%s" % exc, file=sys.stderr)
        return EXIT_DECLARATION

    try:
        assessment = gait.assess_trot(
            execution["samples"], params, movement_limits["max_tilt_moving_deg"]
        )
    except quadruped_contract.CommandRejectedError as exc:
        print("判据无依据：%s" % exc, file=sys.stderr)
        return EXIT_CRITERIA

    samples_path = report_path.with_name("samples.json")
    samples_path.parent.mkdir(parents=True, exist_ok=True)
    samples_path.write_text(
        json.dumps(
            {
                "schema_version": "iraf.go2-trot-in-place-samples/v1",
                "simulation": True,
                "gait": {"frequency_hz": params["frequency_hz"], "period_s": params["period_s"]},
                "samples": execution["samples"],
            },
            ensure_ascii=False,
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )

    failed = list(assessment["failed_checks"]) + [
        item["name"] for item in cross_checks if not item["passed"]
    ]
    report = {
        "schema_version": REPORT_SCHEMA_VERSION,
        "kind": REPORT_KIND,
        "generated_at": datetime.datetime.now().astimezone().isoformat(timespec="seconds"),
        "simulation": True,
        "robot": {
            "id": str(_dig(declaration, "robot.id")),
            "profile": _relative(root, profile_path),
            "profile_sha256": _sha256(profile_path),
            "backend": backend_key,
            "joints": [str(item) for item in profile.joints],
            "safety_policy": _relative(root, safety_path),
            "safety_policy_sha256": _sha256(safety_path),
        },
        "model": {
            "path": str(_dig(declaration, "model.file")),
            "sha256": _sha256(
                _resolve(root, _dig(declaration, "model.file"))
            ) if _resolve(root, _dig(declaration, "model.file")).is_file() else None,
        },
        "config": {
            "path": _relative(root, config_path),
            "sha256": _sha256(config_path),
            "gait": {
                "kind": params["kind"],
                "frequency_hz": params["frequency_hz"],
                "step_height_m": params["step_height_m"],
                "duty_factor": params["duty_factor"],
                "swing_profile": params["swing_profile"],
                "ramp_s": params["ramp_s"],
                "phase_groups": params["phase_groups"],
            },
            "control": {
                "frequency_hz": control_hz,
                "kp_nm_per_rad": float(_dig(declaration, "control.kp_nm_per_rad")),
                "kd_nm_s_per_rad": float(_dig(declaration, "control.kd_nm_s_per_rad")),
                "gravity_feedforward": bool(_dig(declaration, "control.gravity_feedforward")),
                "torque_limit_source": str(_dig(declaration, "control.torque_limit_source")),
            },
        },
        "execution": {
            "capability": execution["capability"],
            "path": execution["path"],
            "execution_id": execution["execution_id"],
            "fencing_token": execution["fencing_token"],
            "lease_ttl_s": execution["duration_ms"] / 1000.0 * LEASE_TTL_MARGIN,
            "control_cycles": execution["control_cycles"],
            "substeps_per_control": execution["substeps_per_control"],
            "geometry": execution["geometry"],
            "torque_limits_source": "model",
        },
        "trot": assessment["metrics"],
        "series_path": str(samples_path),
        "checks": assessment["checks"] + cross_checks,
        "failed_checks": failed,
        "not_proved": [
            "原地踏步 ≠ 行走/转向/到点能力：本报告只证明声明的步态参数下四腿协调、"
            "机身稳定、接触序列为对角小跑结构且控制量未饱和。",
            "落地冲击/腾空相等步态细节不在本步骤判据内：摆动相采用半个正弦轨迹（离地/落地高度为 0），"
            "未做落地软着陆整形。",
            "步态在仿真场景 handoff_lab 的台面上取得（simulation=true）；"
            "台面摩擦、地形、真机时延均不代表真机表现，目标端/真机验收 DEFERRED（板卡不在场）。",
            "本入口不经 TaskFlow/SkillRuntime/PolicyGateway：它是**控制器验收路径**（Provider 直调 + "
            "控制权租约）；能力面（skills/locomote 与 Profile capabilities 回填）属步骤 03。",
        ],
    }
    report["passed"] = not failed
    report["exit_code"] = EXIT_OK if report["passed"] else EXIT_CRITERIA
    report["report_path"] = str(report_path)
    report_path.write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )

    summary = {
        "passed": report["passed"],
        "exit_code": report["exit_code"],
        "report_path": str(report_path),
        "simulation": True,
        "duration_s": report["trot"]["duration_s"],
        "height_mean_m": report["trot"]["height_mean_m"],
        "height_std_m": report["trot"]["height_std_m"],
        "min_base_height_m": report["trot"]["min_base_height_m"],
        "max_tilt_deg": report["trot"]["max_tilt_deg"],
        "max_tracking_error_rad": report["trot"]["max_tracking_error_rad"],
        "ctrl_saturated_samples": report["trot"]["ctrl_saturated_samples"],
        "per_leg": {
            leg: {
                "stance_fraction": item["stance_fraction"],
                "clear_swing_cycles": item["clear_swing_cycles"],
                "cycles_observed": item["cycles_observed"],
            }
            for leg, item in report["trot"]["per_leg"].items()
        },
        "checks": len(report["checks"]),
        "failed_checks": report["failed_checks"],
    }
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    return int(report["exit_code"])


if __name__ == "__main__":
    raise SystemExit(main())
