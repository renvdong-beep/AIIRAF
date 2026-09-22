#!/usr/bin/env python3
"""落足点规划（迈步式 crawl）**专项验收**入口。

与 `verify_go2_gait_in_place.py` 的关系
--------------------------------------
- **同一套采样与相位约定、同一条生产控制路径**（`backend.gait_in_place`）；
- **不同的判据集**：本入口消费 `gait.foothold.verification`（落点误差 / 接触达标率 /
  周期净漂移 / 峰值机身位移 + 安全类判据）；既有「原地踏步」的 20 项判据与阈值**一字不改**，
  仍由 `verify_go2_gait_in_place.py` 强制执行（两条入口并存，互不放宽）。

fail-closed 规则（缺依据不得判通过）
------------------------------------
1. `gait.foothold.mode` 必须是 `per_phase`（static 模式下没有可验收的落点内容 ⇒ 退 2）；
2. 采样必须带 `foot_trunk_m`（足端躯干系位置）⇒ 否则退 5，不写半份报告；
3. 稳态窗必须非空，且每条腿在稳态窗内至少有两个落点等级可比。

用法
----
    # 真实验收（仿真；结论只适用 simulation=true）
    PYTHONPATH=src /usr/bin/python3 scripts/verify_go2_crawl.py \\
        --config build/iraf-24h-3/step05/per-phase-crawl.yaml

    # 判据自检（**不跑仿真**：证明判据集不是恒真/恒假）
    /usr/bin/python3 scripts/verify_go2_crawl.py --self-check
"""

import argparse
import copy
import datetime
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT / "src") not in sys.path:
    sys.path.insert(0, str(ROOT / "src"))

import yaml  # noqa: E402

from iraf_adapters.unitree import gait  # noqa: E402
from iraf_adapters.unitree import quadruped as quadruped_contract  # noqa: E402
from iraf_adapters.unitree import unitree_go2  # noqa: E402
from iraf_core.authority import ControlAuthorityManager  # noqa: E402
from iraf_core.profile import ProfileError, load_robot_profile  # noqa: E402
from iraf_skills import quadruped as quadruped_skills  # noqa: E402

EXIT_OK = 0
EXIT_USAGE = 1
EXIT_DECLARATION = 2
EXIT_REFERENCE = 3
EXIT_BACKEND = 4
EXIT_CRITERIA = 5

REPORT_SCHEMA_VERSION = "iraf.go2-crawl-acceptance/v1"
SERIES_SCHEMA_VERSION = "iraf.go2-crawl-samples/v1"
LEASE_TTL_MARGIN = 3.0

DIRECTIONS = {"FL": {"x": 0.0, "y": -1.0}, "FR": {"x": 0.0, "y": 1.0},
              "RR": {"x": 0.0, "y": -1.0}, "RL": {"x": 0.0, "y": 1.0}}


def _dig(document, dotted, label="声明"):
    node = document
    for part in dotted.split("."):
        if not isinstance(node, dict) or part not in node:
            raise quadruped_contract.DeclarationError("%s 缺少声明键: %s" % (label, dotted))
        node = node[part]
    return node


def _resolve(root, value):
    path = Path(str(value))
    return path if path.is_absolute() else Path(root) / path


def _self_check():
    """判据自检：合成样本上「好样本必须过、坏样本必须挂、缺依据必须退 2/5」。

    为什么必须有：一条恒真的判据与一条有效的判据在「全绿」时看起来一样。
    本自检用**不跑仿真**的合成样本把两件事分开。
    """
    params = gait.load_gait_declaration(
        yaml.safe_load((ROOT / "config/go2_loopback.yaml").read_text(encoding="utf-8")),
        [str(item) for item in yaml.safe_load(
            (ROOT / "profiles/unitree_go2_mujoco.yaml").read_text(encoding="utf-8"))["spec"]["joints"]],
    )
    stride = 0.02
    foothold = {"mode": "per_phase", "stride_m": stride, "phase_order": ["FL", "FR", "RR", "RL"],
                "ramp_s": 0.0, "smooth_s": 0.2,
                "directions": {code: (item["x"], item["y"]) for code, item in DIRECTIONS.items()},
                "verification": dict(params["foothold"]["verification"])}
    crawl = copy.deepcopy(params)
    crawl["foothold"] = foothold
    period = float(crawl["period_s"])
    duty = float(crawl["duty_factor"])
    onset = 0.01

    def build(landing_error_ratio=0.005, contact_rate=1.0, drift_per_cycle=0.0, tilt=1.0,
              with_feet=True, levels_only_one=False):
        samples = []
        for index in range(1, int(10.0 / 0.01) + 1):
            elapsed = float(index) * 0.01
            samples.append({
                "time_s": onset + elapsed,
                "base_height_m": 0.28,
                "base_position_xy_m": [drift_per_cycle * elapsed / period, 0.0],
                "tilt_deg": tilt,
                "ctrl_saturated": 0,
                "contact_n": {},
                **({"foot_trunk_m": {}} if with_feet else {}),
            })
        # ⚠ 两遍构造：相位必须用与 `assess_crawl` **完全相同的表达式**（`time_s − 首帧 time_s`）
        # 来算。首版用构造时的 `elapsed` 直接算相位 ⇒ 与判据差一个浮点舍入，恰好落在 duty
        # 边界上的帧会被判据算成「支撑相」而样本里还标着摆动相（接触 0）⇒ 好样本也挂在
        # `leg_*_stance_contact_rate` 上（自检首轮实测踩到，3 例 FAIL）。
        first_time = samples[0]["time_s"]
        for index, sample in enumerate(samples, start=1):
            judge_elapsed = sample["time_s"] - first_time
            for code in sorted(crawl["legs"]):
                offset = float(crawl["legs"][code]["phase_offset"])
                stance = ((judge_elapsed / period) + offset) % 1.0 < duty
                if stance:
                    sample["contact_n"][code] = 40.0 if (index % 100) < 100 * contact_rate else 0.0
                else:
                    sample["contact_n"][code] = 0.0
                if with_feet:
                    plan = gait.foothold_offset_m(crawl, code, judge_elapsed)
                    direction = foothold["directions"][code]
                    level = int(round((plan[0] * direction[0] + plan[1] * direction[1]) / stride))
                    if levels_only_one:
                        level = 0
                    base = [0.18, 0.0]
                    # 误差按**等级沿方向**注入（= 把落点位移按比例缩放）：本判据测的是
                    # 「落点位移是否兑现」（两个等级之差），两件事会让用例失效：
                    # ① 所有帧同加常量 ⇒ 差分里抵消（首版踩到，超限用例假通过）；
                    # ② 在两个轴上都加误差 ⇒ hypot 后 0.000283 > 0.000227，好样本被判挂（第二版踩到）。
                    gain = 1.0 + landing_error_ratio
                    sample["foot_trunk_m"][code] = [
                        base[0] + level * gain * stride * direction[0],
                        base[1] + level * gain * stride * direction[1],
                        -0.26,
                    ]
        return samples

    cases = []
    good = gait.assess_crawl(build(), crawl, 15.0)
    cases.append(("好样本必须全过", not good["failed_checks"], "失败项=%s" % good["failed_checks"]))
    bad_landing = gait.assess_crawl(build(landing_error_ratio=0.5), crawl, 15.0)
    cases.append(("落点误差超限必须挂", bool([n for n in bad_landing["failed_checks"] if "landing_error" in n]),
                  "失败项=%s" % bad_landing["failed_checks"]))
    bad_contact = gait.assess_crawl(build(contact_rate=0.5), crawl, 15.0)
    cases.append(("接触率不足必须挂", bool([n for n in bad_contact["failed_checks"] if "contact_rate" in n]),
                  "失败项=%s" % bad_contact["failed_checks"]))
    bad_drift = gait.assess_crawl(build(drift_per_cycle=0.05), crawl, 15.0)
    cases.append(("净漂移超限必须挂", bool([n for n in bad_drift["failed_checks"] if "drift" in n]),
                  "失败项=%s" % bad_drift["failed_checks"]))
    bad_tilt = gait.assess_crawl(build(tilt=30.0), crawl, 15.0)
    cases.append(("倾角超限必须挂", "max_tilt_deg" in bad_tilt["failed_checks"],
                  "失败项=%s" % bad_tilt["failed_checks"]))
    try:
        gait.assess_crawl(build(with_feet=False), crawl, 15.0)
        cases.append(("缺 foot_trunk_m 必须显式失败", False, "未抛 CommandRejectedError"))
    except quadruped_contract.CommandRejectedError as exc:
        cases.append(("缺 foot_trunk_m 必须显式失败", True, str(exc)[:60]))
    try:
        gait.assess_crawl(build(levels_only_one=True), crawl, 15.0)
        rejected = False
        detail = "未失败（判据在只有一个落点等级时应判失败）"
    except quadruped_contract.CommandRejectedError as exc:
        rejected = True
        detail = str(exc)[:60]
    if not rejected:
        result = gait.assess_crawl(build(levels_only_one=True), crawl, 15.0)
        rejected = all("landing_error" in name for name in result["failed_checks"]) and bool(result["failed_checks"])
        detail = "无法比较相邻周期落点 ⇒ 失败项=%s" % result["failed_checks"]
    cases.append(("只有一个落点等级必须失败", rejected, detail))
    static_params = copy.deepcopy(params)
    try:
        gait.assess_crawl(build(), static_params, 15.0)
        cases.append(("static 模式必须拒绝（防恒真通过）", False, "未抛 CommandRejectedError"))
    except quadruped_contract.CommandRejectedError as exc:
        cases.append(("static 模式必须拒绝（防恒真通过）", True, str(exc)[:60]))

    width = max(len(name) for name, _, _ in cases)
    print("落足点规划专项判据自检（合成样本，不跑仿真）")
    failed = 0
    for name, ok, detail in cases:
        print("  [%s] %-*s  %s" % ("PASS" if ok else "FAIL", width, name, detail))
        failed += 0 if ok else 1
    print("\n[harness] 用例 %d，通过 %d，失败 %d" % (len(cases), len(cases) - failed, failed))
    return EXIT_OK if not failed else EXIT_CRITERIA


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--config", type=Path, default=None, help="机型声明（必须 foothold.mode=per_phase）")
    parser.add_argument("--root", type=Path, default=None, help="仓库根（默认按本文件位置推断）")
    parser.add_argument("--report", type=Path, default=None, help="覆盖报告输出路径")
    parser.add_argument("--self-check", action="store_true", help="判据自检（合成样本，不跑仿真）")
    args = parser.parse_args(argv)

    if args.self_check:
        return _self_check()
    if args.config is None:
        print("用法错误：必须给出 --config <声明文件>（或 --self-check）", file=sys.stderr)
        return EXIT_USAGE
    root = args.root or unitree_go2.repo_root()
    config_path = _resolve(root, args.config)
    if not config_path.is_file():
        print("用法错误：声明文件不存在: %s" % config_path, file=sys.stderr)
        return EXIT_USAGE

    try:
        declaration, _ = unitree_go2.load_declaration(config_path)
        for key in ("robot.profile", "robot.backend", "skills.safety_policy",
                    "gait.foothold.verification.report", "gait.foothold.verification.duration_s"):
            _dig(declaration, key)
    except quadruped_contract.ModelUnavailableError as exc:
        print("引用完整性失败：%s" % exc, file=sys.stderr)
        return EXIT_REFERENCE
    except quadruped_contract.DeclarationError as exc:
        print("声明非法：%s" % exc, file=sys.stderr)
        return EXIT_DECLARATION

    mode = str(_dig(declaration, "gait.foothold.mode"))
    if mode != "per_phase":
        print(
            "声明非法：本入口只验收 foothold.mode=per_phase（落足点规划）；当前 mode=%r。" % mode
            + " static 模式下落点恒为中立足端、没有可验收的落点内容，不得判为通过。",
            file=sys.stderr,
        )
        return EXIT_DECLARATION

    profile_path = _resolve(root, _dig(declaration, "robot.profile"))
    safety_path = _resolve(root, _dig(declaration, "skills.safety_policy"))
    report_path = _resolve(root, args.report or _dig(declaration, "gait.foothold.verification.report"))
    if not profile_path.is_file() or not safety_path.is_file():
        print("引用完整性失败：Profile（%s）或安全策略（%s）不存在" % (profile_path, safety_path),
              file=sys.stderr)
        return EXIT_REFERENCE

    try:
        profile = load_robot_profile(profile_path)
        movement_limits = quadruped_skills.load_movement_limits(safety_path)
        params = gait.load_gait_declaration(declaration, profile.joints)
    except (ProfileError, quadruped_skills.SkillContractError,
            quadruped_contract.DeclarationError) as exc:
        print("声明非法：%s" % exc, file=sys.stderr)
        return EXIT_DECLARATION

    authority = ControlAuthorityManager()
    backend_key = str(_dig(declaration, "robot.backend"))
    from iraf_adapters.factory import KNOWN_BACKENDS, load_backend  # noqa: E402

    if backend_key not in KNOWN_BACKENDS:
        print("后端装配失败：robot.backend=%s 未在 factory.KNOWN_BACKENDS 登记" % backend_key,
              file=sys.stderr)
        return EXIT_BACKEND
    try:
        backend = load_backend(KNOWN_BACKENDS[backend_key], str(config_path), profile, authority)
    except quadruped_contract.ModelUnavailableError as exc:
        print("引用完整性失败（模型/关键帧/关节/接触几何不可用）：%s" % exc, file=sys.stderr)
        return EXIT_REFERENCE
    except quadruped_contract.DeclarationError as exc:
        print("声明非法（后端契约）：%s" % exc, file=sys.stderr)
        return EXIT_DECLARATION
    except Exception as exc:  # noqa: BLE001 - 模型编译等：显式失败，不返回半成品
        print("后端装配失败：%s" % exc, file=sys.stderr)
        return EXIT_BACKEND

    duration_s = float(params["foothold"]["verification"]["duration_s"])
    lease = authority.acquire(profile.name + "-mujoco", "crawl-acceptance",
                              ttl_seconds=duration_s * LEASE_TTL_MARGIN)
    try:
        execution = backend.gait_in_place(lease, duration_ms=int(duration_s * 1000.0))
    except quadruped_contract.ModelUnavailableError as exc:
        print("引用完整性失败（步态几何实测）：%s" % exc, file=sys.stderr)
        return EXIT_REFERENCE
    except (quadruped_contract.DeclarationError, quadruped_contract.CommandRejectedError) as exc:
        print("声明非法（步态目标/几何）：%s" % exc, file=sys.stderr)
        return EXIT_DECLARATION

    try:
        assessment = gait.assess_crawl(execution["samples"], params,
                                      movement_limits["max_tilt_moving_deg"])
    except quadruped_contract.CommandRejectedError as exc:
        print("判据无依据（fail-closed）：%s" % exc, file=sys.stderr)
        return EXIT_CRITERIA

    samples_path = report_path.with_name("samples.json")
    samples_path.parent.mkdir(parents=True, exist_ok=True)
    samples_path.write_text(
        json.dumps({"schema_version": SERIES_SCHEMA_VERSION, "simulation": True,
                    "gait": {"frequency_hz": params["frequency_hz"], "period_s": params["period_s"],
                             "foothold": {"mode": params["foothold"]["mode"],
                                          "stride_m": params["foothold"]["stride_m"]}},
                    "samples": execution["samples"]}, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )

    report = {
        "schema_version": REPORT_SCHEMA_VERSION,
        "kind": "go2-crawl-acceptance",
        "generated_at": datetime.datetime.now().astimezone().isoformat(timespec="seconds"),
        "simulation": True,
        "robot": profile.name,
        "config": str(config_path.relative_to(Path(root)) if config_path.is_relative_to(Path(root)) else config_path),
        "foothold": {"mode": params["foothold"]["mode"],
                     "stride_m": params["foothold"]["stride_m"],
                     "smooth_s": params["foothold"]["smooth_s"],
                     "phase_order": params["foothold"]["phase_order"],
                     "verification": params["foothold"]["verification"]},
        "checks": assessment["checks"],
        "failed_checks": assessment["failed_checks"],
        "metrics": assessment["metrics"],
        "series_path": str(samples_path),
        "passed": not assessment["failed_checks"],
        "exit_code": EXIT_OK if not assessment["failed_checks"] else EXIT_CRITERIA,
    }
    report["report_path"] = str(report_path)
    report_path.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")

    summary = {
        "kind": report["kind"], "simulation": True, "passed": report["passed"],
        "exit_code": report["exit_code"],
        "checks_total": len(report["checks"]), "failed_checks": report["failed_checks"],
        "net_drift_per_cycle_m": report["metrics"]["net_drift_per_cycle_m"],
        "peak_body_excursion_m": report["metrics"]["peak_body_excursion_m"],
        "max_tilt_deg": report["metrics"]["max_tilt_deg"],
        "per_leg": report["metrics"]["per_leg"],
        "report_path": str(report_path), "series_path": str(samples_path),
    }
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    if report["passed"]:
        print("GO2_CRAWL_ACCEPTANCE_PASSED", file=sys.stderr)
        return EXIT_OK
    for name in report["failed_checks"]:
        detail = next(item["detail"] for item in report["checks"] if item["name"] == name)
        print("· [%s] %s" % (name, detail), file=sys.stderr)
    print("GO2_CRAWL_ACCEPTANCE_FAILED", file=sys.stderr)
    return EXIT_CRITERIA


if __name__ == "__main__":
    raise SystemExit(main())
