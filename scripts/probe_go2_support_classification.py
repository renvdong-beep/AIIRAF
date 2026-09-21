#!/usr/bin/env python3
"""只读探针：**声明相位 × 实测接触** 交叉制表（步骤 02b 决策包，不选路、不改任何声明/实现）。

## 为什么需要它

步骤 02b 的 B1 落地后，wave/trot 双复评「同一批 9 项失败」，根因在台账里被记为
「mg 支撑按**实测接触力**判定的支撑集分摊 ⇒ 该抬未抬的腿被 ~mg/4 压在地面 + 支撑腿位置权重 0
⇒ 自锁」。**那条结论当时是判读，不是本探针口径下的实测**。本探针把它变成数字：

- 声明相位（支撑/摆动）由**声明的** `gait` 段算出（复用 `gait.leg_phase` / `gait.is_stance`，
  与验收判定 `_stance_profile` / `_clear_swing_cycles` **同一套原语**，不写第二份相位推断）；
- 实测接触由报告的逐采样 `contact_n[腿] ≥ 声明阈值` 判定（阈值来自声明的
  `gait.verification.contact_force_threshold_n`）；
- 交叉制表给出「声明摆动窗口内仍接触」的采样数 —— 即**自锁窗口的宽度**，
  以及「若按声明相位判定支撑集，会有多少采样被重新分类」（这是人工决策点 ① 的量化前提）。

## 口径

- 稳态窗口与验收判定一致：`elapsed = time_s − samples[0].time_s`，`elapsed < gait.ramp_s` 的采样丢弃。
- `declared_swing_contact_samples` 的含义：**该腿按声明应在摆动相、实测却仍接触**。
- 权重字段是**推导**不是测量：由「实测接触 → 声明权重映射」推出（映射本身来自代码契约：
  实测接触 ⇒ 支撑关节权重 `balance.stance_weight_position`，否则 `balance.weight_position`），
  JSON 里用 `derivation` 字段显式标注。缺 `balance` 段时该节为 `null`，不编默认值。

## 退出码

- `0` 探测完成（**不等于**验收通过：本探针只给数字，不判步态是否合格）
- `1` 用法错误（缺 `--config` / 文件不存在 / 参数非法）
- `2` 声明非法（`gait` / `balance` 段缺键、取值越界 —— 一律由生产加载器给出）
- `3` 引用完整性失败（报告或 samples 文件缺失 / JSON 破损 / `simulation` 非 true）
- `4` 不同源或结果退化：报告里的步态参数与声明不一致（**不得据此判读**），
  或稳态窗口内没有采样（抽不到样本 ⇒ 不得据此判一致）

## 复跑命令

```
PYTHONPATH=src /usr/bin/python3 scripts/probe_go2_support_classification.py \
  --config build/iraf-24h-2/02b/gait-wave-b1.yaml \
  --report build/acceptance/go2-gait-in-place/wave-b1/report.json \
  --out build/iraf-24h-2/02b/probe-support-classification-wave-b1.json
```
"""

import argparse
import hashlib
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from iraf_adapters.unitree import gait  # noqa: E402
from iraf_adapters.unitree import quadruped as quadruped_contract  # noqa: E402
from iraf_adapters.unitree import unitree_go2  # noqa: E402
from iraf_core.profile import ProfileError, load_robot_profile  # noqa: E402

EXIT_OK = 0
EXIT_USAGE = 1
EXIT_DECLARATION = 2
EXIT_REFERENCE = 3
EXIT_INCONSISTENT = 4

SCHEMA_VERSION = "iraf.go2-support-classification/v1"

#: 报告与声明的同源比对容差（浮点字面量在 YAML→JSON 往返后的噪声上界）。
SOURCE_TOLERANCE = 1.0e-12


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


def phase_offsets_from_groups(groups):
    """`报告.config.gait.phase_groups` → `{腿: 相位偏移}`。

    每个分组可含**多条腿**（trot 是对角两组、每组两条），必须逐条展开：只取 `legs[0]`
    会让 trot 的 RL/RR 落成 `None` ⇒ 同源门禁对 trot 恒失败（本探针首跑实测：
    门禁本身是对的，解析写错了；trot 报告被正确判成「不同源」而拒绝判读）。
    """
    offsets = {}
    for item in groups or []:
        if not isinstance(item, dict):
            continue
        for code in item.get("legs") or []:
            offsets[str(code)] = float(item["offset"])
    return offsets


def validate_samples(samples, legs):
    """逐采样结构校验：缺键/非数字即返回中文原因（调用方映射为退出码 3），绝不带 traceback。

    只校验**本探针要读的字段**：`time_s`、`base_height_m`、`contact_n[腿]`；
    不把「本探针用不到的字段」也算成契约（那不是本探针的事实）。
    """
    if not isinstance(samples, list) or not samples:
        return "逐采样文件里 samples 为空或不是数组"
    for index, sample in enumerate(samples):
        if not isinstance(sample, dict):
            return "samples[%d] 不是对象" % index
        for key in ("time_s", "base_height_m"):
            if key not in sample:
                return "samples[%d] 缺少键 %s" % (index, key)
            try:
                float(sample[key])
            except (TypeError, ValueError):
                return "samples[%d].%s 不是数字：%r" % (index, key, sample[key])
        contact = sample.get("contact_n")
        if not isinstance(contact, dict):
            return "samples[%d].contact_n 不是映射" % index
        for code in legs:
            if code not in contact:
                return "samples[%d].contact_n 缺少腿 %s" % (index, code)
            try:
                float(contact[code])
            except (TypeError, ValueError):
                return "samples[%d].contact_n.%s 不是数字：%r" % (index, code, contact[code])
    return None


def cross_tab(samples, params, legs, ramp_s, height_floor=None):
    """声明相位 × 实测接触 交叉制表（逐腿）。

    `height_floor`：只统计 `base_height_m >= height_floor` 的采样（用于「翻倒前」切片；
    取值只来自声明的 `gait.verification.fall_base_height_m`，本函数不造阈值）。
    返回 `(per_leg, measured_stance_legs_histogram, steady_samples)`；
    `per_leg[腿]` 的键：`declared_stance_contact` / `declared_stance_free` /
    `declared_swing_contact`（自锁窗口）/ `declared_swing_free`（真摆动）+
    接触力统计（`min/max/mean_contact_n`）+ 声明占空比对照。
    """
    from iraf_adapters.unitree import gait as _gait

    threshold = float(params["verification"]["contact_force_threshold_n"])
    duty = float(params["duty_factor"])
    onset = float(samples[0]["time_s"])
    cells = (
        "declared_stance_contact",
        "declared_stance_free",
        "declared_swing_contact",
        "declared_swing_free",
    )
    per_leg = {
        code: dict({cell: 0 for cell in cells}, contact_n=[], declared_stance=0, declared_swing=0)
        for code in legs
    }
    histogram = {}
    steady = 0
    for sample in samples:
        elapsed = float(sample["time_s"]) - onset
        if elapsed < float(ramp_s):
            continue
        if height_floor is not None and float(sample["base_height_m"]) < float(height_floor):
            continue
        steady += 1
        measured = []
        for code in legs:
            contact = float(sample["contact_n"][code])
            per_leg[code]["contact_n"].append(contact)
            in_contact = contact >= threshold
            if in_contact:
                measured.append(code)
            stance = _gait.is_stance(params, _gait.leg_phase(params, code, elapsed))
            phase_key = "declared_stance" if stance else "declared_swing"
            per_leg[code][phase_key] += 1
            per_leg[code][phase_key + ("_contact" if in_contact else "_free")] += 1
        histogram[len(measured)] = int(histogram.get(len(measured), 0)) + 1
    return per_leg, histogram, steady


def _summarize_leg(entry, duty, threshold):
    stance = int(entry["declared_stance"])
    swing = int(entry["declared_swing"])
    total = stance + swing
    forces = entry.pop("contact_n")
    return {
        "steady_samples": total,
        "declared_duty_factor": duty,
        "declared_swing_fraction": (float(swing) / total) if total else None,
        "declared_swing_samples": swing,
        "declared_stance_samples": stance,
        "declared_swing_contact_samples": int(entry["declared_swing_contact"]),
        "declared_swing_free_samples": int(entry["declared_swing_free"]),
        "declared_stance_contact_samples": int(entry["declared_stance_contact"]),
        "declared_stance_free_samples": int(entry["declared_stance_free"]),
        "measured_stance_fraction": (
            float(entry["declared_swing_contact"] + entry["declared_stance_contact"]) / total
            if total
            else None
        ),
        "self_lock_fraction_of_swing": (
            float(entry["declared_swing_contact"]) / swing if swing else None
        ),
        "min_contact_n": min(forces) if forces else None,
        "max_contact_n": max(forces) if forces else None,
        "mean_contact_n": (sum(forces) / len(forces)) if forces else None,
        "contact_force_threshold_n": threshold,
    }


def _derived_weight(entry, declared_swing_samples, stance_weight, swing_weight, enabled):
    """由「实测接触 → 声明权重映射」推导位置级权重（**推导值**，非直接测量）。

    `balance.enabled=false` 时**不存在**这条映射：步态路径根本不传 `torque_provider`
    （`unitree_go2.gait_in_place` 里 `torque_provider = ... if balance_params["enabled"] else None`）
    ⇒ 位置权重恒为 `weight_position`。此时必须如实回报「无映射」，不得照样推出一份接触→权重表。
    """
    if not enabled:
        return {
            "derivation": "平衡器未启用（balance.enabled=false）⇒ 无「实测接触 → 权重」映射，"
            "位置权重恒为声明 weight_position",
            "balance_enabled": False,
            "weight_position": swing_weight,
            "declared_swing_samples": declared_swing_samples,
            "declared_swing_samples_weighted_swing": declared_swing_samples,
        }
    return {
        "derivation": "由声明映射（实测接触 ⇒ 支撑关节权重；否则摆动权重）+ 实测接触推导，非直接测量",
        "balance_enabled": True,
        "stance_weight_position": stance_weight,
        "weight_position": swing_weight,
        "declared_swing_contact_samples": entry["declared_swing_contact_samples"],
        "declared_swing_contact_samples_weighted_stance": entry["declared_swing_contact_samples"],
        "declared_swing_free_samples_weighted_swing": entry["declared_swing_free_samples"],
        "declared_swing_samples": declared_swing_samples,
    }


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="只读探针：声明相位 × 实测接触 交叉制表（步骤 02b 决策包）"
    )
    parser.add_argument("--config", type=Path, default=None, help="机型声明（含 gait / balance 段）")
    parser.add_argument("--report", type=Path, default=None, help="验收报告 report.json")
    parser.add_argument(
        "--samples", type=Path, default=None, help="逐采样文件（默认取报告的兄弟文件 samples.json）"
    )
    parser.add_argument("--out", type=Path, default=None, help="探测结果 JSON 输出路径（可选）")
    parser.add_argument("--root", type=Path, default=None, help="仓库根（默认按本文件位置推断）")
    args = parser.parse_args(argv)

    if args.config is None or args.report is None:
        print("用法错误：必须给出 --config 与 --report", file=sys.stderr)
        return EXIT_USAGE
    root = args.root or unitree_go2.repo_root()
    config_path = _resolve(root, args.config)
    report_path = _resolve(root, args.report)
    for label, path in (("声明", config_path), ("报告", report_path)):
        if not path.is_file():
            print("用法错误：%s文件不存在: %s" % (label, path), file=sys.stderr)
            return EXIT_USAGE
    samples_path = (
        _resolve(root, args.samples) if args.samples is not None else report_path.parent / "samples.json"
    )
    if not samples_path.is_file():
        print("引用完整性失败：逐采样文件不存在: %s" % samples_path, file=sys.stderr)
        return EXIT_REFERENCE

    try:
        declaration, _ = unitree_go2.load_declaration(config_path)
        profile_path = _resolve(root, _dig(declaration, "robot.profile"))
        profile = load_robot_profile(profile_path)
        params = gait.load_gait_declaration(declaration, profile.joints)
    except quadruped_contract.DeclarationError as exc:
        print("声明非法：%s" % exc, file=sys.stderr)
        return EXIT_DECLARATION
    except ProfileError as exc:
        print("声明非法：%s" % exc, file=sys.stderr)
        return EXIT_DECLARATION
    except Exception as exc:  # 模型/Profile 引用不可用：显式失败，不返回半成品
        print("引用完整性失败（Profile/模型不可用）：%s" % exc, file=sys.stderr)
        return EXIT_REFERENCE

    try:
        report = json.loads(report_path.read_text(encoding="utf-8"))
        samples_doc = json.loads(samples_path.read_text(encoding="utf-8"))
    except (OSError, ValueError) as exc:
        print("引用完整性失败：报告或采样文件不可解析：%s" % exc, file=sys.stderr)
        return EXIT_REFERENCE
    if report.get("simulation") is not True or samples_doc.get("simulation") is not True:
        print(
            "引用完整性失败：报告/采样的 simulation 不是 true（本探针口径只适用仿真证据）",
            file=sys.stderr,
        )
        return EXIT_REFERENCE

    # 同源门禁（两道）：① 报告登记的声明 sha256 必须等于本声明的 sha256；
    # ② 报告里登记的步态参数必须与本声明的解析结果一致。任一条不满足即两份证据不同源，
    # 交叉制表的结论会漂到另一份声明上（这是「不得据此判读」的显式失败，不是告警）。
    try:
        declared_gait = _dig(report, "config.gait", "报告")
        reported_sha = str(_dig(report, "config.sha256", "报告"))
    except quadruped_contract.DeclarationError as exc:
        print("引用完整性失败：报告结构不符合契约：%s" % exc, file=sys.stderr)
        return EXIT_REFERENCE
    actual_sha = _sha256(config_path)
    if reported_sha != actual_sha:
        print(
            "不同源：报告记录的声明 sha256=%s 与本声明 sha256=%s 不同（报告由另一份声明生成）"
            "⇒ 不得据此判读" % (reported_sha, actual_sha),
            file=sys.stderr,
        )
        return EXIT_INCONSISTENT
    mismatches = []
    for key, actual in (
        ("kind", params["kind"]),
        ("duty_factor", float(params["duty_factor"])),
        ("ramp_s", float(params["ramp_s"])),
        ("frequency_hz", float(params["frequency_hz"])),
    ):
        reported = declared_gait.get(key)
        if reported is None:
            mismatches.append({"key": key, "report": None, "declaration": actual})
        elif isinstance(actual, str):
            if str(reported) != actual:
                mismatches.append({"key": key, "report": reported, "declaration": actual})
        elif abs(float(reported) - float(actual)) > SOURCE_TOLERANCE:
            mismatches.append({"key": key, "report": reported, "declaration": actual})
    reported_period = float(_dig(samples_doc, "gait.period_s", "采样文件"))
    if abs(reported_period - float(params["period_s"])) > SOURCE_TOLERANCE:
        mismatches.append(
            {"key": "period_s", "report": reported_period, "declaration": float(params["period_s"])}
        )
    offsets = phase_offsets_from_groups(declared_gait.get("phase_groups"))
    for code, leg in sorted(params["legs"].items()):
        reported = offsets.get(code)
        if reported is None or abs(reported - float(leg["phase_offset"])) > SOURCE_TOLERANCE:
            mismatches.append(
                {
                    "key": "phase_offset.%s" % code,
                    "report": reported,
                    "declaration": float(leg["phase_offset"]),
                }
            )
    if mismatches:
        print(
            "不同源：报告与本声明的步态参数不一致（%d 处）⇒ 不得据此判读：%s"
            % (len(mismatches), mismatches),
            file=sys.stderr,
        )
        return EXIT_INCONSISTENT

    legs = sorted(params["legs"])
    problem = validate_samples(samples_doc["samples"], legs)
    if problem is not None:
        print("引用完整性失败：逐采样文件结构不符合契约：%s" % problem, file=sys.stderr)
        return EXIT_REFERENCE
    per_leg_raw, histogram, steady = cross_tab(
        samples_doc["samples"], params, legs, float(params["ramp_s"])
    )
    if steady <= 0:
        print(
            "结果退化：稳态窗口（跳过 ramp_s=%.6f）内没有采样 ⇒ 不得据此判一致" % float(params["ramp_s"]),
            file=sys.stderr,
        )
        return EXIT_INCONSISTENT

    duty = float(params["duty_factor"])
    threshold = float(params["verification"]["contact_force_threshold_n"])
    balance_section = declaration.get("balance")
    per_leg = {}
    for code in legs:
        entry = per_leg_raw[code]
        summary = _summarize_leg(entry, duty, threshold)
        if isinstance(balance_section, dict):
            summary["derived_position_weight"] = _derived_weight(
                {
                    "declared_swing_contact_samples": summary["declared_swing_contact_samples"],
                    "declared_swing_free_samples": summary["declared_swing_free_samples"],
                },
                summary["declared_swing_samples"],
                float(_dig(balance_section, "stance_weight_position", "balance")),
                float(_dig(balance_section, "weight_position", "balance")),
                bool(_dig(balance_section, "enabled", "balance")),
            )
        else:
            summary["derived_position_weight"] = None
        per_leg[code] = summary

    # 「翻倒前」切片（**只**用声明的 fall_base_height_m；声明未给出即不做切片并如实登记，
    # 不造默认值）：生产路径的 wave 会翻倒，翻倒后的接触统计不是「步态」测量，
    # 不切片就无法排除「自由摆动采样其实来自倒地后的姿态」这一混淆。
    fall_floor = params["verification"].get("fall_base_height_m")
    pre_fall = None
    if fall_floor is not None:
        pre_legs_raw, pre_hist, pre_steady = cross_tab(
            samples_doc["samples"], params, legs, float(params["ramp_s"]), float(fall_floor)
        )
        pre_swing = sum(
            int(pre_legs_raw[code]["declared_swing"]) for code in legs
        )
        pre_contact = sum(
            int(pre_legs_raw[code]["declared_swing_contact"]) for code in legs
        )
        pre_fall = {
            "height_floor_m": float(fall_floor),
            "declared_swing_samples_total": pre_swing,
            "declared_swing_contact_samples_total": pre_contact,
            "self_lock_fraction": (float(pre_contact) / pre_swing) if pre_swing else None,
            "pre_fall_samples": pre_steady,
            "measured_stance_legs_histogram": {
                str(key): int(value) for key, value in sorted(pre_hist.items())
            },
            "per_leg_declared_swing_free_samples": {
                code: int(pre_legs_raw[code]["declared_swing_free"]) for code in legs
            },
        }

    swing_total = sum(item["declared_swing_samples"] for item in per_leg.values())
    swing_contact_total = sum(item["declared_swing_contact_samples"] for item in per_leg.values())
    result = {
        "schema_version": SCHEMA_VERSION,
        "simulation": True,
        "declaration": {
            "path": _relative(root, config_path),
            "sha256": _sha256(config_path),
        },
        "report": {
            "path": _relative(root, report_path),
            "sha256": _sha256(report_path),
        },
        "samples": {
            "path": _relative(root, samples_path),
            "sha256": _sha256(samples_path),
        },
        "gait": {
            "kind": params["kind"],
            "duty_factor": duty,
            "period_s": float(params["period_s"]),
            "frequency_hz": float(params["frequency_hz"]),
            "ramp_s": float(params["ramp_s"]),
            "step_height_m": float(params["step_height_m"]),
            "phase_offsets": {code: float(params["legs"][code]["phase_offset"]) for code in legs},
        },
        "contact_force_threshold_n": threshold,
        "samples_total": len(samples_doc["samples"]),
        "steady_state_samples": steady,
        "measured_stance_legs_histogram": {str(key): int(value) for key, value in sorted(histogram.items())},
        "per_leg": per_leg,
        "self_lock": {
            "declared_swing_samples_total": swing_total,
            "declared_swing_contact_samples_total": swing_contact_total,
            "self_lock_fraction": (float(swing_contact_total) / swing_total) if swing_total else None,
            "legs_with_no_free_swing_sample": [
                code
                for code in legs
                if per_leg[code]["declared_swing_free_samples"] == 0
            ],
        },
        "balance_weight_derivation_available": isinstance(balance_section, dict),
        "pre_fall": pre_fall,
        "pre_fall_note": (
            None
            if pre_fall is not None
            else "声明未给出 gait.verification.fall_base_height_m ⇒ 不做翻倒前切片（不造默认值）"
        ),
        "notes": [
            "本探针只做交叉制表：给出「声明摆动窗口内仍接触」的采样数，不判步态是否合格。",
            "declared_swing_contact_samples 即「按声明应在摆动相、实测仍接触」的采样数 = 自锁窗口宽度。",
            "全部结论 simulation=true；真机/目标端验收 DEFERRED（板卡不在场）。",
        ],
    }

    text = json.dumps(result, ensure_ascii=False, indent=2)
    print(text)
    if args.out is not None:
        out_path = _resolve(root, args.out)
        out_path.parent.mkdir(parents=True, exist_ok=True)
        out_path.write_text(text + "\n", encoding="utf-8")
        print("已写入: %s" % _relative(root, out_path), file=sys.stderr)
    return EXIT_OK


if __name__ == "__main__":
    sys.exit(main())
