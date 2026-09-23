"""trot 步态声明：**基准声明 + 差异覆盖 → 既有门禁校验**（A6a-3b 第②步下半）。

要解决的问题：接触表需要 `period_s` / `duty_factor` / 四腿 `phase_offset`，而这些只能来自声明
（铁律 5.3）。但仓库里唯一带 `phase_offset` 的 `gait:` 段是 **wave**（`config/go2_loopback.yaml`，
其 `verification.report` 仍是 `go2-trot-in-place/report.json` ⇒ 该段本就是 trot 声明改的）。
若在 `config/go2_locomote.yaml` 里**再抄一整份** trot 声明，14 个验收阈值、关节绑定、接触几何、
机身阻尼都会出现第二份来源（违反移植清单 §1.1）。

⇒ 本模块只做三件事，且**不含任何数字常量**（覆盖值全部来自 `config/go2_locomote.yaml` 的 `mpc_gait` 段）：

  1. 深拷贝基准声明的 `gait:` 段；
  2. 按 `drop` 删除段（删不存在的键 ⇒ 显式失败，防止 drop 名字写错后静默失效）、
     按 `overrides` 覆盖白名单键（多余键 ⇒ 显式失败，防止假声明）；
  3. 把合并结果交给 `gait.load_gait_declaration` 跑**全部门禁**（trot 相位结构、duty 区间、
     `profile_bins` 偶数、腿部关节必须在 Profile 关节清单内…），再叠加 MPC 自洽门禁：
     `period_s` 必须等于 `1 / mpc_model.gait_hz`（horizon×dt = 步态周期是接触表相位的前提）。

**已完成的对照**（2026-09-23，`build/research/mpc-repo/verify_contact_parity.py`）：
本模块解析出的 `frequency_hz = 3.0`、`duty_factor = 0.6`、按 `LEG_ORDER` 排开的相位偏移
`[0.5, 0.0, 0.0, 0.5]` 与上游 `GAIT_HZ` / `DUTY` / `PHASE_OFFSET` 逐项相等，且由此喂出的接触表
在上游同一 `t0/dt/N` 下 **400 组 × N=16 逐位一致（0 处不一致）** ⇒ 本模块的封禁已解除。
"""

from __future__ import annotations

import copy
from pathlib import Path

import yaml

from iraf_adapters.unitree import gait as gait_mod

__all__ = ["ALLOWED_OVERRIDE_KEYS", "merge_trot_declaration", "load_trot_gait",
           "load_trot_gait_from_config"]

#: 允许覆盖的 `gait` 段键。白名单（而非黑名单）是刻意的：写错名字的"覆盖"会变成
#: 无人消费的假声明，必须显式失败。
ALLOWED_OVERRIDE_KEYS = ("kind", "duty_factor", "frequency_hz", "legs")


def _require_mapping(value, label):
    if not isinstance(value, dict) or not value:
        raise gait_mod.DeclarationError("%s 必须是非空映射，实际: %r" % (label, value))
    return value


def merge_trot_declaration(base_document, fragment):
    """`(base_document, mpc_gait 段) → {"gait": 合并后的段}`。所有校验失败均抛 `DeclarationError`。"""
    base_section = (base_document or {}).get("gait")
    if not isinstance(base_section, dict):
        raise gait_mod.DeclarationError("基准声明缺少 gait 段（%r）" % (base_document and "gait",))
    fragment = _require_mapping(fragment, "mpc_gait")

    drop = fragment.get("drop", [])
    if not isinstance(drop, list):
        raise gait_mod.DeclarationError("mpc_gait.drop 必须是列表，实际: %r" % (drop,))
    overrides = _require_mapping(fragment.get("overrides"), "mpc_gait.overrides")
    unknown = [key for key in overrides if key not in ALLOWED_OVERRIDE_KEYS]
    if unknown:
        raise gait_mod.DeclarationError(
            "mpc_gait.overrides 含不允许的键 %s（白名单: %s）"
            % (unknown, list(ALLOWED_OVERRIDE_KEYS))
        )

    merged = copy.deepcopy(base_section)
    for key in drop:
        if key not in merged:
            raise gait_mod.DeclarationError(
                "mpc_gait.drop 里的 %r 在基准声明中不存在（drop 名字写错会静默失效，故显式失败）"
                % (key,)
            )
        merged.pop(key)
    for key, value in overrides.items():
        if key != "legs":
            merged[key] = value
    leg_overrides = overrides.get("legs")
    if leg_overrides is not None:
        leg_overrides = _require_mapping(leg_overrides, "mpc_gait.overrides.legs")
        legs = merged.get("legs")
        if not isinstance(legs, dict):
            raise gait_mod.DeclarationError("基准声明的 gait.legs 必须是映射")
        for code, patch in leg_overrides.items():
            if code not in legs:
                raise gait_mod.DeclarationError(
                    "mpc_gait.overrides.legs 里的 %r 不在基准声明的腿清单内: %s"
                    % (code, sorted(legs))
                )
            patch = _require_mapping(patch, "mpc_gait.overrides.legs.%s" % code)
            legs[code].update(patch)
    return {"gait": merged}


def load_trot_gait(base_document, fragment, profile_joints, mpc_model=None):
    """合并 → 既有全门禁校验 → 返回规范化参数（并做 MPC 自洽门禁）。"""
    merged = merge_trot_declaration(base_document, fragment)
    params = gait_mod.load_gait_declaration(merged, profile_joints)
    if mpc_model is not None:
        if not isinstance(mpc_model, dict) or "gait_hz" not in mpc_model:
            raise gait_mod.DeclarationError(
                "mpc_model 段缺少 gait_hz（步态周期与 horizon×dt 的自洽关系必须有唯一来源）"
            )
        declared_period = 1.0 / float(mpc_model["gait_hz"])
        if abs(float(params["period_s"]) - declared_period) > 1.0e-12:
            raise gait_mod.DeclarationError(
                "步态周期不自洽：trot 步态声明给出 %r s，mpc_model.gait_hz=%r 给出 %r s"
                % (params["period_s"], mpc_model["gait_hz"], declared_period)
            )
    return params


def load_trot_gait_from_config(config_path, profile_joints, *, root=None):
    """从 `config/go2_locomote.yaml` 读 `mpc_gait` 段与基准声明路径，返回规范化参数。"""
    config_path = Path(config_path)
    document = yaml.safe_load(config_path.read_text(encoding="utf-8")) or {}
    fragment = document.get("mpc_gait")
    if not isinstance(fragment, dict):
        raise gait_mod.DeclarationError("声明缺少 mpc_gait 段（trot 步态来源必须显式声明）")
    base_ref = fragment.get("base_declaration")
    if not isinstance(base_ref, str) or not base_ref:
        raise gait_mod.DeclarationError("mpc_gait.base_declaration 必须是非空字符串路径")
    base_path = Path(base_ref)
    if not base_path.is_absolute():
        base_path = ((Path(root) if root is not None else config_path.parent.parent) / base_ref)
    if not base_path.is_file():
        raise gait_mod.DeclarationError(
            "mpc_gait.base_declaration 指向的文件不存在: %s" % (base_path,)
        )
    base_document = yaml.safe_load(base_path.read_text(encoding="utf-8")) or {}
    return load_trot_gait(base_document, fragment, profile_joints,
                          mpc_model=document.get("mpc_model"))
