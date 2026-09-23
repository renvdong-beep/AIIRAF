"""`mpc.gait_trot`：trot 步态 = 基准声明 + 差异覆盖，且合并结果必须过既有全门禁。

基准用**真实文件**（`config/go2_loopback.yaml`）而不是自造夹具：这样测的是真门禁，
且不需要把 14 个验收阈值抄进测试（抄进测试同样是第二份来源）。
"""

from __future__ import annotations

from pathlib import Path

import pytest
import yaml

from iraf_adapters.unitree import gait as gait_mod
from iraf_adapters.unitree.mpc import gait_trot
from iraf_adapters.unitree.mpc.contact import contact_table
from iraf_core.profile import load_robot_profile

REPO = Path(__file__).resolve().parents[2]
CONFIG = REPO / "config" / "go2_locomote.yaml"
BASE = REPO / "config" / "go2_loopback.yaml"
PROFILE = REPO / "profiles" / "unitree_go2_mujoco.yaml"

pytestmark = pytest.mark.skipif(not (CONFIG.is_file() and BASE.is_file() and PROFILE.is_file()),
                                reason="需要仓库内的真实声明文件")


def _documents():
    base = yaml.safe_load(BASE.read_text(encoding="utf-8")) or {}
    fragment = (yaml.safe_load(CONFIG.read_text(encoding="utf-8")) or {})["mpc_gait"]
    return base, fragment


def _joints():
    return load_robot_profile(PROFILE).joints


def test_merge_only_changes_the_declared_differences():
    base, fragment = _documents()
    merged = gait_trot.merge_trot_declaration(base, fragment)["gait"]
    assert merged["kind"] == "trot"
    assert merged["duty_factor"] == 0.6
    assert merged["frequency_hz"] == 3.0
    assert {code: leg["phase_offset"] for code, leg in merged["legs"].items()} == {
        "FL": 0.5, "FR": 0.0, "RL": 0.0, "RR": 0.5}
    assert "sway" not in merged and "foothold" not in merged
    # 继承项逐项等同基准（关节绑定/接触几何/验收阈值/机身阻尼/步高等）
    for key in ("stabilization", "verification", "step_height_m", "stance_clearance_m",
                "swing_profile", "ramp_s"):
        assert merged[key] == base["gait"][key], key
    for code, leg in base["gait"]["legs"].items():
        for joint_key in ("hip_joint", "thigh_joint", "calf_joint", "contact_geom"):
            assert merged["legs"][code][joint_key] == leg[joint_key]


def test_merged_declaration_passes_existing_gates_and_feeds_contact_table():
    base, fragment = _documents()
    params = gait_trot.load_trot_gait(base, fragment, _joints(),
                                      mpc_model={"gait_hz": 3.0})
    assert params["kind"] == "trot"
    assert params["period_s"] == pytest.approx(1.0 / 3.0)
    assert params["duty_factor"] == 0.6
    assert params["sway"] is None and params["foothold"] is None
    groups = sorted((round(g["offset"], 9), tuple(g["legs"])) for g in params["phase_groups"])
    assert groups == [(0.0, ("FR", "RL")), (0.5, ("FL", "RR"))]
    tab = contact_table(params, 0.0, params["period_s"] / 64.0, 64)
    assert set(tab.sum(axis=0).tolist()) == {2, 4}          # 无腾空、无单足/三足
    assert (tab.sum(axis=0) == 4).mean() == pytest.approx(2 * (0.6 - 0.5), abs=2.0 / 64.0)
    assert tab[0].tolist() == tab[3].tolist()               # FL == RR（对角）
    assert tab[1].tolist() == tab[2].tolist()               # FR == RL


def test_period_must_match_mpc_model_gait_hz():
    base, fragment = _documents()
    with pytest.raises(gait_mod.DeclarationError):
        gait_trot.load_trot_gait(base, fragment, _joints(), mpc_model={"gait_hz": 1.25})
    with pytest.raises(gait_mod.DeclarationError):
        gait_trot.load_trot_gait(base, fragment, _joints(), mpc_model={})


def test_loader_is_actually_running_on_the_merged_declaration():
    """把覆盖值改成非对角配对 ⇒ 必须被**既有** trot 门禁拦住（证明不是"合完就算数"）。"""
    base, fragment = _documents()
    broken = {"drop": ["sway", "foothold"],
              "overrides": {"kind": "trot", "duty_factor": 0.6,
                            "legs": {"FL": {"phase_offset": 0.25}}}}
    with pytest.raises(gait_mod.DeclarationError):
        gait_trot.load_trot_gait(base, broken, _joints())
    # 占空比越界（trot 下限 0.5）同样被拦
    bad_duty = {"drop": ["sway", "foothold"], "overrides": {"duty_factor": 0.4}}
    with pytest.raises(gait_mod.DeclarationError):
        gait_trot.load_trot_gait(base, bad_duty, _joints())


def test_malformed_fragments_fail_explicitly():
    base, fragment = _documents()
    with pytest.raises(gait_mod.DeclarationError):
        gait_trot.merge_trot_declaration(base, {"drop": ["sway", "no_such_section"],
                                                "overrides": {"kind": "trot"}})
    with pytest.raises(gait_mod.DeclarationError):
        gait_trot.merge_trot_declaration(base, {"drop": [], "overrides": {"duty": 0.6}})
    with pytest.raises(gait_mod.DeclarationError):
        gait_trot.merge_trot_declaration(base, {"drop": [], "overrides": {"legs": {"XX": {}}}})
    with pytest.raises(gait_mod.DeclarationError):
        gait_trot.merge_trot_declaration(base, {"drop": [], "overrides": {}})
    with pytest.raises(gait_mod.DeclarationError):
        gait_trot.merge_trot_declaration({}, fragment)


def test_end_to_end_from_real_config_file():
    params = gait_trot.load_trot_gait_from_config(CONFIG, _joints(), root=REPO)
    assert params["kind"] == "trot" and params["period_s"] == pytest.approx(1.0 / 3.0)
    # 基准路径写错 ⇒ 显式失败（不静默退回默认）
    base, fragment = _documents()
    broken = dict(fragment)
    broken["base_declaration"] = "config/does_not_exist.yaml"
    path = REPO / "build" / "tmp-gait-trot-test.yaml"
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(yaml.safe_dump({"mpc_gait": broken, "mpc_model": {"gait_hz": 3.0}}),
                    encoding="utf-8")
    try:
        with pytest.raises(gait_mod.DeclarationError):
            gait_trot.load_trot_gait_from_config(path, _joints(), root=REPO)
    finally:
        path.unlink()
