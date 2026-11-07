# -*- coding: utf-8 -*-
"""SceneRegistry 单元测试（阶段 A · A2）

覆盖计划里 A-T1 ~ A-T10 与解析/热加载/封签行为。
"""

from __future__ import annotations

import sys
from pathlib import Path

import pytest
import yaml

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "src"))

from iraf_core.scene_registry import (  # noqa: E402
    E_CAPABILITY,
    E_REGISTRY_INVALID,
    E_RESOURCE_NOT_FOUND,
    E_SCENE_AMBIGUOUS,
    E_SCENE_NOT_FOUND,
    RegistryError,
    SceneRegistry,
)

PROFILE = """\
robot:
  name: piper
  capabilities: [move_joint, pick_object, place_object, stop, reset_scene]
"""

SCENE = """\
schema: iraf.scene/v1
scene_id: piper-pick
provider_id: iraf/piper-pick
profile: profiles/piper.yaml
safety_policy: profiles/safety/simulation_lab.yaml
simulation: true
max_concurrency: 1
lease_ttl_s: 60
resources:
  - id: piper
    profile: profiles/piper.yaml
    capabilities: [pick_object, stop]
"""

PROVIDER = """\
schema: iraf.provider/v1
provider_id: iraf/piper-pick
scene_id: piper-pick
token_env: IRAF_PROVIDER_TOKEN
hosts:
  - id: x86-1
    endpoint: 10.203.247.121:50052
    capacity: {cores: 4}
    priority: 10
  - id: x86-2
    endpoint: 10.203.247.128:50052
    capacity: {cores: 16}
    priority: 20
schedule: static
health: {interval_s: 5, timeout_s: 2, failover: next}
allowed_hosts: [10.203.247.121, 10.203.247.128]
"""


def write_tree(root: Path, scene: str = SCENE, provider: str = PROVIDER,
               scene_name: str = "piper-pick.yaml",
               provider_name: str = "iraf-piper-pick.yaml") -> None:
    (root / "scenes").mkdir(parents=True, exist_ok=True)
    (root / "providers").mkdir(parents=True, exist_ok=True)
    (root / "profiles").mkdir(parents=True, exist_ok=True)
    (root / "scenes" / scene_name).write_text(scene, encoding="utf-8")
    (root / "providers" / provider_name).write_text(provider, encoding="utf-8")
    (root / "profiles" / "piper.yaml").write_text(PROFILE, encoding="utf-8")


def sealed(tmp_path: Path, **kw) -> SceneRegistry:
    write_tree(tmp_path, **kw)
    SceneRegistry(root=tmp_path, profiles_root=tmp_path).seal()
    return SceneRegistry.load(root=tmp_path, profiles_root=tmp_path)


# ---------------------------------------------------------------- 正常路径 ---
def test_load_and_resolve(tmp_path):
    reg = sealed(tmp_path)
    assert reg.errors == [], reg.errors
    r = reg.resolve("piper-pick")
    assert r.scene.scene_id == "piper-pick"
    assert r.resource.id == "piper"
    assert r.host.id == "x86-1"          # static：priority 最小
    assert r.host.port == "50052"


def test_resolve_by_resource_and_capability(tmp_path):
    reg = sealed(tmp_path)
    r = reg.resolve("piper-pick", resource_id="piper", capability="pick_object")
    assert r.resource.capabilities == ["pick_object", "stop"]


def test_default_scene_fallback(tmp_path):
    reg = sealed(tmp_path)
    r = reg.resolve(None, default_scene="piper-pick")
    assert r.scene.scene_id == "piper-pick"


def test_status_snapshot(tmp_path):
    reg = sealed(tmp_path)
    st = reg.status()
    assert st["scenes"][0]["scene_id"] == "piper-pick"
    assert st["providers"][0]["provider_id"] == "iraf/piper-pick"
    assert st["registry"]["errors"] == []


# ---------------------------------------------------------------- 校验失败 ---
def test_digest_mismatch(tmp_path):
    write_tree(tmp_path)
    reg = SceneRegistry(root=tmp_path, profiles_root=tmp_path)
    with pytest.raises(RegistryError) as ei:
        reg.reload(force=True)
    assert ei.value.code == E_REGISTRY_INVALID
    assert "digest" in ei.value.detail


def test_simulation_missing(tmp_path):
    with pytest.raises(RegistryError) as ei:
        sealed(tmp_path, scene=SCENE.replace("simulation: true\n", ""))
    assert "simulation" in ei.value.detail


def test_scene_filename_must_match_id(tmp_path):
    with pytest.raises(RegistryError) as ei:
        sealed(tmp_path, scene_name="other.yaml")
    assert "文件名必须等于 scene_id" in ei.value.detail


def test_provider_filename_must_match_id(tmp_path):
    with pytest.raises(RegistryError) as ei:
        sealed(tmp_path, provider_name="piper-pick.yaml")
    assert "provider_id 的 /→- 形式" in ei.value.detail


def test_allowed_hosts_must_cover_endpoint(tmp_path):
    bad = PROVIDER.replace("allowed_hosts: [10.203.247.121, 10.203.247.128]",
                           "allowed_hosts: [10.203.247.121]")
    with pytest.raises(RegistryError) as ei:
        sealed(tmp_path, provider=bad)
    assert "不在 allowed_hosts" in ei.value.detail


def test_capability_out_of_profile(tmp_path):
    bad = SCENE.replace("capabilities: [pick_object, stop]",
                        "capabilities: [pick_object, teleport]")
    with pytest.raises(RegistryError) as ei:
        sealed(tmp_path, scene=bad)
    assert "超出 profile 声明" in ei.value.detail


def test_duplicate_resource_id(tmp_path):
    bad = SCENE + "  - id: piper\n    profile: profiles/piper.yaml\n    capabilities: [stop]\n"
    with pytest.raises(RegistryError) as ei:
        sealed(tmp_path, scene=bad)
    assert "resource id 重复" in ei.value.detail


def test_provider_scene_mismatch(tmp_path):
    bad = PROVIDER.replace("scene_id: piper-pick", "scene_id: ur5-pick")
    with pytest.raises(RegistryError) as ei:
        sealed(tmp_path, provider=bad)
    assert "scene_id 不存在" in ei.value.detail


def test_schedule_enum(tmp_path):
    bad = PROVIDER.replace("schedule: static", "schedule: random")
    with pytest.raises(RegistryError) as ei:
        sealed(tmp_path, provider=bad)
    assert "schedule" in ei.value.detail


# ------------------------------------------------------------ resolve 错误 ---
def test_resolve_unknown_scene(tmp_path):
    reg = sealed(tmp_path)
    with pytest.raises(RegistryError) as ei:
        reg.resolve("ghost")
    assert ei.value.code == E_SCENE_NOT_FOUND


def test_resolve_without_scene(tmp_path):
    reg = sealed(tmp_path)
    with pytest.raises(RegistryError) as ei:
        reg.resolve(None)
    assert ei.value.code == E_SCENE_AMBIGUOUS


def test_resolve_unknown_resource(tmp_path):
    reg = sealed(tmp_path)
    with pytest.raises(RegistryError) as ei:
        reg.resolve("piper-pick", resource_id="ur5")
    assert ei.value.code == E_RESOURCE_NOT_FOUND


def test_resolve_capability_unsupported(tmp_path):
    reg = sealed(tmp_path)
    with pytest.raises(RegistryError) as ei:
        reg.resolve("piper-pick", capability="locomote")
    assert ei.value.code == E_CAPABILITY


# -------------------------------------------------------------- 调度策略 ---
def test_least_loaded_without_load_falls_back_to_priority(tmp_path):
    prov = PROVIDER.replace("schedule: static", "schedule: least-loaded")
    reg = sealed(tmp_path, provider=prov)
    assert reg.resolve("piper-pick").host.id == "x86-1"


def test_least_loaded_picks_lowest_load(tmp_path):
    prov = PROVIDER.replace("schedule: static", "schedule: least-loaded")
    prov = prov.replace("capacity: {cores: 4}", "capacity: {cores: 4, load: 0.9}")
    prov = prov.replace("capacity: {cores: 16}", "capacity: {cores: 16, load: 0.1}")
    reg = sealed(tmp_path, provider=prov)
    assert reg.resolve("piper-pick").host.id == "x86-2"


# ---------------------------------------------------------------- 热加载 ---
def test_reload_if_changed_noop_when_unchanged(tmp_path):
    reg = sealed(tmp_path)
    assert reg.reload_if_changed() is False


def test_reload_keeps_previous_on_invalid(tmp_path):
    reg = sealed(tmp_path)
    (tmp_path / "scenes" / "piper-pick.yaml").write_text(
        SCENE.replace("simulation: true\n", ""), encoding="utf-8")
    assert reg.reload_if_changed() is False          # 校验失败 → 不生效
    assert reg.scenes["piper-pick"].simulation is True   # 旧版本仍在
    assert reg.errors                                     # 错误可见


def test_seal_is_idempotent(tmp_path):
    write_tree(tmp_path)
    reg = SceneRegistry(root=tmp_path, profiles_root=tmp_path)
    assert reg.seal() == 2
    reg.seal()
    raw = (tmp_path / "scenes" / "piper-pick.yaml").read_text(encoding="utf-8")
    assert raw.count("digest: sha256:") == 1
