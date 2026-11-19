# -*- coding: utf-8 -*-
"""新机器人接入生命周期：场景生成、能力脚手架、单机验收、注册、状态。

设计边界（AGENTS §12）：
- 验收通过前不得注册为可用；`runnable` 只由验收证据决定；
- 不自动修改已存在 Backend/Profile/Scene（只产出 patch/草案或拒绝覆盖）；
- 所有失败显式，不伪造证据。
"""
from __future__ import annotations

import json
from pathlib import Path
from types import SimpleNamespace
from typing import Any, Dict, Optional

import yaml


class LifecycleError(ValueError):
    pass


class _Authority:
    def validate(self, lease):  # noqa: D401
        return None


def _root(root) -> Path:
    return Path(root or ".").resolve()


def _load_yaml(path: Path) -> dict:
    if not path.is_file():
        raise LifecycleError("文件不存在：%s" % path)
    data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    if not isinstance(data, dict):
        raise LifecycleError("YAML 顶层必须是 object：%s" % path)
    return data


def _manipulation_from_report(report: dict, target_id: str = "") -> Dict[str, Any]:
    targets = report.get("targets") or []
    target_id = str(target_id or report.get("target_id") or (targets[0].get("id") if targets else ""))
    entry = next((x for x in targets if str(x.get("id")) == target_id), None)
    if entry is None:
        raise LifecycleError("报告里没有目标：%s" % target_id)
    gripper = dict(report.get("gripper") or {})
    if not gripper:
        raise LifecycleError("报告缺少 gripper 段（不能生成 pick 配置）")
    gripper.setdefault("max_tilt_deg", 30.0)
    return {"targets": {target_id: {"body": str(entry.get("body", target_id)),
                                    "pose_tolerance_m": float(report.get("pose_tolerance_m", 0.005))}},
            "gripper": gripper}


def build_scene(robot_id: str, scene_id: str, profile: str, provider_endpoint: str,
                capabilities, *, root=".", safety_policy="profiles/safety/simulation_lab.yaml",
                model_path: str = "", report_path: str = "", backend_entrypoint: str = "",
                backend_config: Optional[dict] = None, control_url: str = "",
                allowed_hosts=None, dry_run: bool = True, allow_overwrite: bool = False,
                target_id: str = "") -> Dict[str, Any]:
    """生成 scene/provider 文档；有 report 时自动生成 backend_config。"""
    root = _root(root)
    profile_path = root / profile
    if not profile_path.is_file():
        raise LifecycleError("Profile 不存在：%s" % profile_path)
    profile_doc = _load_yaml(profile_path)
    declared = set((profile_doc.get("spec") or {}).get("capabilities") or [])
    caps = [str(c) for c in (capabilities or [])]
    missing = [c for c in caps if c not in declared]
    if missing:
        raise LifecycleError("Profile 未声明能力 %s（先改 Profile，禁止悬空能力）" % missing)

    cfg = dict(backend_config or {})
    if report_path:
        report = json.loads((root / report_path).read_text(encoding="utf-8"))
        cfg.setdefault("model_path", str(report.get("output") or model_path))
        try:
            cfg.setdefault("manipulation", _manipulation_from_report(report, target_id))
        except LifecycleError:
            if "pick_object" in caps or "place_object" in caps:
                raise
    if model_path:
        cfg.setdefault("model_path", str((root / model_path).resolve() if not Path(model_path).is_absolute() else model_path))
    if backend_entrypoint:
        cfg["__entrypoint__"] = backend_entrypoint
    entrypoint = str(cfg.pop("__entrypoint__", "") or "")
    if entrypoint and not cfg.get("model_path") and not cfg.get("report"):
        raise LifecycleError("backend_entrypoint 需要 model_path 或 report")

    scene = {
        "schema": "iraf.scene/v1", "scene_id": scene_id,
        "provider_id": "iraf/%s" % scene_id, "profile": profile,
        "safety_policy": safety_policy, "simulation": True,
        "switch_name": robot_id, "aliases": [robot_id, scene_id],
        "max_concurrency": 1, "lease_ttl_s": 120,
        "description": "%s 单机 scene（robot_lifecycle 生成）" % robot_id,
        "resources": [{"id": robot_id, "profile": profile, "capabilities": caps}],
    }
    if entrypoint:
        scene["backend_entrypoint"] = entrypoint
        scene["backend_config"] = cfg
    provider = {
        "schema": "iraf.provider/v1", "provider_id": "iraf/%s" % scene_id,
        "scene_id": scene_id, "token_env": "IRAF_PROVIDER_TOKEN",
        "hosts": [{"id": "x86-1", "endpoint": provider_endpoint,
                   "capacity": {"cores": 4, "gpu": "none"}, "priority": 10}],
        "schedule": "static", "health": {"interval_s": 5, "timeout_s": 2, "failover": "none"},
        "allowed_hosts": list(allowed_hosts or [provider_endpoint.rsplit(":", 1)[0]]),
    }
    if control_url:
        provider["hosts"][0]["control_url"] = control_url
    scene_path = root / "config/scenes" / ("%s.yaml" % scene_id)
    provider_path = root / "config/providers" / ("iraf-%s.yaml" % scene_id)
    artifacts = []
    if not dry_run:
        for path in (scene_path, provider_path):
            if path.exists() and not allow_overwrite:
                raise LifecycleError("拒绝覆盖已存在文件：%s（allow_overwrite=false）" % path)
            path.parent.mkdir(parents=True, exist_ok=True)
            doc = scene if path == scene_path else provider
            path.write_text(yaml.safe_dump(doc, allow_unicode=True, sort_keys=False), encoding="utf-8")
            artifacts.append(str(path))
    return {"status": "SUCCEEDED", "robot_id": robot_id, "scene_id": scene_id,
            "scene": scene, "provider": provider, "artifacts": artifacts,
            "dry_run": bool(dry_run), "note": "未验收前不得 register"}


_SCAFFOLDS = {
    "move_joint": {
        "method": '''    def move_joint(self, positions, duration_ms, lease):
        self.authority.validate(lease)
        # TODO: 把 positions 写进执行器/控制器；必须做限位与契约校验
        raise NotImplementedError("实现 move_joint 后删除本行")
''',
        "requires": "move_joint",
    },
    "gripper": {
        "method": '''    def gripper(self, action, lease, duration_ms=1000):
        self.authority.validate(lease)
        # TODO: 映射 open/close 到夹爪 actuator；必须显式声明 open/close 取值
        raise NotImplementedError("实现 gripper 后删除本行")
''',
        "requires": "gripper",
    },
    "pick_object": {
        "method": '''    def pick_object(self, target_id, grasp_pose, duration_ms, lease):
        self.authority.validate(lease)
        # TODO: 目标/夹爪几何/参考解必须来自场景报告，禁止硬编码
        raise NotImplementedError("实现 pick_object 后删除本行")
''',
        "requires": "pick_object",
    },
    "place_object": {
        "method": '''    def place_object(self, place_target_id, payload_id, duration_ms, lease):
        self.authority.validate(lease)
        # TODO: 放置目标/承载面必须来自场景报告
        raise NotImplementedError("实现 place_object 后删除本行")
''',
        "requires": "place_object",
    },
    "stand": {
        "method": '''    def stand(self, lease, targets=None, duration_ms=None):
        self.authority.validate(lease)
        # TODO: 站立目标只来自 Profile.spec.home 或声明参数
        raise NotImplementedError("实现 stand 后删除本行")
''',
        "requires": "stand",
    },
    "locomote": {
        "method": '''    def locomote(self, velocity, duration_ms, lease):
        self.authority.validate(lease)
        # TODO: 速度单位/限幅来自 Profile/声明，不得硬编码
        raise NotImplementedError("实现 locomote 后删除本行")
''',
        "requires": "locomote",
    },
}


def scaffold_capability(robot_id: str, capability: str, *, root=".", dry_run=True,
                        output_dir: str = "") -> Dict[str, Any]:
    """生成能力实现/skill/声明的草案；不自动改现有源码。"""
    root = _root(root)
    cap = str(capability or "").strip()
    if cap not in _SCAFFOLDS:
        raise LifecycleError("暂不支持的能力模板：%s（可用：%s）" % (cap, sorted(_SCAFFOLDS)))
    out = root / (output_dir or ("build/onboarding/%s/scaffold/%s" % (robot_id, cap)))
    files = {
        "backend_method.py": _SCAFFOLDS[cap]["method"],
        "provider.py": '"""%s Provider 草案：把能力调用转给 backend.%s。"""\n\n'
                       'class %sProvider:\n'
                       '    def __init__(self, profile, backend):\n'
                       '        self.profile = profile\n'
                       '        self.backend = backend\n\n'
                       '    def execute(self, inputs, lease=None):\n'
                       '        raise NotImplementedError("按 inputs 调用 backend.%s")\n'
                       % (cap, cap, "".join(p.capitalize() for p in cap.split("_")), cap),
        "skill.yaml": yaml.safe_dump({
            "apiVersion": "iraf.intewell.io/v1", "kind": "Skill",
            "metadata": {"name": cap, "version": "0.1.0"},
            "spec": {"verification": "development",
                     "inputSchema": "%s.input.json" % cap,
                     "outputSchema": "result.output.json",
                     "requires": [_SCAFFOLDS[cap]["requires"]],
                     "preconditions": [], "intentExamples": [],
                     "timeoutSeconds": 30, "cancellation": "not_applicable",
                     "recovery": ["request_handover"], "safetyClass": "motion",
                     "providers": [{"name": "%s_provider" % cap, "type": "python_adapter",
                                    "entrypoint": "TODO:%sProvider" % cap,
                                    "selector": {"mode": "simulation"}, "priority": 0}]}},
            allow_unicode=True, sort_keys=False),
        "profile_patch.yaml": yaml.safe_dump({"spec": {"capabilities": [cap]}}, allow_unicode=True, sort_keys=False),
        "scene_patch.yaml": yaml.safe_dump({"resources": [{"capabilities": [cap]}]}, allow_unicode=True, sort_keys=False),
        "policy_patch.yaml": yaml.safe_dump({"spec": {"allowed_skills": [cap]}}, allow_unicode=True, sort_keys=False),
        "README.md": "把 backend_method.py 合入 Backend，实现 provider.py，"
                     "再按 patch 改 Profile/Scene/Policy，最后跑 iraf_robot_acceptance；未通过不得注册。\n",
    }
    if not dry_run:
        out.mkdir(parents=True, exist_ok=True)
        for name, content in files.items():
            path = out / name
            if path.exists():
                raise LifecycleError("拒绝覆盖已存在文件：%s" % path)
            path.write_text(content, encoding="utf-8")
    return {"status": "SUCCEEDED", "robot_id": robot_id, "capability": cap,
            "files": files, "output_dir": str(out), "dry_run": bool(dry_run)}


def acceptance(robot_id: str, capability: str, *, profile: str, root=".", scene_id: str = "",
               backend_entrypoint: str = "", backend_config=None, params=None,
               dry_run=True, output_dir: str = "") -> Dict[str, Any]:
    """在本地 Backend 上执行一次能力验收；不通过不写 passed。"""
    root = _root(root)
    prof_path = root / profile
    profile_doc = _load_yaml(prof_path)
    spec = profile_doc.get("spec") or {}
    caps = list(spec.get("capabilities") or [])
    cap = str(capability or "")
    if cap not in caps:
        raise LifecycleError("Profile 未声明能力 %s" % cap)
    if spec.get("simulation") is not True:
        raise LifecycleError("只允许 simulation=true 的 Profile 做仿真验收")
    if dry_run:
        return {"status": "SUCCEEDED", "robot_id": robot_id, "capability": cap,
                "passed": False, "dry_run": True, "note": "未执行；去 dry_run 后才产生证据"}
    from iraf_core.profile import load_robot_profile
    from iraf_adapters.factory import load_backend
    profile_obj = load_robot_profile(str(prof_path))
    cfg = dict(backend_config or {})
    entry = backend_entrypoint or "iraf_adapters.mujoco.mujoco_backend:MujocoBackend"
    backend = load_backend(entry, cfg, profile_obj, _Authority())
    lease = SimpleNamespace(fencing_token=1, resource=robot_id, lease_id="acceptance")
    p = dict(params or {})
    evidence = None
    if cap == "move_joint":
        backend.move_joint(dict(p.get("positions") or {}), int(p.get("duration_ms", 1000)), lease)
        evidence = {"positions": p.get("positions"), "duration_ms": int(p.get("duration_ms", 1000))}
    elif cap == "gripper":
        evidence = backend.gripper(str(p.get("action") or "close"), lease, int(p.get("duration_ms", 1000)))
    elif cap == "pick_object":
        evidence = backend.pick_object(str(p.get("target_id") or "box_01"),
                                       dict(p.get("grasp_pose") or {"frame_id": "world", "pose_source": "live_target_body"}),
                                       int(p.get("duration_ms", 30000)), lease)
    elif cap == "place_object":
        evidence = backend.place_object(str(p.get("place_target_id") or "tray_01"),
                                        str(p.get("payload_id") or "box_01"),
                                        int(p.get("duration_ms", 30000)), lease)
    elif cap == "stand":
        evidence = backend.stand(lease, targets=dict(p.get("targets") or {}),
                                 duration_ms=p.get("duration_ms"))
    elif cap == "locomote":
        evidence = backend.locomote(dict(p.get("velocity") or {}),
                                    int(p.get("duration_ms", 2000)), lease)
    else:
        raise LifecycleError("验收模板暂不支持：%s" % cap)
    report = {"schema": "iraf.robot-acceptance/v1", "robot_id": robot_id,
              "scene_id": scene_id, "capability": cap, "passed": True,
              "simulation": True, "evidence": evidence,
              "profile": {"name": profile_doc.get("metadata", {}).get("name"),
                          "version": profile_doc.get("metadata", {}).get("version")}}
    out = root / (output_dir or ("build/acceptance/%s/%s" % (robot_id, cap)))
    out.mkdir(parents=True, exist_ok=True)
    report_path = out / "report.json"
    report_path.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    report["report_path"] = str(report_path)
    return report


def register(scene_file: str, provider_file: str, *, root=".", registry_root="/etc/iraf",
             profiles_root="", acceptance_report: str = "", dry_run=True,
             allow_overwrite: bool = False) -> Dict[str, Any]:
    """验收 passed=true 后，把 scene/provider 装入注册表并 seal。"""
    root = _root(root)
    if acceptance_report:
        rep = json.loads(Path(acceptance_report).read_text(encoding="utf-8"))
        if rep.get("passed") is not True:
            raise LifecycleError("验收未通过，拒绝注册：%s" % acceptance_report)
    else:
        raise LifecycleError("注册必须提供 acceptance_report（passed=true）")
    scene_path = root / scene_file
    provider_path = root / provider_file
    _load_yaml(scene_path); _load_yaml(provider_path)
    reg = Path(registry_root)
    artifacts = []
    if not dry_run:
        (reg / "scenes").mkdir(parents=True, exist_ok=True)
        (reg / "providers").mkdir(parents=True, exist_ok=True)
        for src in (scene_path, provider_path):
            dst = reg / ("scenes" if src.parent.name == "scenes" else "providers") / src.name
            if dst.exists() and not allow_overwrite:
                raise LifecycleError("注册表已存在：%s（拒绝覆盖；allow_overwrite=true 才覆盖）" % dst)
            dst.write_text(src.read_text(encoding="utf-8"), encoding="utf-8")
            artifacts.append(str(dst))
    from iraf_core.scene_registry import SceneRegistry
    r = SceneRegistry(root=str(reg), profiles_root=str(root / profiles_root) if profiles_root else None)
    if not dry_run:
        r.seal()
        r.reload_if_changed()
        if r.errors:
            raise LifecycleError("注册表校验失败：%s" % r.errors)
    return {"status": "SUCCEEDED", "dry_run": bool(dry_run), "artifacts": artifacts,
            "scenes": sorted(r.scenes), "errors": r.errors}


def status(robot_id: str, *, root=".", registry_root="/etc/iraf", profiles_root="") -> Dict[str, Any]:
    root = _root(root)
    from iraf_core.scene_registry import SceneRegistry
    r = SceneRegistry(root=registry_root, profiles_root=str(root / profiles_root) if profiles_root else None)
    r.reload_if_changed()
    matches = []
    for sid, scene in r.scenes.items():
        profiles = [x.profile for x in scene.resources]
        names = [scene.scene_id, scene.switch_name] + list(scene.raw.get("aliases") or []) + profiles
        if any(robot_id == n or robot_id in str(n) for n in names):
            matches.append({"scene_id": sid, "profile": scene.profile, "resources": profiles})
    reports = sorted((root / "build/acceptance" / robot_id).glob("*/report.json")) if (root / "build/acceptance" / robot_id).is_dir() else []
    acc = []
    for p in reports:
        try:
            d = json.loads(p.read_text(encoding="utf-8"))
            acc.append({"capability": d.get("capability"), "passed": d.get("passed"), "path": str(p)})
        except Exception:
            acc.append({"path": str(p), "passed": False})
    return {"status": "SUCCEEDED", "robot_id": robot_id, "scenes": matches,
            "acceptance": acc, "registry_errors": r.errors}
