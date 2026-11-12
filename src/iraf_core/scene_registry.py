# -*- coding: utf-8 -*-
"""SceneRegistry / ProviderRegistry（阶段 A · A2）

单一职责：**场景/主机注册表的解析与校验**。MCP server、CLI 校验器、测试都只调用这里，
不允许在别处再写一套规则（否则必然漂移）。

对外 API
--------
    reg = SceneRegistry.load(root="/etc/iraf", profiles_root="/opt/iraf-sdk/0.4.1/profiles")
    reg.reload_if_changed()          # 热加载：mtime/digest 变化才重读；失败保留旧版本
    reg.resolve(scene_id=None, resource_id=None, default_scene=None)  -> Resolved
    reg.status()                     # 只读快照（给 iraf_scene_status）
    reg.seal()                       # 重新计算并写回 digest

错误：所有失败抛 RegistryError(code, detail)，code 见 CONTRACT 错误码表。
"""

from __future__ import annotations

import hashlib
import re
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional

try:
    import yaml
except Exception:  # pragma: no cover
    yaml = None

SCENE_SCHEMA = "iraf.scene/v1"
PROVIDER_SCHEMA = "iraf.provider/v1"
ID_RE = re.compile(r"^[a-z0-9][a-z0-9-]{1,48}$")
DIGEST_RE = re.compile(r"^digest:[ \t]*sha256:[0-9a-f]{64}[ \t]*\r?\n?", re.MULTILINE)

#: 错误码（与 plans/iraf-single-controlplane/01-contracts.md §6 对齐）
E_SCENE_NOT_FOUND = "IRAF-SCENE-NOT-FOUND"
E_SCENE_AMBIGUOUS = "IRAF-SCENE-AMBIGUOUS"
E_RESOURCE_NOT_FOUND = "IRAF-RESOURCE-NOT-FOUND"
E_CAPABILITY = "IRAF-CAPABILITY-UNSUPPORTED"
E_HOST_REJECTED = "IRAF-PROVIDER-HOST-REJECTED"
E_PROVIDER_UNAVAILABLE = "IRAF-PROVIDER-UNAVAILABLE"
E_RESOURCE_BUSY = "IRAF-RESOURCE-BUSY"
E_REGISTRY_INVALID = "IRAF-REGISTRY-INVALID"


class RegistryError(Exception):
    """注册表/解析错误；code 走契约错误码。"""

    def __init__(self, code: str, detail: str):
        super().__init__("%s: %s" % (code, detail))
        self.code = code
        self.detail = detail


# ------------------------------------------------------------------ model ---
@dataclass
class Resource:
    id: str
    profile: str
    capabilities: List[str] = field(default_factory=list)


@dataclass
class Scene:
    scene_id: str
    provider_id: str
    profile: str
    safety_policy: str
    simulation: bool
    max_concurrency: int
    lease_ttl_s: int
    switch_name: str = ""   # Provider 侧场景名（switch_scene.sh / supervisor 用）
    resources: List[Resource] = field(default_factory=list)
    scenario: Dict[str, Any] = field(default_factory=dict)
    display: Dict[str, Any] = field(default_factory=dict)
    raw: Dict[str, Any] = field(default_factory=dict)


@dataclass
class Host:
    id: str
    endpoint: str
    priority: int
    capacity: Dict[str, Any] = field(default_factory=dict)
    control_url: str = ""       # supervisor 控制面（阶段 C4；空则用 IRAF_SUPERVISOR_URL）

    @property
    def host(self) -> str:
        return self.endpoint.rpartition(":")[0]

    @property
    def port(self) -> str:
        return self.endpoint.rpartition(":")[2]


@dataclass
class Provider:
    provider_id: str
    scene_id: str
    hosts: List[Host] = field(default_factory=list)
    schedule: str = "static"
    allowed_hosts: List[str] = field(default_factory=list)
    health: Dict[str, Any] = field(default_factory=dict)


@dataclass
class Resolved:
    scene: Scene
    provider: Provider
    host: Host
    resource: Resource

    def as_dict(self) -> Dict[str, Any]:
        return {
            "scene_id": self.scene.scene_id,
            "provider_id": self.provider.provider_id,
            "host_id": self.host.id,
            "endpoint": self.host.endpoint,
            "resource_id": self.resource.id,
            "simulation": self.scene.simulation,
        }


# ------------------------------------------------------------------ digest ---
def strip_digest(raw: bytes) -> bytes:
    return DIGEST_RE.sub("", raw.decode("utf-8"), count=1).encode("utf-8")


def compute_digest(raw: bytes) -> str:
    return "sha256:" + hashlib.sha256(strip_digest(raw)).hexdigest()


# ---------------------------------------------------------------- registry ---
class SceneRegistry:
    def __init__(self, root: Path, profiles_root: Optional[Path] = None):
        self.root = Path(root)
        self.profiles_root = Path(profiles_root) if profiles_root else None
        self.scenes: Dict[str, Scene] = {}
        self.providers: Dict[str, Provider] = {}
        self.errors: List[str] = []
        self.warnings: List[str] = []
        self._fingerprint: str = ""
        self._profile_caps: Dict[str, set] = {}

    # -- load -----------------------------------------------------------
    @classmethod
    def load(cls, root="/etc/iraf", profiles_root=None) -> "SceneRegistry":
        reg = cls(root, profiles_root)
        reg.reload(force=True)
        return reg

    def _scan(self) -> str:
        parts = []
        for sub in ("scenes", "providers"):
            d = self.root / sub
            if d.is_dir():
                for p in sorted(d.glob("*.yaml")):
                    st = p.stat()
                    parts.append("%s:%d:%d" % (p.name, st.st_mtime_ns, st.st_size))
        return "|".join(parts)

    def reload(self, force: bool = False) -> bool:
        """重新加载；返回是否真的重载了。校验失败时保留旧数据并追加错误。"""
        fp = self._scan()
        if not force and fp == self._fingerprint:
            return False
        scenes: Dict[str, Scene] = {}
        providers: Dict[str, Provider] = {}
        errors: List[str] = []
        warnings: List[str] = []

        scenes_dir = self.root / "scenes"
        providers_dir = self.root / "providers"
        if not scenes_dir.is_dir():
            raise RegistryError(E_REGISTRY_INVALID, "缺少目录：%s" % scenes_dir)
        if not providers_dir.is_dir():
            raise RegistryError(E_REGISTRY_INVALID, "缺少目录：%s" % providers_dir)

        for path in sorted(scenes_dir.glob("*.yaml")):
            self._load_scene(path, scenes, errors, warnings)
        for path in sorted(providers_dir.glob("*.yaml")):
            self._load_provider(path, scenes, providers, errors, warnings)

        for sid, sc in scenes.items():
            if sc.provider_id not in providers:
                errors.append("scene %s: provider_id 未定义：%r" % (sid, sc.provider_id))

        if errors:
            # 保留上一次成功的快照，不把半成品带进运行期
            if self.scenes and self.providers:
                self.warnings = warnings + ["注册表校验失败，保留上一版本 (%d 项错误)" % len(errors)]
                self.errors = errors
                return False
            self.scenes, self.providers = scenes, providers
            self.errors, self.warnings = errors, warnings
            raise RegistryError(E_REGISTRY_INVALID, "；".join(errors[:5]))

        self.scenes, self.providers = scenes, providers
        self.errors, self.warnings = [], warnings
        self._fingerprint = fp
        return True

    def reload_if_changed(self) -> bool:
        return self.reload(force=False)

    # -- parse/validate --------------------------------------------------
    def _load_yaml(self, path: Path, errors: List[str]) -> Optional[dict]:
        if yaml is None:
            raise RegistryError(E_REGISTRY_INVALID, "PyYAML 不可用")
        raw = path.read_bytes()
        try:
            data = yaml.safe_load(raw.decode("utf-8")) or {}
        except Exception as exc:
            errors.append("%s: YAML 解析失败：%s" % (path.name, exc))
            return None
        declared = str((data or {}).get("digest") or "")
        actual = compute_digest(raw)
        if declared != actual:
            errors.append("%s: digest 不匹配（期望 %s，实际 %r）；跑 seal 重新生成"
                          % (path.name, actual, declared))
        return data

    def _caps_for(self, profile_rel: str, errors: List[str], where: str) -> set:
        if profile_rel in self._profile_caps:
            return self._profile_caps[profile_rel]
        caps: set = set()
        if self.profiles_root:
            p = Path(profile_rel)
            cands = [p] if p.is_absolute() else [
                self.profiles_root / p,
                self.profiles_root / p.name,
                self.profiles_root / "profiles" / p.name,
            ]
            for cand in cands:
                if cand.is_file():
                    data = yaml.safe_load(cand.read_text(encoding="utf-8")) or {}
                    caps = collect_capabilities(data)
                    break
            if not caps:
                self.warnings.append("%s: profile 未读到 capabilities（%s），跳过子集校验"
                                     % (where, profile_rel))
        self._profile_caps[profile_rel] = caps
        return caps

    def _load_scene(self, path, scenes, errors, warnings) -> None:
        data = self._load_yaml(path, errors)
        if not isinstance(data, dict):
            return
        if data.get("schema") != SCENE_SCHEMA:
            errors.append("%s: schema 必须是 %s" % (path.name, SCENE_SCHEMA))
            return
        sid = str(data.get("scene_id") or "")
        if not ID_RE.match(sid):
            errors.append("%s: scene_id 非法或缺失：%r" % (path.name, sid))
            return
        if path.stem != sid:
            errors.append("%s: 文件名必须等于 scene_id（当前 %r）" % (path.name, sid))
        if data.get("simulation") is not True:
            errors.append("%s: 必须显式声明 simulation: true" % path.name)
        mc = data.get("max_concurrency")
        if not isinstance(mc, int) or mc < 1:
            errors.append("%s: max_concurrency 必须是 ≥1 的整数" % path.name)
            mc = 1
        ttl = data.get("lease_ttl_s", 60)
        if not isinstance(ttl, int) or ttl <= 0:
            errors.append("%s: lease_ttl_s 必须是正整数" % path.name)
            ttl = 60
        if not data.get("safety_policy"):
            errors.append("%s: 缺 safety_policy" % path.name)
        resources: List[Resource] = []
        seen = set()
        for res in data.get("resources") or []:
            rid = str((res or {}).get("id") or "")
            prof = str((res or {}).get("profile") or data.get("profile") or "")
            caps = [str(c) for c in ((res or {}).get("capabilities") or [])]
            if not ID_RE.match(rid):
                errors.append("%s: resource id 非法：%r" % (path.name, rid))
                continue
            if rid in seen:
                errors.append("%s: resource id 重复：%s" % (path.name, rid))
                continue
            seen.add(rid)
            if not caps:
                errors.append("%s[%s]: capabilities 必须是非空列表" % (path.name, rid))
            elif prof:
                declared = self._caps_for(prof, errors, "%s[%s]" % (path.name, rid))
                if declared:
                    extra = [c for c in caps if c not in declared]
                    if extra:
                        errors.append("%s[%s]: capabilities 超出 profile 声明：%s"
                                      % (path.name, rid, extra))
            resources.append(Resource(id=rid, profile=prof, capabilities=caps))
        if not resources:
            errors.append("%s: resources 必须是非空列表" % path.name)
        scenes[sid] = Scene(
            scene_id=sid,
            provider_id=str(data.get("provider_id") or ""),
            profile=str(data.get("profile") or ""),
            safety_policy=str(data.get("safety_policy") or ""),
            simulation=data.get("simulation") is True,
            max_concurrency=mc,
            lease_ttl_s=ttl,
            switch_name=str(data.get("switch_name") or ""),
            resources=resources,
            scenario=data.get("scenario") or {},
            display=data.get("display") or {},
            raw=data,
        )

    def _load_provider(self, path, scenes, providers, errors, warnings) -> None:
        data = self._load_yaml(path, errors)
        if not isinstance(data, dict):
            return
        if data.get("schema") != PROVIDER_SCHEMA:
            errors.append("%s: schema 必须是 %s" % (path.name, PROVIDER_SCHEMA))
            return
        pid = str(data.get("provider_id") or "")
        if not pid.startswith("iraf/") or not ID_RE.match(pid.split("/", 1)[1]):
            errors.append("%s: provider_id 必须形如 iraf/<slug>：%r" % (path.name, pid))
            return
        expect = pid.replace("/", "-") + ".yaml"
        if path.name != expect:
            errors.append("%s: 文件名必须等于 provider_id 的 /→- 形式（期望 %s）"
                          % (path.name, expect))
        scene_id = str(data.get("scene_id") or "")
        if scene_id not in scenes:
            errors.append("%s: scene_id 不存在于 scenes/：%r" % (path.name, scene_id))
        elif scenes[scene_id].provider_id != pid:
            errors.append("%s: 与 scene 的 provider_id 不一致（scene 写的是 %r）"
                          % (path.name, scenes[scene_id].provider_id))
        allowed = [str(h) for h in (data.get("allowed_hosts") or [])]
        if not allowed:
            errors.append("%s: allowed_hosts 不能为空" % path.name)
        hosts: List[Host] = []
        seen_id, seen_prio = set(), set()
        for item in data.get("hosts") or []:
            hid = str((item or {}).get("id") or "")
            endpoint = str((item or {}).get("endpoint") or "")
            prio = (item or {}).get("priority")
            if not hid:
                errors.append("%s: host 缺 id" % path.name)
                continue
            if hid in seen_id:
                errors.append("%s: host id 重复：%s" % (path.name, hid))
                continue
            seen_id.add(hid)
            if not isinstance(prio, int):
                errors.append("%s[%s]: priority 必须是整数" % (path.name, hid))
                prio = 100
            elif prio in seen_prio:
                errors.append("%s: priority 重复：%s" % (path.name, prio))
            else:
                seen_prio.add(prio)
            host_part, sep, port = endpoint.rpartition(":")
            if not sep or not port.isdigit():
                errors.append("%s[%s]: endpoint 必须形如 host:port：%r"
                              % (path.name, hid, endpoint))
            elif host_part not in allowed:
                errors.append("%s[%s]: endpoint 主机 %r 不在 allowed_hosts %s"
                              % (path.name, hid, host_part, allowed))
            control_url = str((item or {}).get("control_url") or "")
            if control_url and not control_url.startswith(("http://", "https://")):
                errors.append("%s[%s]: control_url 必须是 http(s)://：%r"
                              % (path.name, hid, control_url))
            hosts.append(Host(id=hid, endpoint=endpoint, priority=int(prio),
                              capacity=(item or {}).get("capacity") or {},
                              control_url=control_url))
        if not hosts:
            errors.append("%s: hosts 必须是非空列表" % path.name)
        sched = data.get("schedule", "static")
        if sched not in ("static", "least-loaded"):
            errors.append("%s: schedule 只能是 static / least-loaded（当前 %r）"
                          % (path.name, sched))
            sched = "static"
        providers[pid] = Provider(provider_id=pid, scene_id=scene_id, hosts=hosts,
                                  schedule=sched, allowed_hosts=allowed,
                                  health=data.get("health") or {})

    # -- resolve ---------------------------------------------------------
    def infer(self, text: Optional[str]) -> str:
        """按自然语言里的机型/场景词推断 scene_id；推不出或有歧义时返回空串。

        数据来源（不写死机型）：scene_id、`raw.aliases`、`resources[].profile` 的机型词。
        歧义规则：命中多个场景时，只接受 scene_id 以命中词开头的那一个
        （例如 piper → piper-pick）；仍歧义则返回空串，调用方继续走 default_scene 或显式报错。
        这样"piper 抓取"不会再落到联合世界（joint-handoff 里也有 piper）。
        """
        try:
            self.reload_if_changed()
        except Exception:
            return ""
        low = (text or "").strip().lower()
        if not low:
            return ""
        stopwords = {"mujoco", "profile", "profiles", "yaml", "sim", "pick"}
        scored = {}
        for sid, scene in self.scenes.items():
            tokens = {sid.lower()}
            tokens.update(str(a).lower() for a in (scene.raw.get("aliases") or []))
            tokens.update(part.lower() for part in sid.split("-") if part)
            for res in scene.resources:
                stem = (res.profile or "").rsplit("/", 1)[-1].rsplit(".", 1)[0]
                tokens.update(p.lower() for p in stem.split("_") if p)
            hits = sorted((t for t in tokens if t and t not in stopwords and t in low),
                          key=len, reverse=True)
            if hits:
                scored[sid] = hits[0]
        if not scored:
            return ""
        if len(scored) == 1:
            return next(iter(scored))
        # 命中多个场景时先看“命中词有多具体”：ur5e 应压过 joint 里 ur5 的泛命中。
        longest = max(len(token) for token in scored.values())
        best = {sid: token for sid, token in scored.items() if len(token) == longest}
        if len(best) == 1:
            return next(iter(best))
        pref = [sid for sid, token in best.items() if sid.lower().startswith(token)]
        return pref[0] if len(pref) == 1 else ""

    def resolve(self, scene_id: Optional[str] = None,
                resource_id: Optional[str] = None,
                default_scene: Optional[str] = None,
                capability: Optional[str] = None) -> Resolved:
        sid = (scene_id or "").strip() or (default_scene or "").strip()
        if not sid:
            raise RegistryError(E_SCENE_AMBIGUOUS,
                                "未指定场景；可用：%s" % ", ".join(sorted(self.scenes)))
        scene = self.scenes.get(sid)
        if scene is None:
            raise RegistryError(E_SCENE_NOT_FOUND,
                                "场景 %r 不存在；可用：%s" % (sid, ", ".join(sorted(self.scenes))))
        provider = self.providers.get(scene.provider_id)
        if provider is None:
            raise RegistryError(E_REGISTRY_INVALID,
                                "scene %s 的 provider 未加载：%s" % (sid, scene.provider_id))
        if resource_id:
            res = next((r for r in scene.resources if r.id == resource_id), None)
            if res is None:
                raise RegistryError(E_RESOURCE_NOT_FOUND,
                                    "场景 %s 没有 resource %r；可用：%s"
                                    % (sid, resource_id, ", ".join(r.id for r in scene.resources)))
        else:
            res = scene.resources[0]
        if capability and capability not in res.capabilities:
            raise RegistryError(E_CAPABILITY,
                                "resource %s 不支持 %s（支持：%s）"
                                % (res.id, capability, ", ".join(res.capabilities)))
        return Resolved(scene=scene, provider=provider,
                        host=select_host(provider), resource=res)

    # -- status ----------------------------------------------------------
    def status(self) -> Dict[str, Any]:
        return {
            "registry": {"root": str(self.root), "fingerprint": self._fingerprint,
                         "errors": self.errors, "warnings": self.warnings},
            "scenes": [
                {"scene_id": s.scene_id, "provider_id": s.provider_id, "profile": s.profile,
                 "simulation": s.simulation, "max_concurrency": s.max_concurrency,
                 "lease_ttl_s": s.lease_ttl_s, "switch_name": s.switch_name,
                 "resources": [{"id": r.id, "capabilities": r.capabilities} for r in s.resources],
                 "scenario": s.scenario, "display": s.display}
                for s in sorted(self.scenes.values(), key=lambda x: x.scene_id)
            ],
            "providers": [
                {"provider_id": p.provider_id, "scene_id": p.scene_id, "schedule": p.schedule,
                 "allowed_hosts": p.allowed_hosts,
                 "hosts": [{"id": h.id, "endpoint": h.endpoint, "priority": h.priority,
                            "capacity": h.capacity, "control_url": h.control_url}
                           for h in p.hosts]}
                for p in sorted(self.providers.values(), key=lambda x: x.provider_id)
            ],
        }

    # -- seal ------------------------------------------------------------
    def seal(self) -> int:
        n = 0
        for sub in ("scenes", "providers"):
            for path in sorted((self.root / sub).glob("*.yaml")):
                raw = path.read_bytes()
                digest = compute_digest(raw)
                body = DIGEST_RE.sub("", raw.decode("utf-8"), count=1)
                if not body.endswith("\n"):
                    body += "\n"
                path.write_text(body + "digest: %s\n" % digest, encoding="utf-8")
                n += 1
        self._fingerprint = ""
        return n


# ------------------------------------------------------------------ helpers ---
def collect_capabilities(node, acc=None) -> set:
    acc = acc if acc is not None else set()
    if isinstance(node, dict):
        for key, value in node.items():
            if key == "capabilities" and isinstance(value, list):
                acc.update(str(v) for v in value)
            else:
                collect_capabilities(value, acc)
    elif isinstance(node, list):
        for item in node:
            collect_capabilities(item, acc)
    return acc


def select_host(provider: Provider) -> Host:
    """static：priority 最小；least-loaded：capacity.load 最小（缺字段降级 static）。"""
    if provider.schedule == "least-loaded":
        def load_of(host: Host) -> float:
            try:
                return float(host.capacity.get("load"))
            except Exception:
                return float("inf")
        hosts = sorted(provider.hosts, key=lambda h: (load_of(h), h.priority))
        if load_of(hosts[0]) != float("inf"):
            return hosts[0]
    return sorted(provider.hosts, key=lambda h: h.priority)[0]
