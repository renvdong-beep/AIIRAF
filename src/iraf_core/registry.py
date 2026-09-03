"""Skill manifest loading, schema validation and deterministic Provider routing."""
from dataclasses import dataclass
import hashlib, importlib, json, re
from pathlib import Path
import yaml
from jsonschema import Draft202012Validator

class RegistryError(ValueError):
    pass

_SEMVER = re.compile(r"^(0|[1-9]\d*)\.(0|[1-9]\d*)\.(0|[1-9]\d*)(?:-[0-9A-Za-z.-]+)?(?:\+[0-9A-Za-z.-]+)?$")

def _semver_key(value):
    text = str(value)
    match = _SEMVER.match(text)
    if not match:
        raise RegistryError("Skill version must be SemVer: " + text)
    major, minor, patch = (int(item) for item in match.groups()[:3])
    prerelease = text.split("-", 1)[1].split("+", 1)[0] if "-" in text else ""
    # Stable releases sort after prereleases for the same numeric version.
    return major, minor, patch, (1 if not prerelease else 0), prerelease



def _constraint_matches(version, constraint):
    """Support exact, comparator, caret and tilde ranges used by Skill clients."""
    text = str(constraint).strip()
    if not text:
        return True
    version_key = _semver_key(version)
    for item in (part.strip() for part in text.split(",")):
        if not item:
            continue
        if item.startswith("^") or item.startswith("~"):
            operator, base_text = item[0], item[1:].strip()
            base = _semver_key(base_text)
            major, minor, patch = base[:3]
            if operator == "^":
                upper = (major + 1, 0, 0, 1, "") if major else ((0, minor + 1, 0, 1, "") if minor else (0, 0, patch + 1, 1, ""))
            else:
                upper = (major, minor + 1, 0, 1, "")
            if not (version_key >= base and version_key < upper):
                return False
            continue
        match = re.match(r"^(<=|>=|<|>|=)?\s*(.+)$", item)
        if not match:
            raise RegistryError("invalid Skill version constraint: " + item)
        operator, target_text = match.group(1) or "=", match.group(2)
        target = _semver_key(target_text)
        if operator == "=" and str(version) != target_text:
            return False
        if operator == "<" and not version_key < target:
            return False
        if operator == "<=" and not version_key <= target:
            return False
        if operator == ">" and not version_key > target:
            return False
        if operator == ">=" and not version_key >= target:
            return False
    return True

@dataclass(frozen=True)
class ProviderSpec:
    name: str; type: str; entrypoint: str; selector: dict; priority: int

@dataclass(frozen=True)
class SkillManifest:
    name: str; version: str; digest: str; verification: str; requires: tuple; preconditions: tuple; intent_examples: tuple; timeout_seconds: int; cancellation: str; recovery: tuple; safety_class: str; input_schema: dict; output_schema: dict; providers: tuple

@dataclass(frozen=True)
class InvocationResult:
    output: dict; provider_name: str; provider_type: str

class RegisteredSkill:
    def __init__(self, manifest):
        self.manifest = manifest

    def validate_inputs(self, inputs):
        errors = sorted(Draft202012Validator(self.manifest.input_schema).iter_errors(inputs), key=lambda e: list(e.path))
        if errors:
            raise RegistryError("Skill input does not match schema: " + errors[0].message)

    def invoke(self, profile, backend, inputs, lease):
        self.validate_inputs(inputs)
        mode = "simulation" if profile.simulation else "hardware"
        candidates = [p for p in self.manifest.providers if not p.selector.get("mode") or p.selector.get("mode") == mode]
        if not candidates:
            raise RegistryError("no Provider matches runtime mode")
        provider_spec = sorted(candidates, key=lambda p: (p.priority, p.name))[0]
        if provider_spec.type != "python_adapter":
            raise RegistryError("unsupported Provider type: " + provider_spec.type)
        module_name, class_name = provider_spec.entrypoint.split(":", 1)
        provider_class = getattr(importlib.import_module(module_name), class_name)
        output = provider_class(profile, backend).execute(inputs, lease)
        errors = sorted(Draft202012Validator(self.manifest.output_schema).iter_errors(output), key=lambda e: list(e.path))
        if errors:
            raise RegistryError("Provider output does not match schema: " + errors[0].message)
        return InvocationResult(output, provider_spec.name, provider_spec.type)

class SkillRegistry:
    def __init__(self):
        self._skills = {}

    def load_directory(self, root):
        for path in sorted(Path(root).glob("*/skill.yaml")):
            self.register(load_skill_manifest(path))
        if not self._skills:
            raise RegistryError("no Skill manifest found")
        return self

    def register(self, manifest):
        key = (manifest.name, manifest.version)
        if key in self._skills:
            raise RegistryError("duplicate Skill version: " + str(key))
        self._skills[key] = RegisteredSkill(manifest)

    def resolve(self, name, version_constraint=""):
        candidates = [s for (n, v), s in self._skills.items() if n == name and _constraint_matches(v, version_constraint)]
        return sorted(candidates, key=lambda s: _semver_key(s.manifest.version), reverse=True)[0] if candidates else None

    def names(self):
        return tuple(sorted({name for name, version in self._skills}))

    def intent_contracts(self):
        return {name: {"inputSchema": self.resolve(name).manifest.input_schema, "examples": self.resolve(name).manifest.intent_examples, "safetyClass": self.resolve(name).manifest.safety_class} for name in self.names()}

def load_skill_manifest(path):
    path = Path(path)
    raw = path.read_bytes()
    data = yaml.safe_load(raw) or {}
    meta = data.get("metadata") or {}
    spec = data.get("spec") or {}
    if data.get("kind") != "Skill" or not meta.get("name") or not meta.get("version"):
        raise RegistryError("invalid Skill manifest: " + str(path))
    _semver_key(meta["version"])
    required = ("inputSchema", "outputSchema", "requires", "preconditions", "intentExamples", "timeoutSeconds", "cancellation", "recovery", "safetyClass", "providers")
    missing = [key for key in required if key not in spec]
    if missing:
        raise RegistryError("Skill manifest missing fields: " + str(missing))
    def schema(name):
        try:
            return json.loads((path.parent / spec[name]).read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as exc:
            raise RegistryError("invalid Skill schema: " + str(path.parent / spec[name])) from exc
    providers = []
    for item in spec["providers"]:
        for key in ("name", "type", "entrypoint"):
            if key not in item:
                raise RegistryError("Provider missing field: " + key)
        if ":" not in str(item["entrypoint"]):
            raise RegistryError("Provider entrypoint must be module:Class")
        providers.append(ProviderSpec(item["name"], item["type"], item["entrypoint"], item.get("selector") or {}, int(item.get("priority", 100))))
    manifest = SkillManifest(meta["name"], str(meta["version"]), hashlib.sha256(raw).hexdigest(), str(spec.get("verification", "unverified")), tuple(spec["requires"]), tuple(spec["preconditions"]), tuple(spec["intentExamples"]), int(spec["timeoutSeconds"]), str(spec["cancellation"]), tuple(spec["recovery"]), str(spec["safetyClass"]), schema("inputSchema"), schema("outputSchema"), tuple(providers))
    if manifest.timeout_seconds <= 0 or not manifest.providers:
        raise RegistryError("Skill timeout/providers invalid")
    return manifest
