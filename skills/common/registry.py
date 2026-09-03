"""Compatibility wrapper; Registry remains a core service."""
from iraf_core.registry import SkillRegistry, SkillManifest, RegistryError, load_skill_manifest

__all__ = ["SkillRegistry", "SkillManifest", "RegistryError", "load_skill_manifest"]
