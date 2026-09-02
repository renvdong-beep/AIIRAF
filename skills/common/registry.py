"""兼容导入；Registry 的唯一实现位于 iraf_core.registry。"""
from iraf_core.registry import SkillRegistry,SkillManifest,RegistryError,load_skill_manifest
__all__=["SkillRegistry","SkillManifest","RegistryError","load_skill_manifest"]