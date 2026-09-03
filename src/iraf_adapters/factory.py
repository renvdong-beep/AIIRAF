"""从部署配置装配 Capability Backend。"""
import importlib
def load_backend(entrypoint,config,profile,authority):
    if not entrypoint or ":" not in entrypoint: raise ValueError("IRAF_BACKEND_ENTRYPOINT 无效")
    module_name,class_name=entrypoint.split(":",1); backend_class=getattr(importlib.import_module(module_name),class_name)
    if not hasattr(backend_class,"from_config"): raise ValueError("Backend 必须实现 from_config")
    return backend_class.from_config(config,profile,authority)