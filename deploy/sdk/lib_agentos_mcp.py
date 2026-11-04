# -*- coding: utf-8 -*-
"""AgentOS worker 的 MCP 配置合并（纯逻辑，便于单测；被 board_agentos_mcp_bind.sh 调用）。

为什么单独成模块：
  · config.yaml 是 AgentOS 的内容层配置，可能已有 model / 其他 mcp_servers；
    直接 append 会产生重复顶层键或覆盖别的 server。
  · 这里只做"读 → 合并 iraf-task → 原子写回"，并保留其它键；
  · 不 import AgentOS，不依赖容器。

注意：PyYAML 会丢注释；写前必须备份（脚本负责）。
"""
from __future__ import annotations

import os
import tempfile
from pathlib import Path
from typing import Mapping, Optional

DEFAULT_SERVER_NAME = "iraf-task"
WORKER_PREFIX = "agentos-worker-"


def discover_shared_dir(inspect_payload: Mapping) -> str:
    """从 docker inspect 的 Mounts 里找 /home/agentos/.intewell 的宿主源目录。"""
    for mount in inspect_payload.get("Mounts") or []:
        if (mount.get("Destination") == "/home/agentos/.intewell"
                and mount.get("Type") == "bind" and mount.get("Source")):
            return str(mount["Source"])
    return ""


def merge_server(config: Optional[Mapping], *, name: str, url: str, token: str) -> dict:
    """把 iraf-task 合并进 config，保留其它键与 server；幂等。"""
    data = dict(config or {})
    servers = data.get("mcp_servers")
    if servers is None:
        servers = {}
    if not isinstance(servers, Mapping):
        raise ValueError("mcp_servers 必须是对象，实际：%s" % type(servers).__name__)
    servers = dict(servers)
    servers[name] = {
        "url": str(url),
        "headers": {"Authorization": "Bearer " + str(token)},
        "enabled": True,
    }
    data["mcp_servers"] = servers
    return data


def remove_server(config: Optional[Mapping], *, name: str) -> tuple[dict, bool]:
    """删除指定 server；返回 (新配置, 是否删除)。保留其余 server。"""
    data = dict(config or {})
    servers = data.get("mcp_servers")
    if not isinstance(servers, Mapping) or name not in servers:
        return data, False
    servers = dict(servers)
    servers.pop(name, None)
    if servers:
        data["mcp_servers"] = servers
    else:
        data.pop("mcp_servers", None)
    return data, True


def load_config(path) -> dict:
    import yaml

    target = Path(path)
    if not target.is_file():
        return {}
    text = target.read_text(encoding="utf-8")
    data = yaml.safe_load(text) if text.strip() else {}
    if data is None:
        return {}
    if not isinstance(data, dict):
        raise ValueError("config.yaml 顶层必须是对象，实际：%s" % type(data).__name__)
    return data


def write_config(path, config, *, owner=None, backup_suffix=None) -> None:
    """原子写 YAML；可选 chown(owner=(uid,gid))；备份由调用方先做。"""
    import yaml

    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    text = yaml.safe_dump(dict(config), allow_unicode=True, sort_keys=False)
    fd, tmp = tempfile.mkstemp(prefix=".config.yaml.", dir=str(target.parent))
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as handle:
            handle.write(text)
        os.chmod(tmp, 0o600)
        if owner is not None:
            os.chown(tmp, int(owner[0]), int(owner[1]))
        os.replace(tmp, target)
    finally:
        if os.path.exists(tmp):
            os.unlink(tmp)
