#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把模型网关配置（**含 api_key**）写进 AgentOS 的 config.yaml —— 宿主与 worker 容器各一份。

为什么需要（2026-10-23 实机）：
  · 平台 UI / 部分路由只看 `model.*` 段；api_key 只在 .env 与进程环境里时，
    页面会把它判成"未配置"（./apply_board_dashboard_apikey.sh 的 restore 脚本故意不写 key）。
  · 网页 /chat 跑在 worker 容器里，容器家目录 bind 到 /var/lib/agentos-worker-<n>/.intewell，
    那里**根本没有 config.yaml** ⇒ 容器内的 agent 只能用环境变量凑，模型配置不可见。

边界：
  · 只写 config.yaml（内容层配置），不动框架/代码/容器镜像；
  · 写入前逐份备份 *.bak-model-key，可 --revert 原样还原；
  · key 值不打印（只报长度与头 6 位），文件 600，容器 home 保持 10001 属主。

用法（开发机，经 ssh stdin 送远端 python3）：
  sshpass -e ssh root@<板> "python3 - --status" < scripts/agentos_model_config.py
  sshpass -e ssh root@<板> "python3 - --apply"  < scripts/agentos_model_config.py
  sshpass -e ssh root@<板> "python3 - --revert" < scripts/agentos_model_config.py
包装：scripts/agentos_content_layer.sh --model-config [--revert]
"""
import os
import shutil
import sys
from pathlib import Path

KEY_FILE = Path("/etc/iraf/agentos.key")
HOST_HOME = Path("/root/.intewell")
WORKER_GLOB = "/var/lib/agentos-worker-*/.intewell"
DEFAULT_MODEL = "deepseek-flash"
DEFAULT_PROVIDER = "custom"
DEFAULT_BASE_URL = "http://192.168.100.89:3000/v1"
BACKUP_SUFFIX = ".bak-model-key"


def _yaml():
    import yaml  # 板上已装（安装器依赖同源）；缺了就显式失败
    return yaml


def read_key() -> str:
    if not KEY_FILE.is_file():
        raise SystemExit("缺 %s（模型网关 key；600）" % KEY_FILE)
    value = KEY_FILE.read_text(encoding="utf-8").strip()
    if not value:
        raise SystemExit("%s 为空" % KEY_FILE)
    return value


def targets() -> list:
    """要处理的 config.yaml 路径：宿主 + 每个 worker 容器家目录（存在才写）。"""
    out = [HOST_HOME / "config.yaml"]
    for base in sorted(Path("/var/lib").glob("agentos-worker-*/.intewell")):
        out.append(base / "config.yaml")
    return out


def desired(existing: dict, key: str) -> dict:
    model = dict(existing.get("model") or {})
    model.setdefault("default", DEFAULT_MODEL)
    model.setdefault("provider", DEFAULT_PROVIDER)
    model.setdefault("base_url", DEFAULT_BASE_URL)
    model["api_key"] = key          # ← 本次要修的字段
    data = dict(existing)
    data["model"] = model
    return data


def status() -> int:
    yaml = _yaml()
    for path in targets():
        if not path.is_file():
            print("  ✗ 不存在      %s" % path)
            continue
        data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
        model = data.get("model") or {}
        key = str(model.get("api_key") or "")
        head = (key[:6] + "…") if key else "（空）"
        print("  %s api_key=%s len=%d  provider=%s default=%s  %s"
              % ("✓" if key else "✗", head, len(key), model.get("provider"),
                 model.get("default"), path))
    return 0


def apply() -> int:
    yaml = _yaml()
    key = read_key()
    for path in targets():
        path.parent.mkdir(parents=True, exist_ok=True)
        existing = {}
        if path.is_file():
            existing = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
            backup = path.with_name(path.name + BACKUP_SUFFIX)
            if not backup.exists():
                shutil.copy2(path, backup)
        data = desired(existing, key)
        path.write_text(yaml.safe_dump(data, allow_unicode=True, sort_keys=False),
                        encoding="utf-8")
        os.chmod(path, 0o600)
        # 容器 home 的属主是 10001（bind 进容器后由 agentos 用户读写），别留成 root-only
        if str(path).startswith("/var/lib/agentos-worker-"):
            try:
                os.chown(path, 10001, 10001)
            except OSError as exc:
                print("  [警告] chown 10001:10001 失败（%s）：%s" % (path, exc))
        print("  ✓ 已写 api_key（len=%d, head=%s…）→ %s" % (len(key), key[:6], path))
    print("\n提示：dashboard 与 worker 容器需要重启才会重新读取（容器重启会丢手工 pip 包，注意）。")
    return 0


def revert() -> int:
    for path in targets():
        backup = path.with_name(path.name + BACKUP_SUFFIX)
        if backup.is_file():
            shutil.copy2(backup, path)
            os.chmod(path, 0o600)
            if str(path).startswith("/var/lib/agentos-worker-"):
                try:
                    os.chown(path, 10001, 10001)
                except OSError:
                    pass
            print("  ✓ 已还原 ← %s" % backup)
        else:
            print("  - 无备份，跳过 %s" % path)
    return 0


if __name__ == "__main__":
    if "--status" in sys.argv:
        raise SystemExit(status())
    if "--revert" in sys.argv:
        raise SystemExit(revert())
    raise SystemExit(apply())
