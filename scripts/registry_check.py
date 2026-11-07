#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""SceneRegistry / ProviderRegistry 校验 CLI（薄壳）

规则全部在 `iraf_core.scene_registry`，本脚本只做参数解析与输出——
避免"脚本一套规则、库里另一套规则"的漂移。

用法
----
    python3 scripts/registry_check.py --root /etc/iraf \
        --profiles-root /opt/iraf-sdk/current/profiles
    python3 scripts/registry_check.py --root config --seal
    python3 scripts/registry_check.py --root config --json

退出码：0 全部通过 / 2 校验失败 / 3 目录或依赖缺失
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

# 仓库内运行（scripts/ 的上一级有 src/）与安装树运行（/opt/iraf-sdk/current/src）都要能找到 lib
_cands = [Path(__file__).resolve().parents[1] / "src", Path("/opt/iraf-sdk/current/src"),
          Path("/opt/iraf-sdk/0.4.1/src")]
for _c in _cands:
    if (_c / "iraf_core").is_dir():
        sys.path.insert(0, str(_c))
        break

from iraf_core.scene_registry import RegistryError, SceneRegistry  # noqa: E402


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="校验 SceneRegistry / ProviderRegistry")
    ap.add_argument("--root", default="/etc/iraf", help="注册表根（含 scenes/、providers/）")
    ap.add_argument("--profiles-root", default="/opt/iraf-sdk/current/profiles",
                    help="profile 目录（用于 capabilities 子集校验）")
    ap.add_argument("--seal", action="store_true", help="重新计算并写回 digest")
    ap.add_argument("--json", action="store_true", help="JSON 输出")
    args = ap.parse_args(argv)

    root = Path(args.root)
    if not root.is_dir():
        print("目录不存在：%s" % root, file=sys.stderr)
        return 3
    try:
        reg = SceneRegistry(root=root, profiles_root=Path(args.profiles_root))
    except Exception as exc:  # pragma: no cover
        print("初始化失败：%s" % exc, file=sys.stderr)
        return 3

    if args.seal:
        n = reg.seal()
        print("sealed %d files under %s" % (n, root))
        return 0

    try:
        reg.reload(force=True)
    except RegistryError as exc:
        payload = {"ok": False, "errors": [exc.detail], "warnings": reg.warnings}
        print(json.dumps(payload, ensure_ascii=False, indent=1) if args.json
              else "FAIL %s" % exc.detail)
        return 2

    status = reg.status()
    errors = status["registry"]["errors"]
    warnings = status["registry"]["warnings"]
    stats = {"scenes": len(status["scenes"]), "providers": len(status["providers"])}

    if args.json:
        print(json.dumps({"ok": not errors, "stats": stats, "errors": errors,
                          "warnings": warnings}, ensure_ascii=False, indent=1))
    else:
        for w in warnings:
            print("WARN %s" % w)
        for e in errors:
            print("FAIL %s" % e)
        if not errors:
            print("OK scenes=%d providers=%d" % (stats["scenes"], stats["providers"]))
    return 0 if not errors else 2


if __name__ == "__main__":
    sys.exit(main())
