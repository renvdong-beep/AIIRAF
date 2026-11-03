# -*- coding: utf-8 -*-
"""MCP token 管理 CLI（签发/列出/撤销），只存 SHA-256，不存明文。

用法（推荐 root 或安装用户）：
  python -m iraf_tools.mcp_tokens issue --subject harness-a --tenant cust-a \
      --scopes task.submit,task.read --resources piper_mujoco-mujoco \
      --file ~/.config/iraf/mcp-tokens.json
  python -m iraf_tools.mcp_tokens list --file ...
  python -m iraf_tools.mcp_tokens revoke --id <token-id> --file ...

安全约定：
  · 明文只在 issue 的标准输出出现一次，注册表写 600；不要写进 shell 历史/日志；
  · 注册表权限过宽（group/other 可读）时拒绝读写（与 server 侧 fail-closed 一致）；
  · 撤销/过期由 server 按文件 mtime 热加载，不需要重启。
"""
from __future__ import annotations

import argparse
import datetime as _dt
import hashlib
import json
import os
import secrets
import sys
import tempfile
from pathlib import Path

KNOWN_SCOPES = ("task.submit", "task.read")
DEFAULT_FILE = Path.home() / ".config/iraf/mcp-tokens.json"


def _fail(message: str) -> int:
    print("[iraf-mcp-token][错误] %s" % message, file=sys.stderr)
    return 2


def _load(path: Path) -> dict:
    if not path.is_file():
        return {"version": 1, "tokens": []}
    mode = path.stat().st_mode
    if mode & 0o077:
        raise ValueError("注册表权限过宽（%o）：必须 600" % (mode & 0o777))
    data = json.loads(path.read_text(encoding="utf-8"))
    if data.get("version") != 1 or not isinstance(data.get("tokens"), list):
        raise ValueError("注册表格式非法：需要 version=1 与 tokens 数组")
    return data


def _write(path: Path, data: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, tmp = tempfile.mkstemp(prefix=".mcp-tokens.", dir=str(path.parent))
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as handle:
            json.dump(data, handle, ensure_ascii=False, indent=2, sort_keys=True)
            handle.write("\n")
        os.chmod(tmp, 0o600)
        os.replace(tmp, path)
    finally:
        if os.path.exists(tmp):
            os.unlink(tmp)


def _expires(days: int) -> str:
    if days <= 0:
        return ""
    moment = _dt.datetime.now(_dt.timezone.utc) + _dt.timedelta(days=days)
    return moment.isoformat().replace("+00:00", "Z")


def cmd_issue(args) -> int:
    scopes = [s.strip() for s in (args.scopes or "").split(",") if s.strip()]
    if not scopes:
        return _fail("--scopes 不能为空（例如 task.submit,task.read）")
    unknown = sorted(set(scopes) - set(KNOWN_SCOPES))
    if unknown:
        return _fail("未知 scope：%s（允许：%s）" % (",".join(unknown), ",".join(KNOWN_SCOPES)))
    resources = [r.strip() for r in (args.resources or "").split(",") if r.strip()] or ["*"]
    path = Path(args.file).expanduser()
    try:
        data = _load(path)
    except (ValueError, OSError, json.JSONDecodeError) as exc:
        return _fail(str(exc))
    token_id = args.id or ("tok-" + secrets.token_hex(4))
    if any(item.get("id") == token_id for item in data["tokens"]):
        return _fail("token id 已存在：%s" % token_id)
    token = secrets.token_urlsafe(32)
    data["tokens"].append({
        "id": token_id,
        "sha256": hashlib.sha256(token.encode("utf-8")).hexdigest(),
        "subject": args.subject,
        "tenant": args.tenant or "default",
        "scopes": scopes,
        "resources": resources,
        "enabled": True,
        "expires_at": _expires(args.expires_days),
        "note": args.note or "",
    })
    try:
        _write(path, data)
    except OSError as exc:
        return _fail("写入注册表失败：%s（%s）" % (path, exc))
    print("[iraf-mcp-token] 已签发：id=%s subject=%s tenant=%s scopes=%s resources=%s"
          % (token_id, args.subject, args.tenant or "default", ",".join(scopes), ",".join(resources)))
    print("[iraf-mcp-token] 注册表：%s（600，只存 SHA-256）" % path)
    print("[iraf-mcp-token] ⚠ 明文只显示这一次，请立即交给调用方，不要写进日志/仓库：")
    print(token)
    return 0


def cmd_list(args) -> int:
    path = Path(args.file).expanduser()
    try:
        data = _load(path)
    except (ValueError, OSError, json.JSONDecodeError) as exc:
        return _fail(str(exc))
    if not data["tokens"]:
        print("[iraf-mcp-token] 注册表为空：%s" % path)
        return 0
    for item in data["tokens"]:
        print("%-16s subject=%-20s tenant=%-12s enabled=%-5s expires=%-20s resources=%s"
              % (item.get("id"), item.get("subject"), item.get("tenant"),
                 item.get("enabled", True), item.get("expires_at") or "-",
                 ",".join(item.get("resources") or [])))
    return 0


def cmd_revoke(args) -> int:
    path = Path(args.file).expanduser()
    try:
        data = _load(path)
    except (ValueError, OSError, json.JSONDecodeError) as exc:
        return _fail(str(exc))
    hit = [item for item in data["tokens"] if item.get("id") == args.id]
    if not hit:
        return _fail("未找到 token id：%s" % args.id)
    for item in hit:
        item["enabled"] = False
        item["revoked_at"] = _dt.datetime.now(_dt.timezone.utc).isoformat().replace("+00:00", "Z")
    try:
        _write(path, data)
    except OSError as exc:
        return _fail("写入注册表失败：%s（%s）" % (path, exc))
    print("[iraf-mcp-token] 已撤销 id=%s（服务端按 mtime 热加载，无需重启）" % args.id)
    return 0


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(prog="iraf-mcp-token", description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--file", default=os.environ.get("IRAF_MCP_TOKEN_FILE") or str(DEFAULT_FILE),
                        help="注册表路径（默认 %(default)s）")
    sub = parser.add_subparsers(dest="command", required=True)

    issue = sub.add_parser("issue", help="签发一个新 token（明文只输出一次）")
    issue.add_argument("--subject", required=True)
    issue.add_argument("--tenant", default="default")
    issue.add_argument("--scopes", default="task.submit,task.read")
    issue.add_argument("--resources", default="*", help="允许的 resource_id，逗号分隔；* 表示部署级不隔离")
    issue.add_argument("--id", default="")
    issue.add_argument("--expires-days", type=int, default=90)
    issue.add_argument("--note", default="")
    issue.set_defaults(func=cmd_issue)

    listing = sub.add_parser("list", help="列出 token 元数据（不含哈希/明文）")
    listing.set_defaults(func=cmd_list)

    revoke = sub.add_parser("revoke", help="撤销 token（置 enabled=false）")
    revoke.add_argument("--id", required=True)
    revoke.set_defaults(func=cmd_revoke)

    args = parser.parse_args(argv)
    return args.func(args)


if __name__ == "__main__":
    raise SystemExit(main())
