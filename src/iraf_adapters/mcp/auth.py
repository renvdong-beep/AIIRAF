# -*- coding: utf-8 -*-
"""MCP 北向鉴权：不透明 Bearer token → Principal（身份只来自受信 transport）。

为什么单独成模块（2026-11-03）：
  对外交付的 harness/LLM 场景会以 **streamable-http** 连 IRAF 的 MCP 端点。原实现只支持
  stdio（进程即身份），HTTP 形态没有任何鉴权；一旦跨机/多租户使用，就等于把
  `iraf_submit_task` 变成无身份后门（违反 AGENTS 1.8：身份只能来自受信 transport）。

设计要点（fail-closed）：
  · token 只在签发时明文出现一次；注册表只存 SHA-256，权限必须 600，否则拒绝加载；
  · 认证结果是一个不可变 Principal（subject/tenant/scopes/resources/token_id）；
  · 工具层只读 contextvar 里的 Principal，**不读请求体里的 caller/role/tenant**；
  · 撤销/过期按文件 mtime 热加载，不用重启进程；
  · stdio 形态没有网络鉴权，身份由 IRAF_RUNTIME_SUBJECT 给出（进程边界即信任边界）。

本模块只依赖标准库，stdio 形态 import 它不会引入 web 依赖。
"""
from __future__ import annotations

import contextlib
import contextvars
import datetime as _dt
import hashlib
import hmac
import json
import os
import sys
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, Iterable, Optional, Tuple

#: 允许的 scope 全集（与 PolicyGateway 的 context.roles 对齐）。
KNOWN_SCOPES = ("task.submit", "task.read")
DEFAULT_HTTP_SCOPES = ("task.submit", "task.read")

CODE_UNAUTHENTICATED = "IRAF-UNAUTHENTICATED"
CODE_FORBIDDEN = "IRAF-POLICY-DENIED"
CODE_AUTH_NOT_CONFIGURED = "IRAF-AUTH-NOT-CONFIGURED"


class AuthError(RuntimeError):
    """鉴权/授权失败：带 HTTP 状态与稳定错误码，调用方不得据此猜测身份。"""

    def __init__(self, code: str, reason: str, *, http_status: int = 401):
        self.code = code
        self.reason = reason
        self.http_status = int(http_status)
        super().__init__("%s: %s" % (code, reason))


@dataclass(frozen=True)
class Principal:
    """认证后的调用方身份（不可变）。

    resources 为允许的 resource_id 集合；`("*",)` 表示该部署不按资源隔离（单租户）。
    """

    subject: str
    tenant: str
    scopes: frozenset
    resources: Tuple[str, ...] = ("*",)
    token_id: str = ""
    mtls_cn: str = ""

    def has(self, scope: str) -> bool:
        return scope in self.scopes

    def allows_resource(self, resource_id: str) -> bool:
        if not self.resources or "*" in self.resources:
            return True
        return str(resource_id or "") in self.resources

    def as_metadata(self) -> dict:
        """可进审计/日志的字段（绝不含 token 明文或哈希）。"""
        return {"subject": self.subject, "tenant": self.tenant,
                "scopes": sorted(self.scopes), "token_id": self.token_id}


def hash_token(token: str) -> str:
    return hashlib.sha256(token.encode("utf-8")).hexdigest()


def parse_bearer(value: Optional[str]) -> str:
    """从 Authorization 头取 Bearer token；缺失/格式错都显式失败。"""
    raw = (value or "").strip()
    if not raw:
        raise AuthError(CODE_UNAUTHENTICATED, "缺少 Authorization: Bearer <token>")
    parts = raw.split(None, 1)
    if len(parts) != 2 or parts[0].lower() != "bearer" or not parts[1].strip():
        raise AuthError(CODE_UNAUTHENTICATED, "Authorization 头不是 Bearer <token> 形式")
    return parts[1].strip()


def _parse_iso(value: str) -> float:
    text = str(value or "").strip()
    if not text:
        return 0.0
    if text.endswith("Z"):
        text = text[:-1] + "+00:00"
    return _dt.datetime.fromisoformat(text).timestamp()


@dataclass
class _Entry:
    token_sha256: str
    subject: str
    tenant: str
    scopes: frozenset
    resources: Tuple[str, ...]
    token_id: str
    enabled: bool = True
    expires_at: float = 0.0
    mtls_cn: str = ""


class TokenRegistry:
    """token 注册表（文件为唯一事实来源；只存哈希，600 权限）。"""

    def __init__(self, path: Path, entries: Iterable[_Entry]):
        self.path = Path(path)
        self._entries = tuple(entries)
        self._signature = self._stat_signature(self.path)
        self._lock = threading.RLock()

    # ---------- 加载 ----------
    @staticmethod
    def _stat_signature(path: Path) -> tuple:
        try:
            info = path.stat()
        except OSError:
            return (0, 0, 0, 0)
        # 四个量一起看：只比较 mtime 在粗粒度文件系统/同一秒内重写时会漏掉撤销。
        return (info.st_mtime_ns, info.st_size, info.st_ino, info.st_ctime_ns)

    @classmethod
    def from_file(cls, path) -> "TokenRegistry":
        target = Path(path)
        try:
            mode = target.stat().st_mode
        except OSError as exc:
            raise AuthError(CODE_UNAUTHENTICATED,
                            "token 注册表不可读：%s（%s）" % (target, exc), http_status=503)
        if mode & 0o077:
            raise AuthError(CODE_UNAUTHENTICATED,
                            "token 注册表权限过宽（%o）：必须 600，拒绝加载" % (mode & 0o777),
                            http_status=503)
        raw = json.loads(target.read_text(encoding="utf-8"))
        if raw.get("version") != 1 or not isinstance(raw.get("tokens"), list):
            raise AuthError(CODE_UNAUTHENTICATED,
                            "token 注册表格式非法：需要 version=1 与 tokens 数组", http_status=503)
        entries = []
        seen_ids = set()
        for item in raw["tokens"]:
            if not isinstance(item, dict):
                raise AuthError(CODE_UNAUTHENTICATED, "tokens[] 必须是对象", http_status=503)
            digest = str(item.get("sha256") or "").strip().lower()
            if len(digest) != 64 or any(ch not in "0123456789abcdef" for ch in digest):
                raise AuthError(CODE_UNAUTHENTICATED,
                                "token 项缺少合法的 sha256（只存哈希，不存明文）", http_status=503)
            subject = str(item.get("subject") or "").strip()
            if not subject:
                raise AuthError(CODE_UNAUTHENTICATED, "token 项缺少 subject", http_status=503)
            token_id = str(item.get("id") or "").strip()
            if not token_id:
                raise AuthError(CODE_UNAUTHENTICATED, "token 项缺少 id（审计/撤销用）", http_status=503)
            if token_id in seen_ids:
                raise AuthError(CODE_UNAUTHENTICATED, "token id 重复：%s" % token_id, http_status=503)
            seen_ids.add(token_id)
            scopes = item.get("scopes") or list(DEFAULT_HTTP_SCOPES)
            if not isinstance(scopes, list) or not scopes:
                raise AuthError(CODE_UNAUTHENTICATED, "token 项 scopes 必须是非空数组", http_status=503)
            unknown = sorted(set(scopes) - set(KNOWN_SCOPES))
            if unknown:
                raise AuthError(CODE_UNAUTHENTICATED,
                                "token 项含未知 scope：%s" % ",".join(unknown), http_status=503)
            resources = item.get("resources")
            if resources is None:
                resources = ["*"]
            if not isinstance(resources, list) or not all(isinstance(r, str) and r for r in resources):
                raise AuthError(CODE_UNAUTHENTICATED,
                                "token 项 resources 必须是非空字符串数组", http_status=503)
            entries.append(_Entry(
                token_sha256=digest,
                subject=subject,
                tenant=str(item.get("tenant") or "default"),
                scopes=frozenset(str(s) for s in scopes),
                resources=tuple(str(r) for r in resources),
                token_id=token_id,
                enabled=bool(item.get("enabled", True)),
                expires_at=_parse_iso(item.get("expires_at") or ""),
                mtls_cn=str(item.get("mtls_cn") or ""),
            ))
        if not entries:
            raise AuthError(CODE_UNAUTHENTICATED, "token 注册表为空", http_status=503)
        return cls(target, entries)

    # ---------- 热加载 ----------
    def reload_if_changed(self) -> None:
        signature = self._stat_signature(self.path)
        if signature == self._signature:
            return
        with self._lock:
            if signature == self._signature:
                return
            fresh = type(self).from_file(self.path)
            self._entries = fresh._entries
            self._signature = fresh._signature

    # ---------- 认证 ----------
    def authenticate(self, token: str, *, peer_cn: Optional[str] = None,
                     now: Optional[float] = None) -> Principal:
        """校验 token → Principal。成功/失败都走同一比较循环，避免按 token 提前返回。"""
        if not token:
            raise AuthError(CODE_UNAUTHENTICATED, "token 为空")
        self.reload_if_changed()
        presented = hash_token(token)
        moment = time.time() if now is None else float(now)
        matched: Optional[_Entry] = None
        with self._lock:
            entries = self._entries
        for entry in entries:
            if hmac.compare_digest(presented, entry.token_sha256):
                matched = entry
        if matched is None:
            raise AuthError(CODE_UNAUTHENTICATED, "token 无效")
        if not matched.enabled:
            raise AuthError(CODE_UNAUTHENTICATED, "token 已停用")
        if matched.expires_at and moment >= matched.expires_at:
            raise AuthError(CODE_UNAUTHENTICATED, "token 已过期")
        if matched.mtls_cn and (peer_cn or "") != matched.mtls_cn:
            raise AuthError(CODE_UNAUTHENTICATED, "mTLS 客户端证书 CN 与 token 绑定不符")
        return Principal(subject=matched.subject, tenant=matched.tenant,
                         scopes=matched.scopes, resources=matched.resources,
                         token_id=matched.token_id, mtls_cn=matched.mtls_cn)

    def redacted(self) -> list:
        """管理命令用：只列元数据，不泄露哈希。"""
        with self._lock:
            return [{"id": e.token_id, "subject": e.subject, "tenant": e.tenant,
                     "scopes": sorted(e.scopes), "resources": list(e.resources),
                     "enabled": e.enabled,
                     "expires_at": (_dt.datetime.fromtimestamp(e.expires_at, _dt.timezone.utc)
                                    .isoformat().replace("+00:00", "Z") if e.expires_at else "")}
                    for e in self._entries]


# ---------------------------------------------------------------------------
# 进程内身份传递（stdio: 环境；http: 中间件 set/reset）
# ---------------------------------------------------------------------------
_CURRENT: contextvars.ContextVar[Optional[Principal]] = contextvars.ContextVar(
    "iraf_mcp_principal", default=None)


def current_principal() -> Optional[Principal]:
    return _CURRENT.get()


@contextlib.contextmanager
def bind_principal(principal: Principal):
    token = _CURRENT.set(principal)
    try:
        yield principal
    finally:
        _CURRENT.reset(token)


def env_principal() -> Principal:
    """stdio 形态的身份：进程边界即信任边界，身份来自部署环境。"""
    subject = (os.environ.get("IRAF_RUNTIME_SUBJECT") or "").strip()
    if not subject:
        raise AuthError(CODE_UNAUTHENTICATED,
                        "stdio 形态缺少 IRAF_RUNTIME_SUBJECT（身份不能猜）", http_status=503)
    raw_scopes = os.environ.get("IRAF_MCP_SCOPES") or ",".join(DEFAULT_HTTP_SCOPES)
    scopes = frozenset(s.strip() for s in raw_scopes.split(",") if s.strip())
    if not scopes or set(scopes) - set(KNOWN_SCOPES):
        raise AuthError(CODE_UNAUTHENTICATED,
                        "IRAF_MCP_SCOPES 非法：%s" % raw_scopes, http_status=503)
    raw_resources = os.environ.get("IRAF_MCP_RESOURCES") or ""
    resources = tuple(s.strip() for s in raw_resources.split(",") if s.strip()) or ("*",)
    return Principal(subject=subject,
                     tenant=(os.environ.get("IRAF_MCP_TENANT") or "default").strip(),
                     scopes=scopes, resources=resources, token_id="local",
                     mtls_cn="")


def require_scope(principal: Principal, scope: str) -> None:
    if not principal.has(scope):
        raise AuthError(CODE_FORBIDDEN,
                        "调用方 %s 缺少 scope %s" % (principal.subject, scope), http_status=403)


def require_resource(principal: Principal, resource_id: str) -> None:
    if not principal.allows_resource(resource_id):
        raise AuthError(CODE_FORBIDDEN,
                        "调用方 %s 无权访问资源 %s" % (principal.subject, resource_id),
                        http_status=403)


# ---------------------------------------------------------------------------
# 审计（可选；失败只告警，不改变判定）
# ---------------------------------------------------------------------------
_AUDIT_LOCK = threading.Lock()


def audit(event: str, principal: Optional[Principal] = None, **fields) -> None:
    path = (os.environ.get("IRAF_MCP_AUDIT_LOG") or "").strip()
    if not path:
        return
    record = {"ts": _dt.datetime.now(_dt.timezone.utc).isoformat().replace("+00:00", "Z"),
              "event": event}
    if principal is not None:
        record.update(principal.as_metadata())
    record.update({k: v for k, v in fields.items() if v is not None})
    try:
        line = json.dumps(record, ensure_ascii=False, sort_keys=True) + "\n"
        with _AUDIT_LOCK:
            with open(path, "a", encoding="utf-8") as handle:
                handle.write(line)
    except OSError as exc:  # 审计写失败不阻断业务，但必须可见
        print("[iraf-mcp][警告] 审计日志写入失败：%s" % exc, file=sys.stderr, flush=True)


# ---------------------------------------------------------------------------
# ASGI 中间件（纯 ASGI，不依赖 starlette）
# ---------------------------------------------------------------------------
def _peer_cn(scope) -> Optional[str]:
    """尽力取 mTLS 客户端证书 CN；当前 uvicorn 不暴露时返回 None（由部署层保证）。"""
    extensions = scope.get("extensions") or {}
    tls = extensions.get("tls") or {}
    chain = tls.get("client_cert_chain") or ()
    if chain:
        return str(chain[0].get("subject", "") or "").strip() or None
    state = scope.get("state") or {}
    return str(state.get("client_cert_cn") or "") or None


class BearerAuthMiddleware:
    """对除 /healthz 外的所有 HTTP 请求做 Bearer 认证，并把 Principal 绑进 contextvar。"""

    def __init__(self, app, registry: TokenRegistry):
        self.app = app
        self.registry = registry

    async def __call__(self, scope, receive, send):
        if scope.get("type") != "http":
            return await self.app(scope, receive, send)
        path = scope.get("path") or ""
        if path in ("/healthz", "/health"):
            body = b'{"status":"ok"}'
            await send({"type": "http.response.start", "status": 200,
                        "headers": [(b"content-type", b"application/json"),
                                    (b"content-length", str(len(body)).encode())]})
            await send({"type": "http.response.body", "body": body})
            return
        headers = {k.decode("latin-1").lower(): v.decode("latin-1")
                   for k, v in (scope.get("headers") or [])}
        remote = ""
        client = scope.get("client")
        if client:
            remote = "%s:%s" % (client[0], client[1])
        try:
            token = parse_bearer(headers.get("authorization"))
            principal = self.registry.authenticate(token, peer_cn=_peer_cn(scope))
        except AuthError as exc:
            audit("auth_failed", reason=exc.code, remote=remote)
            body = json.dumps({"error": exc.code, "reason": exc.reason},
                              ensure_ascii=False).encode("utf-8")
            await send({"type": "http.response.start", "status": exc.http_status,
                        "headers": [(b"content-type", b"application/json; charset=utf-8"),
                                    (b"content-length", str(len(body)).encode()),
                                    (b"www-authenticate", b'Bearer realm="iraf"')]})
            await send({"type": "http.response.body", "body": body})
            return
        audit("auth_ok", principal, remote=remote)
        with bind_principal(principal):
            return await self.app(scope, receive, send)
