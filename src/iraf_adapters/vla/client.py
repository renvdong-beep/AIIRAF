"""VLA 推理 HTTP 客户端（端侧 runtime / x86 服务的统一入口）。"""
from __future__ import annotations

import base64
import json
import time
import urllib.error
import urllib.request
from dataclasses import dataclass
from typing import Any, Mapping


class VlaError(RuntimeError):
    """显式失败：网络、鉴权、模型错误，不伪造动作。"""

    def __init__(self, code: str, detail: str):
        super().__init__("%s: %s" % (code, detail))
        self.code = code
        self.detail = detail


@dataclass(frozen=True)
class VlaResult:
    model_id: str
    actions: tuple[float, ...]
    chunk: tuple[tuple[float, ...], ...]
    latency_ms: float
    raw: Mapping[str, Any]


class VlaClient:
    """调用 `/v1/vla/infer`；服务实现见 `vla_server.py`（SmolVLA 原型）。"""

    def __init__(self, base_url: str, timeout_s: float = 30.0, token: str | None = None):
        self.base_url = base_url.rstrip("/")
        self.timeout_s = float(timeout_s)
        self.token = token

    def _request(self, path: str, payload: dict | None = None) -> dict:
        data = None if payload is None else json.dumps(payload).encode()
        headers = {"Content-Type": "application/json"}
        if self.token:
            headers["Authorization"] = "Bearer " + self.token
        req = urllib.request.Request(self.base_url + path, data=data,
                                     headers=headers, method="GET" if data is None else "POST")
        try:
            with urllib.request.urlopen(req, timeout=self.timeout_s) as resp:
                return json.loads(resp.read().decode())
        except urllib.error.HTTPError as exc:
            raise VlaError("HTTP-%d" % exc.code, exc.read().decode()[:200]) from exc
        except (urllib.error.URLError, TimeoutError) as exc:
            raise VlaError("TRANSPORT", str(exc)[:200]) from exc
        except json.JSONDecodeError as exc:
            raise VlaError("BAD-RESPONSE", "not json") from exc

    def health(self) -> dict:
        return self._request("/health")

    def infer(self, task: str, state: list[float],
              images: Mapping[str, bytes | str]) -> VlaResult:
        encoded = {}
        for name, img in images.items():
            if isinstance(img, bytes):
                encoded[name] = base64.b64encode(img).decode()
            else:
                encoded[name] = img
        t0 = time.time()
        out = self._request("/v1/vla/infer", {"task": task, "state": list(map(float, state)),
                                             "images": encoded})
        if out.get("status") != "OK":
            raise VlaError("MODEL", str(out.get("error", out))[:200])
        actions = tuple(float(v) for v in out["actions"])
        chunk = tuple(tuple(float(v) for v in row) for row in out.get("chunk", [actions]))
        return VlaResult(model_id=str(out.get("model_id", "")), actions=actions, chunk=chunk,
                         latency_ms=float(out.get("latency_ms", (time.time() - t0) * 1000)),
                         raw=out)
