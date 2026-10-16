#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""**离线意图模型桩**（开发/演示用；不是真 LLM）。

用途：在没有可用意图模型端点时，把"AgentOS 语义 → IRAF skill"这条链**接线**跑通，
让 `scripts/agentos_cli.py --text "…"` / `scripts/agentos_view.py --text "…"` 能在本机离线演示。
它只实现最小可用的**受约束解码**：从请求体里拿到 IRaf Registry 传来的 Skill 契约（含 examples），
按关键词/示例匹配挑一个 skill，再产出严格符合该 skill inputSchema 的 parameters。

硬边界（不得含糊）
  · 这是**桩**：输出与真实 LLM 无关，日志/报告里必须标明"桩，不是真 LLM"；
  · 它**不编造执行参数**：只填 schema 必需的、语义上确实可知的字段（如 visual_pick 的 target_id）；
    需要位姿/几何的参数（如 display_pick 的 grasp_pose）它**不去猜**，改为选一个参数面更小的等价 skill；
  · 匹配不到就返回一个**明确不合法**的 skill 名，让上层照常走 `IRAF-INTENT-PARSE-FAILED` 拒绝路径
    （禁止"兜底猜一个"）。

用法：
    python3 scripts/dev_stub_intent_model.py [--port 8077] [--host 127.0.0.1]
    export IRAF_AGENTOS_MODEL_URL=http://127.0.0.1:8077/v1
    export IRAF_AGENTOS_MODEL=stub-intent
    python3 scripts/agentos_view.py --text "启动piper机械臂进行抓取仿真"
"""
from __future__ import annotations

import argparse
import json
import re
from http.server import BaseHTTPRequestHandler, HTTPServer

#: 关键词 → skill（顺序敏感：先具体后笼统）。只用"语义上可知"的参数面最小的 skill：
#: 抓取类选 visual_pick（schema 只需 target_id），**不选** display_pick（它要求 grasp_pose，
#: 桩无法在不编造的前提下给出；显示通道会自行用 display_pick 执行同一条闭环）。
KEYWORDS = (
    ("抓取", "visual_pick"), ("拾取", "visual_pick"), ("夹取", "visual_pick"),
    ("pick", "visual_pick"), ("grasp", "visual_pick"),
    ("放", "place_object"), ("place", "place_object"),
    ("站起来", "stand"), ("站立", "stand"), ("stand", "stand"),
    ("停", "stop"), ("stop", "stop"),
    ("走", "locomote"), ("移动", "locomote"), ("locomote", "locomote"),
    ("关节", "move_joint"), ("move joint", "move_joint"),
    ("标定夹爪", "calibrate_grasp"), ("标定相机", "calibrate_camera_to_base"),
)


def pick_skill(text: str, contracts: dict) -> str:
    """按关键词挑 skill；挑不到返回空串（上层照常拒绝，不兜底猜）。"""
    lowered = text.lower()
    for token, name in KEYWORDS:
        if token.lower() in lowered and name in contracts:
            return name
    return ""


def build_parameters(skill: str, contracts: dict, text: str) -> dict:
    """只填 schema 必需的、可从语义得知的字段（其余留给默认/上层装配）。"""
    schema = (contracts.get(skill) or {}).get("inputSchema") or {}
    required = list(schema.get("required") or [])
    params = {}
    if "target_id" in required:
        match = re.search(r"(box_[A-Za-z0-9_]+)", text)
        params["target_id"] = match.group(1) if match else "box_01"
    # 需要位姿/几何等桩无法得知的必填字段 ⇒ 明确失败（不编造），让上层拒绝
    unknown_required = [key for key in required if key not in params]
    if unknown_required:
        return {"__stub_refuses__": unknown_required}
    return params


def build_response(body: dict) -> dict:
    messages = body.get("messages") or []
    text = ""
    contracts = {}
    for item in messages:
        if item.get("role") == "system":
            raw = str(item.get("content") or "")
            marker = "Skill契约："
            if marker in raw:
                try:
                    contracts = json.loads(raw.split(marker, 1)[1])
                except json.JSONDecodeError:
                    contracts = {}
        elif item.get("role") == "user":
            text = str(item.get("content") or "")
    skill = pick_skill(text, contracts)
    if not skill:
        # 不兜底：给一个不在契约里的名字，上层会按 schema 校验失败 → IRAF-INTENT-PARSE-FAILED
        intent = {"skill": "__stub_no_match__", "parameters": {}}
    else:
        params = build_parameters(skill, contracts, text)
        if "__stub_refuses__" in params:
            intent = {"skill": "__stub_refuses__", "parameters": params}
        else:
            intent = {"skill": skill, "parameters": params}
    payload = {
        "id": "chatcmpl-stub",
        "object": "chat.completion",
        "model": body.get("model") or "stub-intent",
        "choices": [{"index": 0, "finish_reason": "stop",
                     "message": {"role": "assistant", "content": json.dumps(intent, ensure_ascii=False)}}],
        "usage": {"prompt_tokens": 0, "completion_tokens": 0, "total_tokens": 0},
        "stub_note": "本响应来自离线桩（dev_stub_intent_model.py），不是真实 LLM 输出",
    }
    return payload


class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def log_message(self, fmt, *args):  # 静默：只保留业务输出
        return

    def _send(self, code: int, payload: dict) -> None:
        data = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):  # noqa: N802 - BaseHTTPRequestHandler 约定
        if self.path.rstrip("/").endswith("/models"):
            self._send(200, {"object": "list", "data": [{"id": "stub-intent", "object": "model"}]})
        else:
            self._send(404, {"error": {"message": "桩只实现 /v1/models 与 /v1/chat/completions"}})

    def do_POST(self):  # noqa: N802
        if not self.path.rstrip("/").endswith("/chat/completions"):
            self._send(404, {"error": {"message": "桩只实现 /v1/chat/completions"}})
            return
        length = int(self.headers.get("Content-Length") or 0)
        try:
            body = json.loads(self.rfile.read(length).decode("utf-8") or "{}")
        except json.JSONDecodeError:
            self._send(400, {"error": {"message": "请求体不是 JSON"}})
            return
        self._send(200, build_response(body))


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="离线意图模型桩（开发/演示用，不是真 LLM）")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8077)
    args = parser.parse_args(argv)
    server = HTTPServer((args.host, args.port), Handler)
    print("[stub-intent] 监听 http://%s:%d/v1（离线桩，不是真 LLM）" % (args.host, args.port), flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
