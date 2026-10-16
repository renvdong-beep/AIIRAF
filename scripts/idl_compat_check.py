#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""IDL 破坏性变更检查（铁律 2.10：合并前必须通过 IDL 兼容检查）。

为什么需要（2026-10-16 补）：`api/proto/` 是管理与 SDK 的**唯一事实来源**（铁律 2.2），
但仓库此前只有 `scripts/gen_proto_stubs.sh`（**生成**桩），没有**兼容性**校验 ——
即"改了 proto 是否破坏已在场的客户端/服务端"没有人管。
典型事故：把 message 的字段编号复用给另一个字段、把 int32 改成 string、删掉一个 rpc。
这些在编译期全都通过，只在运行期变成"对方读到垃圾值"或"调用不存在的方法"。

检查项（对照 proto3 wire 兼容规则）
-----------------------------------
破坏性（退出码 2）：
  · message / enum / service / rpc 被删除
  · 字段编号被删除（删字段必须 `reserved`，否则该编号会被后来者复用）
  · 同一编号的字段**换了类型**（含 label 从 optional 变 repeated）
  · 同一编号的字段**换了名字**（wire 兼容但 JSON 名与生成代码全部破坏，按本项目口径算破坏）
  · enum 值被删除，或同一编号换了名字
非破坏性（退出码 0，仅提示）：
  · 新增 message / enum / service / rpc
  · 新增字段编号（未占用过的新号）与新增 enum 值

用法::

    # 比对"上一个提交"与"当前工作区"（改动 proto 后、提交前跑）
    PYTHONPATH=src build/mcp-venv/bin/python scripts/idl_compat_check.py --baseline-ref HEAD

    # 比对两个目录（CI / 单测用，不需要 git）
    python3 scripts/idl_compat_check.py --baseline-dir /path/a --current-dir /path/b

退出码：`0` 兼容（或基线不存在 proto）｜`1` 用法错误｜`2` 存在破坏性变更｜`3` 环境缺失（缺 grpcio-tools）
"""
from __future__ import annotations

import argparse
import subprocess
import sys
import tempfile
from pathlib import Path

EXIT_OK, EXIT_USAGE, EXIT_BREAKING, EXIT_ENV = 0, 1, 2, 3

REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_PROTO_SUBDIR = "api/proto"
DEFAULT_BASELINE_REF = "HEAD"


def _descriptor_set(proto_root: Path, out_path: Path) -> None:
    """用 grpcio-tools 自带的 protoc 生成 FileDescriptorSet（不依赖系统 protoc）。"""
    import grpc_tools  # noqa: F401  延迟导入：缺它时给出可操作提示
    from grpc_tools import protoc

    well_known = Path(protoc.__file__).resolve().parent / "_proto"
    files = sorted(str(path) for path in proto_root.rglob("*.proto"))
    if not files:
        raise FileNotFoundError("目录里没有 .proto：%s" % proto_root)
    argv = ["protoc", "-I", str(proto_root)]
    if well_known.is_dir():
        argv += ["-I", str(well_known)]
    argv += ["--descriptor_set_out", str(out_path), "--include_imports", *files]
    rc = protoc.main(argv)
    if rc != 0:
        raise RuntimeError("protoc 编译失败（rc=%d）：检查 proto 语法与 import 路径" % rc)
    if not out_path.is_file():
        raise RuntimeError("protoc 未产出 descriptor set：%s" % out_path)


def snapshot(descriptor_set_path: Path) -> dict:
    """把 FileDescriptorSet 压成可比较的快照（含嵌套 message 与 enum）。"""
    from google.protobuf import descriptor_pb2

    fds = descriptor_pb2.FileDescriptorSet()
    fds.ParseFromString(descriptor_set_path.read_bytes())

    snap = {"messages": {}, "enums": {}, "services": {}}

    def walk_message(prefix: str, message) -> None:
        full = prefix + "." + message.name
        snap["messages"][full] = {
            int(field.number): {
                "name": field.name,
                "type": int(field.type),
                "label": int(field.label),
            }
            for field in message.field
        }
        for enum in message.enum_type:
            snap["enums"][full + "." + enum.name] = {
                int(value.number): value.name for value in enum.value
            }
        for nested in message.nested_type:
            walk_message(full, nested)

    for file in fds.file:
        prefix = file.package
        for message in file.message_type:
            walk_message(prefix, message)
        for enum in file.enum_type:
            snap["enums"][prefix + "." + enum.name] = {
                int(value.number): value.name for value in enum.value
            }
        for service in file.service:
            snap["services"][prefix + "." + service.name] = {
                method.name: {
                    "input": method.input_type,
                    "output": method.output_type,
                    "server_streaming": bool(method.server_streaming),
                    "client_streaming": bool(method.client_streaming),
                }
                for method in service.method
            }
    return snap


def compare(baseline: dict, current: dict) -> tuple:
    """返回 `(breaking, compatible)`，每项是 (kind, detail) 的人类可读说明。"""
    breaking, compatible = [], []

    # ---- message 与字段 ----
    for name, fields in sorted(baseline["messages"].items()):
        if name not in current["messages"]:
            breaking.append(("message-removed", name))
            continue
        current_fields = current["messages"][name]
        for number, spec in sorted(fields.items()):
            target = "%s#%d(%s)" % (name, number, spec["name"])
            if number not in current_fields:
                breaking.append(("field-removed",
                                 "%s —— 删字段必须写 `reserved %d;`，否则编号会被后来者复用" % (target, number)))
                continue
            now = current_fields[number]
            if now["type"] != spec["type"]:
                breaking.append(("field-type-changed",
                                 "%s: type %d → %d（wire 不兼容）" % (target, spec["type"], now["type"])))
            elif now["label"] != spec["label"]:
                breaking.append(("field-label-changed",
                                 "%s: label %d → %d（repeated/optional 语义变化）"
                                 % (target, spec["label"], now["label"])))
            elif now["name"] != spec["name"]:
                breaking.append(("field-renamed",
                                 "%s → #%d(%s)（JSON 名与生成代码全破坏）" % (target, number, now["name"])))
        for number, spec in sorted(current_fields.items()):
            if number not in fields:
                compatible.append(("field-added", "%s#%d(%s)" % (name, number, spec["name"])))
    for name in sorted(current["messages"]):
        if name not in baseline["messages"]:
            compatible.append(("message-added", name))

    # ---- enum ----
    for name, values in sorted(baseline["enums"].items()):
        if name not in current["enums"]:
            breaking.append(("enum-removed", name))
            continue
        now_values = current["enums"][name]
        for number, value_name in sorted(values.items()):
            target = "%s=%d(%s)" % (name, number, value_name)
            if number not in now_values:
                breaking.append(("enum-value-removed",
                                 "%s —— 删 enum 值必须 `reserved %d;`" % (target, number)))
            elif now_values[number] != value_name:
                breaking.append(("enum-value-renamed",
                                 "%s → =%d(%s)" % (target, number, now_values[number])))
        for number, value_name in sorted(now_values.items()):
            if number not in values:
                compatible.append(("enum-value-added", "%s=%d(%s)" % (name, number, value_name)))

    # ---- service 与 rpc ----
    for name, methods in sorted(baseline["services"].items()):
        if name not in current["services"]:
            breaking.append(("service-removed", name))
            continue
        now_methods = current["services"][name]
        for method_name, spec in sorted(methods.items()):
            target = "%s.%s" % (name, method_name)
            if method_name not in now_methods:
                breaking.append(("rpc-removed", target))
                continue
            now = now_methods[method_name]
            for key in ("input", "output", "server_streaming", "client_streaming"):
                if now[key] != spec[key]:
                    breaking.append(("rpc-signature-changed",
                                     "%s: %s %r → %r" % (target, key, spec[key], now[key])))
        for method_name in sorted(now_methods):
            if method_name not in methods:
                compatible.append(("rpc-added", "%s.%s" % (name, method_name)))
    for name in sorted(current["services"]):
        if name not in baseline["services"]:
            compatible.append(("service-added", name))

    return breaking, compatible


def _export_baseline_from_git(ref: str, subdir: str, dest: Path) -> bool:
    """把某个 git ref 里的 proto 目录导出到 dest；该 ref 里没有 proto 时返回 False。"""
    listing = subprocess.run(["git", "ls-tree", "-r", "--name-only", ref, subdir],
                             cwd=str(REPO_ROOT), capture_output=True, text=True)
    if listing.returncode != 0:
        raise RuntimeError("git ls-tree 失败（ref=%s）：%s" % (ref, listing.stderr.strip()))
    names = [line.strip() for line in listing.stdout.splitlines() if line.strip()]
    if not names:
        return False
    relative_root = Path(subdir)
    for name in names:
        blob = subprocess.run(["git", "show", "%s:%s" % (ref, name)],
                              cwd=str(REPO_ROOT), capture_output=True)
        if blob.returncode != 0:
            raise RuntimeError("git show %s:%s 失败" % (ref, name))
        target = dest / Path(name).relative_to(relative_root)
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(blob.stdout)
    return True


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="IDL 破坏性变更检查（proto3 wire 兼容）")
    parser.add_argument("--baseline-ref", default=None,
                        help="基线 git ref（缺省 %s）；该 ref 无 proto 则视为首个版本，直接通过" % DEFAULT_BASELINE_REF)
    parser.add_argument("--baseline-dir", type=Path, default=None, help="基线 proto 目录（与 --baseline-ref 二选一）")
    parser.add_argument("--current-dir", type=Path, default=None,
                        help="当前 proto 目录（缺省仓库的 %s）" % DEFAULT_PROTO_SUBDIR)
    parser.add_argument("--proto-subdir", default=DEFAULT_PROTO_SUBDIR, help="仓库内的 proto 相对路径")
    parser.add_argument("--json", action="store_true", help="以 JSON 输出")
    args = parser.parse_args(argv)

    current_dir = (args.current_dir or (REPO_ROOT / args.proto_subdir)).resolve()
    if not current_dir.is_dir():
        print("[idl-compat][用法错误] 当前 proto 目录不存在：%s" % current_dir, file=sys.stderr)
        return EXIT_USAGE
    if args.baseline_dir and args.baseline_ref:
        print("[idl-compat][用法错误] --baseline-dir 与 --baseline-ref 只能给一个", file=sys.stderr)
        return EXIT_USAGE

    try:
        import grpc_tools  # noqa: F401
        from google.protobuf import descriptor_pb2  # noqa: F401
    except ImportError as exc:
        print("[idl-compat][环境缺失] 需要 grpcio-tools（含 protoc）与 protobuf：%s\n"
              "  安装：pip install grpcio-tools（本仓库可用 build/mcp-venv/bin/python 跑本脚本）"
              % exc, file=sys.stderr)
        return EXIT_ENV

    with tempfile.TemporaryDirectory(prefix="iraf-idl-compat-") as tmpdir:
        tmp = Path(tmpdir)
        baseline_dir = args.baseline_dir
        if args.baseline_ref:
            exported = tmp / "baseline"
            try:
                if not _export_baseline_from_git(args.baseline_ref, args.proto_subdir, exported):
                    print("[idl-compat] 基线 %s 内没有 %s ⇒ 视为首个版本，跳过检查"
                          % (args.baseline_ref, args.proto_subdir))
                    return EXIT_OK
            except RuntimeError as exc:
                print("[idl-compat][用法错误] %s" % exc, file=sys.stderr)
                return EXIT_USAGE
            baseline_dir = exported
        if baseline_dir is None:
            print("[idl-compat][用法错误] 需要 --baseline-ref 或 --baseline-dir", file=sys.stderr)
            return EXIT_USAGE
        if not Path(baseline_dir).is_dir():
            print("[idl-compat][用法错误] 基线目录不存在：%s" % baseline_dir, file=sys.stderr)
            return EXIT_USAGE

        try:
            base_set, cur_set = tmp / "base.desc", tmp / "cur.desc"
            _descriptor_set(Path(baseline_dir), base_set)
            _descriptor_set(current_dir, cur_set)
            breaking, compatible = compare(snapshot(base_set), snapshot(cur_set))
        except Exception as exc:  # noqa: BLE001 显式失败：检查工具本身出错不得当成"兼容"
            print("[idl-compat][错误] 生成/解析 descriptor 失败：%s" % exc, file=sys.stderr)
            return EXIT_ENV

    if args.json:
        import json
        print(json.dumps({"breaking": breaking, "compatible": compatible},
                         ensure_ascii=False, indent=2))
    else:
        for kind, detail in compatible:
            print("  · 兼容新增 [%s] %s" % (kind, detail))
        for kind, detail in breaking:
            print("  ✗ 破坏性变更 [%s] %s" % (kind, detail))
        if breaking:
            print("[idl-compat] 发现 %d 项破坏性变更：契约兼容性未通过（铁律 2.10）" % len(breaking))
        else:
            print("[idl-compat] 通过：无破坏性变更（兼容新增 %d 项）" % len(compatible))

    return EXIT_BREAKING if breaking else EXIT_OK


if __name__ == "__main__":
    raise SystemExit(main())
