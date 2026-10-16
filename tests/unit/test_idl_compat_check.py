# -*- coding: utf-8 -*-
"""IDL 破坏性变更检查工具的用例（铁律 2.10）。

用**合成 proto** 造成对：每个用例只改一处，断言退出码与命中的检查项种类。
不依赖 git（git 路径由 `--baseline-ref` 那条分支覆盖，属集成用法）。
缺 grpcio-tools 的环境（如系统 python3）自动 skip —— 本工具依赖它自带的 protoc。
"""
from __future__ import annotations

import importlib.util
from pathlib import Path

import pytest

pytest.importorskip("grpc_tools", reason="需要 grpcio-tools（自带 protoc）")

REPO = Path(__file__).resolve().parents[2]

_spec = importlib.util.spec_from_file_location(
    "idl_compat_check", REPO / "scripts" / "idl_compat_check.py")
idl = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(idl)

BASE = """syntax = "proto3";
package t;
message M {
  int32 a = 1;
  string b = 2;
}
enum E {
  E_UNSPECIFIED = 0;
  E_A = 1;
}
service S {
  rpc P(M) returns (M);
}
"""


def _write(root: Path, text: str, name: str = "t.proto") -> Path:
    root.mkdir(parents=True, exist_ok=True)
    (root / name).write_text(text, encoding="utf-8")
    return root


def _run(tmp_path, current_text, baseline_text=BASE, extra=()):
    base = _write(tmp_path / "base", baseline_text)
    cur = _write(tmp_path / "cur", current_text)
    return idl.main(["--baseline-dir", str(base), "--current-dir", str(cur), *extra])


# ---------- 兼容 ----------

def test_identical_is_compatible(tmp_path):
    assert _run(tmp_path, BASE) == idl.EXIT_OK


def test_added_field_is_compatible(tmp_path):
    current = BASE.replace("  string b = 2;", "  string b = 2;\n  int64 c = 3;")
    assert _run(tmp_path, current) == idl.EXIT_OK


def test_added_message_and_rpc_are_compatible(tmp_path):
    current = BASE + "\nmessage N { bool ok = 1; }\nservice T { rpc Q(N) returns (N); }\n"
    assert _run(tmp_path, current) == idl.EXIT_OK


def test_added_enum_value_is_compatible(tmp_path):
    current = BASE.replace("  E_A = 1;", "  E_A = 1;\n  E_B = 2;")
    assert _run(tmp_path, current) == idl.EXIT_OK


# ---------- 破坏性 ----------

def _breaking_kinds(tmp_path, current_text):
    out = {"breaking": []}
    assert _run(tmp_path, current_text, extra=("--json",)) == idl.EXIT_BREAKING
    import json
    import io
    import contextlib
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        idl.main(["--baseline-dir", str(tmp_path / "base"),
                  "--current-dir", str(tmp_path / "cur"), "--json"])
    return [kind for kind, _ in json.loads(buf.getvalue())["breaking"]]


def test_removed_field_is_breaking(tmp_path):
    current = BASE.replace("  string b = 2;\n", "")
    assert _run(tmp_path, current) == idl.EXIT_BREAKING


def test_field_type_change_is_breaking(tmp_path):
    current = BASE.replace("  int32 a = 1;", "  string a = 1;")
    assert _run(tmp_path, current) == idl.EXIT_BREAKING


def test_field_number_reuse_is_breaking(tmp_path):
    """编号复用：同一个 #2 换了个字段名 —— 编译期无感，JSON 名与生成代码全破坏。"""
    current = BASE.replace("  string b = 2;", "  string renamed_b = 2;")
    kinds = _breaking_kinds(tmp_path, current)
    assert "field-renamed" in kinds


def test_removed_message_is_breaking(tmp_path):
    """删 message：用**不被 service 引用**的 message，否则 protoc 先编译失败（退 3）。"""
    baseline = BASE + "\nmessage Unused { int32 x = 1; }\n"
    assert _run(tmp_path, BASE, baseline_text=baseline) == idl.EXIT_BREAKING


def test_removed_rpc_is_breaking(tmp_path):
    current = BASE.replace("  rpc P(M) returns (M);\n", "")
    assert _run(tmp_path, current) == idl.EXIT_BREAKING


def test_rpc_streaming_change_is_breaking(tmp_path):
    current = BASE.replace("rpc P(M) returns (M);", "rpc P(M) returns (stream M);")
    kinds = _breaking_kinds(tmp_path, current)
    assert "rpc-signature-changed" in kinds


def test_removed_enum_value_is_breaking(tmp_path):
    current = BASE.replace("  E_A = 1;\n", "")
    assert _run(tmp_path, current) == idl.EXIT_BREAKING


def test_enum_value_renamed_is_breaking(tmp_path):
    current = BASE.replace("  E_A = 1;", "  E_RENAMED = 1;")
    kinds = _breaking_kinds(tmp_path, current)
    assert "enum-value-renamed" in kinds


# ---------- 用法与环境 ----------

def test_missing_both_baseline_args_is_usage_error(tmp_path):
    cur = _write(tmp_path / "cur", BASE)
    assert idl.main(["--current-dir", str(cur)]) == idl.EXIT_USAGE


def test_nonexistent_current_dir_is_usage_error(tmp_path):
    base = _write(tmp_path / "base", BASE)
    assert idl.main(["--baseline-dir", str(base),
                     "--current-dir", str(tmp_path / "nope")]) == idl.EXIT_USAGE


def test_real_repo_against_head_is_clean(tmp_path):
    """真实仓库：工作区 api/proto 相对 HEAD 必须无破坏性变更（提交前自检用）。"""
    assert idl.main(["--baseline-ref", "HEAD"]) in (idl.EXIT_OK, idl.EXIT_USAGE)
