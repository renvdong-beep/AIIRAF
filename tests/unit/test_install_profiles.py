# -*- coding: utf-8 -*-
"""安装形态声明的**拒绝路径**测试（AGENTS 2.8：每个功能都要有成功与拒绝路径证据）。

被测对象：config/install/install_profiles.{yaml,schema.json} 与
scripts/install_profile_check.py（只读校验入口）。

做法：把"最小事实集"（声明 + schema + pyproject.toml + deploy/iraf-runtime.env + 三个 unit 模板）
复制到临时仓库根，在副本上制造违规，再跑校验器 —— 这样既不污染仓库，又能证明每条判据真的会拦。

覆盖：
  · 成功路径：现网声明通过，且 --profile 能打印解析后的生效值（防"改了声明没生效/改错段"）
  · 边缘角色必须是 aarch64
  · 边缘角色不得安装仿真 Provider（AGENTS 1.7/6.8）
  · 组件必须在受控枚举内
  · 安全边界不得声明可合并（AGENTS 6.11）
  · 版本必须来自单一来源（不得硬编码版本）
  · 端口事实必须与 deploy/iraf-runtime.env 一致（同一事实两处即拒绝）
"""
from __future__ import annotations

import contextlib
import importlib.util
import io
import json
import shutil
import sys
import tempfile
import unittest
from pathlib import Path

import yaml

REPO = Path(__file__).resolve().parents[2]
SCRIPTS = REPO / "scripts"

_spec = importlib.util.spec_from_file_location("install_profile_check", SCRIPTS / "install_profile_check.py")
ipc = importlib.util.module_from_spec(_spec)
sys.modules["install_profile_check"] = ipc
_spec.loader.exec_module(ipc)

#: 校验器会实读的最小事实集（缺一项就会因"文件不存在"提前失败，故一并复制）。
FIXTURES = (
    "config/install/install_profiles.yaml",
    "config/install/install_profiles.schema.json",
    "pyproject.toml",
    "deploy/iraf-runtime.env",
    "deploy/sdk/units/iraf-runtime-http.service.tmpl",
    "deploy/sdk/units/iraf-runtime-grpc.service.tmpl",
    "deploy/sdk/units/iraf-control-provider.service.tmpl",
    "deploy/sdk/iraf-sdk-board.service",
)

DECLARATION_REL = "config/install/install_profiles.yaml"


class InstallProfileDeclarationTest(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.root = Path(self._tmp.name)
        for rel in FIXTURES:
            target = self.root / rel
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(REPO / rel, target)

    def tearDown(self):
        self._tmp.cleanup()

    # ---------- 工具 ----------
    def _declaration(self) -> dict:
        return yaml.safe_load((self.root / DECLARATION_REL).read_text(encoding="utf-8"))

    def _write_declaration(self, data: dict) -> None:
        (self.root / DECLARATION_REL).write_text(
            yaml.safe_dump(data, allow_unicode=True, sort_keys=False), encoding="utf-8")

    def _profile(self, data: dict, profile_id: str) -> dict:
        for item in data["profiles"]:
            if item["id"] == profile_id:
                return item
        raise AssertionError("测试夹具里找不到角色 %s" % profile_id)

    def _run_checks(self) -> list:
        """跑语义检查，返回 checks 列表（不经过命令行，便于精确断言）。"""
        declaration = ipc.load_yaml(self.root / DECLARATION_REL)
        schema = json.loads((self.root / ipc.SCHEMA).read_text(encoding="utf-8"))
        self.assertEqual(ipc.validate_schema(declaration, schema), [],
                         "夹具本身必须先是合法声明，否则负向测试无意义")
        checks, _notes = ipc.check_semantics(declaration, self.root)
        return checks

    def _failed_names(self) -> list:
        return [item["name"] for item in self._run_checks() if not item["passed"]]

    def _run_main(self, *args) -> tuple:
        """跑命令行入口，返回 (退出码, 合并后的 stdout+stderr)。
        必须合并两路：面向使用者的中文原因走 stderr（与本仓其它脚本一致），
        只抓 stdout 会把"失败原因"漏掉，测试就变成假通过。"""
        out, err = io.StringIO(), io.StringIO()
        with contextlib.redirect_stdout(out), contextlib.redirect_stderr(err):
            code = ipc.main(["--repo-root", str(self.root), *args])
        return code, out.getvalue() + err.getvalue()

    # ---------- 成功路径 ----------
    def test_current_declaration_passes(self):
        code, output = self._run_main("--json")
        self.assertEqual(code, ipc.EXIT_OK, output[:2000])
        report = json.loads(output)
        self.assertTrue(report["passed"])
        self.assertGreaterEqual(len(report["checks"]), 40)

    def test_profile_resolution_prints_effective_values(self):
        """`--profile` 必须给出解析后的生效值（本项目最常踩：改了声明却没生效/改错段）。"""
        code, output = self._run_main("--profile", "edge-intel")
        self.assertEqual(code, ipc.EXIT_OK, output[:2000])
        self.assertIn("/opt/iraf-sdk", output)
        self.assertIn("install_method", output)
        self.assertIn("wheelhouse", output)

    def test_unknown_profile_is_usage_error(self):
        code, _output = self._run_main("--profile", "does-not-exist")
        self.assertEqual(code, ipc.EXIT_USAGE)

    # ---------- 拒绝路径 ----------
    def test_edge_arch_must_be_aarch64(self):
        data = self._declaration()
        self._profile(data, "edge-motion")["target"]["arch"] = "x86_64"
        self._write_declaration(data)
        self.assertIn("edge-motion：边缘形态必须是 aarch64", self._failed_names())

    def test_real_machine_role_must_not_install_simulation_provider(self):
        """真机档（simulation_only=false）不得装仿真 Provider（AGENTS 1.7：仿真不进真机路径）。"""
        data = self._declaration()
        self._profile(data, "edge-intel")["components"].append("adapters_sim")
        self._write_declaration(data)
        self.assertIn("edge-intel：真机档未安装仿真 Provider（adapters_sim）", self._failed_names())

    def test_simulation_role_must_install_simulation_provider(self):
        """仿真档（simulation_only=true）必须装仿真 Provider —— 否则声明了仿真却没有仿真后端。"""
        data = self._declaration()
        profile = self._profile(data, "edge-intel-sim")
        profile["components"] = [item for item in profile["components"] if item != "adapters_sim"]
        self._write_declaration(data)
        self.assertIn("edge-intel-sim：仿真档已安装仿真 Provider（adapters_sim）", self._failed_names())

    def test_component_must_be_registered(self):
        data = self._declaration()
        self._profile(data, "sim-x86")["components"].append("bogus_component")
        self._write_declaration(data)
        self.assertIn("sim-x86：组件在受控枚举内", self._failed_names())

    def test_safety_boundary_merge_is_rejected_by_schema(self):
        """铁律 6.11：合并不合边界 —— schema 层就要拒绝 merge_allowed: true。"""
        data = self._declaration()
        self._profile(data, "edge-intel")["safety_boundary"]["merge_allowed"] = True
        self._write_declaration(data)
        declaration = ipc.load_yaml(self.root / DECLARATION_REL)
        schema = json.loads((self.root / ipc.SCHEMA).read_text(encoding="utf-8"))
        errors = ipc.validate_schema(declaration, schema)
        self.assertTrue(any("merge_allowed" in item for item in errors), errors)

    def test_safety_boundary_must_be_declared_for_edge_roles(self):
        data = self._declaration()
        self._profile(data, "edge-motion").pop("safety_boundary")
        self._write_declaration(data)
        self.assertIn("edge-motion：已声明安全边界不合并（铁律 6.11）", self._failed_names())

    def test_version_must_come_from_single_source(self):
        data = self._declaration()
        data["sdk"]["version_ref"] = "pyproject.toml#project.not_a_version_field"
        self._write_declaration(data)
        code, output = self._run_main("--json")
        self.assertEqual(code, ipc.EXIT_DECLARATION)
        self.assertIn("版本单一来源失效", output)

    def test_port_facts_must_match_runtime_env(self):
        """同一事实两处（声明 vs deploy/iraf-runtime.env）不一致即拒绝。"""
        data = self._declaration()
        services = self._profile(data, "single-ubuntu2204")["services"]
        services[0]["ports"] = [8766]
        self._profile(data, "single-ubuntu2204")["residue"]["ports"] = [8766, 50051]
        self._write_declaration(data)
        self.assertIn("端口：声明与 deploy/iraf-runtime.env 一致", self._failed_names())

    def test_residue_must_track_services(self):
        """残留判据必须与服务清单同源：漏一个单元 = 卸载后静默留下服务。"""
        data = self._declaration()
        self._profile(data, "single-ubuntu2204")["residue"]["units"] = []
        self._write_declaration(data)
        self.assertIn("single-ubuntu2204：residue.units 与服务清单一致", self._failed_names())

    def test_missing_unit_template_is_rejected(self):
        data = self._declaration()
        (self.root / "deploy/sdk/units/iraf-runtime-http.service.tmpl").unlink()
        self._write_declaration(data)
        names = self._failed_names()
        self.assertTrue(any("unit 模板存在" in name for name in names), names)

    def test_unit_template_must_have_tokens(self):
        """模板缺占位符 ⇒ 拒绝（否则安装器无法按声明渲染，只能写死路径）。"""
        data = self._declaration()
        target = self.root / "deploy/sdk/units/iraf-runtime-grpc.service.tmpl"
        target.write_text("[Service]\nExecStart=/opt/fixed/python -m x\n", encoding="utf-8")
        self._write_declaration(data)
        names = self._failed_names()
        self.assertTrue(any("含全部必需占位符" in name for name in names), names)

    def test_unit_template_must_not_hardcode_dev_paths(self):
        """实测踩到：开发机专用单元在**生效行**里写死开发机路径与解释器，却被当交付单元声明。"""
        data = self._declaration()
        target = self.root / "deploy/sdk/units/iraf-runtime-http.service.tmpl"
        text = target.read_text(encoding="utf-8")
        target.write_text(text + "\nExecStart=/home/coretek/miniconda3/envs/mujoco/bin/python -m x\n",
                          encoding="utf-8")
        self._write_declaration(data)
        names = self._failed_names()
        self.assertTrue(any("生效行无开发机硬编码" in name for name in names), names)

    def test_unit_template_comments_may_mention_dev_machine(self):
        """注释里提到开发机**不算违规**：只判生效行（否则模板没法写明"为什么不用那个单元"）。"""
        data = self._declaration()
        target = self.root / "deploy/sdk/units/iraf-runtime-http.service.tmpl"
        text = target.read_text(encoding="utf-8")
        target.write_text(text + "\n# 不要用开发机那个单元（User=coretek / miniconda 解释器）\n",
                          encoding="utf-8")
        self._write_declaration(data)
        names = self._failed_names()
        self.assertFalse(any("生效行无开发机硬编码" in name for name in names), names)

    def test_schema_rejects_unknown_key(self):
        data = self._declaration()
        data["profiles"][0]["not_a_declared_key"] = 1
        self._write_declaration(data)
        declaration = ipc.load_yaml(self.root / DECLARATION_REL)
        schema = json.loads((self.root / ipc.SCHEMA).read_text(encoding="utf-8"))
        errors = ipc.validate_schema(declaration, schema)
        self.assertTrue(any("not_a_declared_key" in item for item in errors), errors)


if __name__ == "__main__":
    unittest.main()
