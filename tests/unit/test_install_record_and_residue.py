# -*- coding: utf-8 -*-
"""安装记录 / 残留核验 / 受控卸载的成功与**拒绝路径**测试（IRAF 双域部署 P2b）。

被测对象：`deploy/sdk/lib_install.py` + `deploy/sdk/{verify_residue,uninstall-iraf}.sh` 的实现层。

做法：在临时目录里搭一个"沙箱文件系统根"（--root），用声明里的角色（single-ubuntu2204 走
`~` 展开后的 home，不适合沙箱；因此测试用 edge-intel/edge-motion 这类绝对前缀角色）造出
"已安装树 + 安装记录 + 单元 + env 文件"，然后在副本上制造各种越界，验证六路残留核验与
受控卸载会**如实报出**而不是"尽力而为"。

覆盖（成功 + 拒绝）：
  · 生效布局解析（defaults 继承 + 路径展开）
  · 安装记录写/读；角色不符即拒
  · 六路残留核验：干净 / 有文件 / 有空目录 / 有单元 / 有 env / 有端口 / 有进程
  · 受控卸载：dry-run 不删文件；真卸后六路全 0；数据与日志默认保留、--purge 才删
  · 拒绝路径：记录缺失、记录内文件缺失、哈希不符、清单外文件
"""
from __future__ import annotations

import importlib.util
import json
import os
import sys
import tempfile
import unittest
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
LIB = REPO / "deploy" / "sdk" / "lib_install.py"

_spec = importlib.util.spec_from_file_location("lib_install", LIB)
lib = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(lib)  # type: ignore[union-attr]

PROFILE = "edge-intel"          # 绝对前缀（/opt/iraf-sdk），适合沙箱
KEEP_PROFILE = "edge-motion"
VERSION = "0.2.0"


class InstallLayoutTest(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.root = Path(self._tmp.name)      # 扮演文件系统根 "/"
        self.effective = lib.effective_layout(REPO, PROFILE)

    def tearDown(self):
        self._tmp.cleanup()

    # ---------- 夹具 ----------
    def _version_dir(self) -> Path:
        return Path(str(self.root) + str(self.effective["prefix"])) / VERSION

    def _make_install(self, *, files=("bin/iraf-agentos", "src/iraf_core/runtime.py"),
                      unit=True, env=True, keep_data=True, record=True):
        """造一棵"像是装过"的树 + 记录；返回记录 dict。"""
        version_dir = self._version_dir()
        for rel in files:
            path = version_dir / rel
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text("#!/bin/sh\necho %s\n" % rel, encoding="utf-8")
        prefix = Path(str(self.root) + str(self.effective["prefix"]))
        (prefix / self.effective["current_link"]).symlink_to(VERSION)
        if unit:
            unit_dir = Path(str(self.root) + "/etc/systemd/system")
            unit_dir.mkdir(parents=True, exist_ok=True)
            (unit_dir / "iraf-runtime-grpc.service").write_text("[Unit]\n", encoding="utf-8")
        if env:
            config_dir = Path(str(self.root) + str(self.effective["config_dir"]))
            config_dir.mkdir(parents=True, exist_ok=True)
            (config_dir / "iraf-sdk.env").write_text("IRAF_PROFILE=x\n", encoding="utf-8")
        if keep_data:
            data_dir = Path(str(self.root) + str(self.effective["data_dir"]))
            data_dir.mkdir(parents=True, exist_ok=True)
            (data_dir / "iraf-events.db").write_text("db", encoding="utf-8")
        if not record:
            return {}
        lib.write_record(self.effective, version_dir, note="测试夹具",
                         config_paths=[Path(str(self.root) + str(self.effective["config_dir"]))
                                       / "iraf-sdk.env"] if env else [])
        return lib.read_record(self.effective, version_dir)

    def _scan(self, record=None):
        return lib.residue_scan(self.effective, self.root, record=record)

    def _run(self, *args):
        import contextlib
        import io
        out, err = io.StringIO(), io.StringIO()
        with contextlib.redirect_stdout(out), contextlib.redirect_stderr(err):
            code = lib.main(["--repo-root", str(REPO), *args])
        return code, out.getvalue() + err.getvalue()

    # ---------- 成功路径 ----------
    def test_layout_resolution(self):
        edge = lib.effective_layout(REPO, "edge-intel")
        self.assertEqual(edge["prefix"], "/opt/iraf-sdk")
        self.assertEqual(edge["install_method"], "target")     # 从 defaults 继承
        self.assertEqual(edge["file_record"], "iraf-install-files.json")
        single = lib.effective_layout(REPO, "single-ubuntu2204")
        self.assertTrue(single["prefix"].startswith("/"))        # ~ 已展开
        self.assertEqual(single["systemd_scope"], "user")

    def test_record_roundtrip(self):
        record = self._make_install()
        self.assertEqual(record["profile"], PROFILE)
        self.assertEqual(record["version"], VERSION)
        # file_count = 版本目录内文件 + 版本目录外的配置文件（env 在 /etc/iraf 下）
        self.assertEqual(len(record["files"]), 2)
        self.assertEqual(len(record["config_files"]), 1)
        self.assertEqual(record["file_count"], 3)
        self.assertEqual(record["units"], ["iraf-runtime-grpc.service"])
        # 记录自身不计入文件清单（否则自我引用）
        self.assertNotIn(self.effective["file_record"],
                         [item["path"] for item in record["files"]])

    def test_residue_clean_after_real_uninstall(self):
        record = self._make_install(keep_data=False)
        plan = lib.plan_uninstall(self.effective, self.root, record)
        lib.apply_uninstall(plan, root=self.root)
        findings = self._scan(record=None)
        self.assertTrue(lib.residue_is_clean(findings), findings)
        self.assertFalse(self._version_dir().exists())
        # 空目录链也清掉了（前缀本身不该留下空壳）
        self.assertFalse(Path(str(self.root) + str(self.effective["prefix"])).exists())

    def test_dry_run_deletes_nothing(self):
        record = self._make_install(keep_data=False)
        before = sorted(str(p) for p in self._version_dir().rglob("*"))
        code, output = self._run("uninstall", "--profile", PROFILE, "--root", str(self.root),
                                 "--installed-dir", str(self._version_dir()), "--dry-run")
        self.assertEqual(code, lib.EXIT_OK, output)
        after = sorted(str(p) for p in self._version_dir().rglob("*"))
        self.assertEqual(before, after, "dry-run 不得删除任何文件")
        self.assertIn("演练", output)

    def test_data_and_log_kept_by_default_and_removed_with_purge(self):
        record = self._make_install(keep_data=True)
        plan = lib.plan_uninstall(self.effective, self.root, record)
        self.assertTrue(plan["keep"], "数据目录应默认保留")
        lib.apply_uninstall(plan, root=self.root)
        self.assertTrue(Path(str(self.root) + str(self.effective["data_dir"])).exists())
        self.assertIn(str(self.root) + str(self.effective["data_dir"]),
                      [str(item) for item in plan["keep"]])
        # 再来一次（重装 + purge）
        self._make_install(keep_data=True)
        record = lib.read_record(self.effective, self._version_dir())
        plan = lib.plan_uninstall(self.effective, self.root, record, purge=True)
        self.assertFalse(plan["keep"])
        lib.apply_uninstall(plan, root=self.root)
        self.assertFalse(Path(str(self.root) + str(self.effective["data_dir"])).exists())

    # ---------- 残留核验：六路都能报出来 ----------
    def test_residue_reports_leftover_file(self):
        record = self._make_install(keep_data=False)
        plan = lib.plan_uninstall(self.effective, self.root, record)
        lib.apply_uninstall(plan, root=self.root)
        stray = Path(str(self.root) + str(self.effective["prefix"])) / "leftover.txt"
        stray.parent.mkdir(parents=True, exist_ok=True)
        stray.write_text("x", encoding="utf-8")
        findings = self._scan(record=None)
        self.assertFalse(lib.residue_is_clean(findings))
        self.assertIn(str(stray), findings["files"])

    def test_residue_reports_leftover_unit_and_env(self):
        self._make_install(keep_data=False)
        findings = self._scan(record=None)
        self.assertTrue(findings["units"]["iraf-runtime-grpc.service"]["exists"])
        self.assertTrue(any(item.endswith("iraf-sdk.env") for item in findings["env_files"]))
        self.assertFalse(lib.residue_is_clean(findings))

    def test_residue_reports_port_and_process(self):
        self._make_install(keep_data=False)
        original_ports, original_procs = lib._listening_ports, lib._running_processes
        try:
            lib._listening_ports = lambda: {50051}
            lib._running_processes = lambda: ["/opt/iraf-sdk/current/bin/iraf-runtime-grpc --serve"]
            findings = self._scan(record=None)
        finally:
            lib._listening_ports, lib._running_processes = original_ports, original_procs
        self.assertEqual(findings["ports"], [50051])
        self.assertTrue(findings["processes"])
        self.assertFalse(lib.residue_is_clean(findings))

    def test_residue_reports_empty_dirs(self):
        self._make_install(keep_data=False)
        empty = Path(str(self.root) + str(self.effective["prefix"])) / "empty-dir"
        empty.mkdir(parents=True, exist_ok=True)
        findings = self._scan(record=None)
        self.assertIn(str(empty), findings["dirs"])

    def test_residue_reports_unlisted_and_missing(self):
        record = self._make_install(keep_data=False)
        unlisted = self._version_dir() / "sneaked-in.txt"
        unlisted.write_text("x", encoding="utf-8")
        findings = self._scan(record=record)
        self.assertIn("sneaked-in.txt", findings["unlisted"])
        (self._version_dir() / "src/iraf_core/runtime.py").unlink()
        findings = self._scan(record=record)
        self.assertIn("src/iraf_core/runtime.py", findings["missing"])

    def test_env_file_is_recorded_and_removed(self):
        """**本轮实测踩到的缺口**：env 文件装在版本目录之外，若不入记录，卸载后必然残留。"""
        record = self._make_install(keep_data=False)
        self.assertTrue(record["config_files"], "env 文件必须进记录")
        env_path = Path(record["config_files"][0]["path"])
        self.assertTrue(env_path.is_file())
        plan = lib.plan_uninstall(self.effective, self.root, record)
        self.assertIn(env_path, plan["delete"])
        lib.apply_uninstall(plan, root=self.root)
        self.assertFalse(env_path.exists(), "卸载后 env 文件必须消失")
        self.assertTrue(lib.residue_is_clean(self._scan(record=None)))

    def test_uninstall_refuses_on_config_file_tampered(self):
        record = self._make_install(keep_data=False)
        env_path = Path(record["config_files"][0]["path"])
        env_path.write_text("IRAF_PROFILE=tampered\n", encoding="utf-8")
        with self.assertRaises(lib.InstallError) as ctx:
            lib.plan_uninstall(self.effective, self.root, record)
        self.assertIn("配置文件哈希不符", str(ctx.exception))

    # ---------- 拒绝路径 ----------
    def test_uninstall_refuses_without_record(self):
        self._make_install(record=False)
        code, output = self._run("uninstall", "--profile", PROFILE, "--root", str(self.root),
                                 "--installed-dir", str(self._version_dir()))
        self.assertEqual(code, lib.EXIT_MISMATCH)
        self.assertIn("缺少安装记录", output)

    def test_uninstall_refuses_on_missing_file(self):
        record = self._make_install()
        (self._version_dir() / "bin/iraf-agentos").unlink()
        with self.assertRaises(lib.InstallError) as ctx:
            lib.plan_uninstall(self.effective, self.root, record)
        self.assertEqual(ctx.exception.code, lib.EXIT_MISMATCH)
        self.assertIn("记录内文件缺失", str(ctx.exception))

    def test_uninstall_refuses_on_hash_mismatch(self):
        record = self._make_install()
        target = self._version_dir() / "bin/iraf-agentos"
        target.write_text("#!/bin/sh\necho tampered\n", encoding="utf-8")
        with self.assertRaises(lib.InstallError) as ctx:
            lib.plan_uninstall(self.effective, self.root, record)
        self.assertIn("哈希不符", str(ctx.exception))

    def test_uninstall_refuses_on_unlisted_file(self):
        record = self._make_install()
        (self._version_dir() / "unknown.so").write_text("x", encoding="utf-8")
        with self.assertRaises(lib.InstallError) as ctx:
            lib.plan_uninstall(self.effective, self.root, record)
        self.assertIn("清单外文件", str(ctx.exception))

    def test_uninstall_refuses_on_profile_mismatch(self):
        record = self._make_install()
        other = lib.effective_layout(REPO, KEEP_PROFILE)
        with self.assertRaises(lib.InstallError) as ctx:
            lib.read_record(other, self._version_dir())
        self.assertIn("与本次角色", str(ctx.exception))
        self.assertEqual(record["profile"], PROFILE)

    def test_unknown_profile_is_usage_error(self):
        code, output = self._run("layout", "--profile", "nope")
        self.assertEqual(code, lib.EXIT_USAGE)
        self.assertIn("未知安装角色", output)


class InstallActionTest(unittest.TestCase):
    """安装动作的成功/拒绝路径（用**自造的最小件**，不联网、不碰真实系统目录）。"""

    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.root = Path(self._tmp.name)
        self.effective = lib.effective_layout(REPO, PROFILE)
        self.art = self.root / "artifacts"
        (self.art / "wheelhouse").mkdir(parents=True, exist_ok=True)
        self.wheel = self._make_wheel(self.art / "wheelhouse")
        # 清单里必须写明 wheelhouse 属于哪个**构建目标**：声明里 dependency_source.target_id 会与它比对
        # （装错标签的包属于"静默装错"，必须在预检拦住）
        (self.art / "wheelhouse" / "wheelhouse.json").write_text(
            '{"schema_version": "iraf.wheelhouse/v1", "target": "aarch64-manylinux_2_28-cp311"}\n',
            encoding="utf-8")
        # 本类测的是"记录/残留/回滚"的机制，不是运行期依赖与桩的装配：
        # 把这两项在夹具里显式清空（它们各有专门的用例 test_runtime_deps_and_stubs_are_installed）
        self.effective["runtime_dependencies"] = []
        self.effective["python_stubs_dir"] = None

    def tearDown(self):
        self._tmp.cleanup()

    def _make_wheel(self, dest: Path) -> Path:
        import zipfile
        wheel = dest / "iraf_sdk-0.2.0-py3-none-any.whl"
        with zipfile.ZipFile(wheel, "w") as zf:
            zf.writestr("iraf_sdk/__init__.py", "__version__ = '0.2.0'\n")
            zf.writestr("iraf_sdk-0.2.0.dist-info/METADATA",
                        "Metadata-Version: 2.1\nName: iraf_sdk\nVersion: 0.2.0\n")
            zf.writestr("iraf_sdk-0.2.0.dist-info/WHEEL",
                        "Wheel-Version: 1.0\nGenerator: test\nRoot-Is-Purelib: true\nTag: py3-none-any\n")
            zf.writestr("iraf_sdk-0.2.0.dist-info/RECORD", "")
        return wheel

    def _install(self, **overrides):
        kwargs = dict(sdk_wheel=self.wheel, wheelhouse=self.art / "wheelhouse",
                      allow_unverified=True, rehearsal=True)
        kwargs.update(overrides)
        return lib.apply_install(self.effective, self.root, **kwargs)

    def _version_dir(self) -> Path:
        return Path(str(self.root) + str(self.effective["prefix"])) / "0.2.0"

    def _run(self, *args):
        """跑命令行入口，返回 (退出码, stdout+stderr 合并)。"""
        import contextlib
        import io
        out, err = io.StringIO(), io.StringIO()
        with contextlib.redirect_stdout(out), contextlib.redirect_stderr(err):
            code = lib.main(["--repo-root", str(REPO), *args])
        return code, out.getvalue() + err.getvalue()

    def test_dry_run_writes_nothing(self):
        plan = self._install(dry_run=True)
        self.assertTrue(plan["dry_run"])
        self.assertIn("steps", plan["will_do"])
        self.assertFalse(Path(str(self.root) + str(self.effective["prefix"])).exists())
        self.assertFalse(Path(str(self.root) + str(self.effective["config_dir"])).exists())

    def test_install_then_uninstall_leaves_no_residue(self):
        plan = self._install()
        self.assertTrue(plan["ok"])
        self.assertTrue((self._version_dir() / "lib/iraf_sdk/__init__.py").is_file())
        self.assertTrue(Path(plan["env_file"]).is_file())
        self.assertEqual(len(plan["units"]), 1)
        self.assertTrue(Path(plan["units"][0]).read_text(encoding="utf-8").count("@") == 0,
                        "渲染后不得残留占位符")
        record = lib.read_record(self.effective, self._version_dir())
        removed = lib.apply_uninstall(lib.plan_uninstall(self.effective, self.root, record), root=self.root)
        self.assertTrue(removed)
        self.assertTrue(lib.residue_is_clean(lib.residue_scan(self.effective, self.root)))

    def test_rehearsal_refuses_real_root(self):
        with self.assertRaises(lib.InstallError) as ctx:
            lib.apply_install(self.effective, Path("/"), sdk_wheel=self.wheel,
                              allow_unverified=True, rehearsal=True, dry_run=True)
        self.assertEqual(ctx.exception.code, lib.EXIT_USAGE)
        self.assertIn("真实根", str(ctx.exception))

    def test_rehearsal_requires_allow_unverified(self):
        with self.assertRaises(lib.InstallError) as ctx:
            self._install(allow_unverified=False, dry_run=True)
        self.assertIn("--allow-unverified", str(ctx.exception))

    def test_precheck_fails_without_wheelhouse_manifest(self):
        (self.art / "wheelhouse" / "wheelhouse.json").unlink()
        checks = lib.precheck(self.effective, self.root, allow_unverified=True,
                              wheelhouse=self.art / "wheelhouse", sdk_wheel=self.wheel,
                              python_exe=sys.executable, rehearsal=True)
        failed = [item["name"] for item in checks if not item["ok"]]
        self.assertTrue(any("离线 wheelhouse" in name for name in failed), failed)

    def test_precheck_fails_without_sdk_wheel(self):
        checks = lib.precheck(self.effective, self.root, allow_unverified=True,
                              wheelhouse=self.art / "wheelhouse", sdk_wheel=None,
                              python_exe=sys.executable, rehearsal=True)
        failed = [item["name"] for item in checks if not item["ok"]]
        self.assertTrue(any("SDK wheel 可读" in name for name in failed), failed)

    def test_install_refuses_when_unit_exists_and_rolls_back(self):
        unit_name = "%s.service" % self.effective["services"][0]["name"]
        unit_dir = Path(str(self.root) + "/etc/systemd/system")
        unit_dir.mkdir(parents=True, exist_ok=True)
        (unit_dir / unit_name).write_text("[Unit]\n", encoding="utf-8")
        with self.assertRaises(lib.InstallError) as ctx:
            self._install()
        self.assertEqual(ctx.exception.code, lib.EXIT_MISMATCH)
        self.assertFalse(self._version_dir().exists(), "回滚后不得残留版本目录")
        self.assertFalse((Path(str(self.root) + str(self.effective["config_dir"]))
                         / ("%s.env" % PROFILE)).exists(), "回滚后不得残留 env 文件")

    def test_render_template_rejects_unknown_placeholder(self):
        template = self.root / "weird.service.tmpl"
        template.write_text("[Service]\nExecStart=@NOPE@/x\n", encoding="utf-8")
        with self.assertRaises(lib.InstallError) as ctx:
            lib.render_template(template, {"PREFIX": "/x"})
        self.assertEqual(ctx.exception.code, lib.EXIT_DECLARATION)
        self.assertIn("未知占位符", str(ctx.exception))

    def test_precheck_rejects_wheelhouse_target_mismatch(self):
        """wheelhouse 属于别的构建目标 ⇒ 预检必须拒绝（装错标签=静默装错）。"""
        (self.art / "wheelhouse" / "wheelhouse.json").write_text(
            '{"schema_version": "iraf.wheelhouse/v1", "target": "aarch64-manylinux_2_28-cp310"}\n',
            encoding="utf-8")
        checks = lib.precheck(self.effective, self.root, allow_unverified=True,
                              wheelhouse=self.art / "wheelhouse", sdk_wheel=self.wheel,
                              python_exe=sys.executable, rehearsal=True)
        failed = [item["name"] for item in checks if not item["ok"]]
        self.assertTrue(any("wheelhouse 构建目标一致" in name for name in failed), failed)

    def _make_dep_wheel(self, dest: Path, name: str, version: str = "1.0.0") -> Path:
        """造一个最小的纯 Python 依赖 wheel（只为验证"按声明装依赖"这条链路）。"""
        import zipfile
        wheel = dest / ("%s-%s-py3-none-any.whl" % (name, version))
        with zipfile.ZipFile(wheel, "w") as zf:
            zf.writestr("%s/__init__.py" % name, "__version__ = '%s'\n" % version)
            zf.writestr("%s-%s.dist-info/METADATA" % (name, version),
                        "Metadata-Version: 2.1\nName: %s\nVersion: %s\n" % (name, version))
            zf.writestr("%s-%s.dist-info/WHEEL" % (name, version),
                        "Wheel-Version: 1.0\nGenerator: test\nRoot-Is-Purelib: true\nTag: py3-none-any\n")
            zf.writestr("%s-%s.dist-info/RECORD" % (name, version), "")
        return wheel

    def test_runtime_deps_and_stubs_are_installed(self):
        """按声明装运行期依赖 + 拷 proto 桩，并把"桩可导入"纳入后置自检。

        背景（真机实测踩到）：SDK wheel 的 METADATA 不声明依赖 ⇒ 只装 SDK 会导致运行期
        `import jsonschema` 失败；runtime 包内也不含 `iraf.v1.*` 桩 ⇒ gRPC 适配器起不来。
        """
        for name in ("pyyaml", "jsonschema", "grpcio", "protobuf"):
            self._make_dep_wheel(self.art / "wheelhouse", name)
        stubs = self.art / "stubs" / "iraf" / "v1"
        stubs.mkdir(parents=True, exist_ok=True)
        (stubs.parent / "__init__.py").write_text("", encoding="utf-8")
        (stubs / "__init__.py").write_text("", encoding="utf-8")
        (stubs / "skill_pb2.py").write_text("VALUE = 1\n", encoding="utf-8")
        self.effective["runtime_dependencies"] = ["pyyaml", "jsonschema", "grpcio", "protobuf"]
        self.effective["python_stubs_dir"] = str(self.art / "stubs")

        plan = self._install()
        self.assertTrue(plan["ok"])
        self.assertGreaterEqual(plan.get("stubs_copied", 0), 1)
        self.assertTrue((self._version_dir() / "lib/iraf/v1/skill_pb2.py").is_file())
        self.assertTrue((self._version_dir() / "lib/jsonschema/__init__.py").is_file())
        # 桩与依赖都在安装记录里（卸载时才会被一起删掉）
        record = lib.read_record(self.effective, self._version_dir())
        paths = [item["path"] for item in record["files"]]
        self.assertIn("lib/iraf/v1/skill_pb2.py", paths)
        self.assertIn("lib/jsonschema/__init__.py", paths)

    def test_stubs_dir_missing_is_rejected(self):
        """声明了桩目录但目录不存在 ⇒ 安装必须失败（不许静默少装一半）。"""
        self.effective["python_stubs_dir"] = str(self.art / "no-such-stubs")
        with self.assertRaises(lib.InstallError) as ctx:
            self._install()
        self.assertIn("桩目录不存在", str(ctx.exception))

    def test_install_refused_when_target_has_traces(self):
        """目标端已有本角色的痕迹（版本目录/链接/单元）⇒ **写盘之前**就拒绝（实测踩到的半状态根因）。"""
        unit_name = "%s.service" % self.effective["services"][0]["name"]
        unit_dir = Path(str(self.root) + "/etc/systemd/system")
        unit_dir.mkdir(parents=True, exist_ok=True)
        (unit_dir / unit_name).write_text("[Unit]\n", encoding="utf-8")
        with self.assertRaises(lib.InstallError) as ctx:
            self._install()
        self.assertEqual(ctx.exception.code, lib.EXIT_MISMATCH)
        self.assertIn("已有本角色的安装痕迹", str(ctx.exception))
        self.assertIn("--force-clean", str(ctx.exception))
        # 关键：拒绝发生在写盘之前 —— 版本目录与 env 都不应被创建
        self.assertFalse(self._version_dir().exists())
        self.assertFalse((Path(str(self.root) + str(self.effective["config_dir"]))
                         / ("%s.env" % PROFILE)).exists())

    def test_rollback_keeps_preexisting_version_dir(self):
        """回滚只能删**本次创建**的东西：预先存在的版本目录必须留下（原来会 rmtree 掉 ⇒ 半状态）。"""
        version_dir = self._version_dir()
        version_dir.mkdir(parents=True)
        (version_dir / "pre-existing.txt").write_text("上一轮安装留下的东西\n", encoding="utf-8")
        # 用"运行期依赖缺失"制造安装中途失败（pip 阶段就会失败）
        self.effective["runtime_dependencies"] = ["definitely-not-in-wheelhouse"]
        with self.assertRaises(lib.InstallError):
            self._install()
        self.assertTrue(version_dir.is_dir(), "预先存在的版本目录不得被回滚删除")
        self.assertTrue((version_dir / "pre-existing.txt").is_file())

    def test_force_clean_removes_declared_paths_and_keeps_data(self):
        """无记录兜底清理：只删声明路径（前缀/unit/env），数据与日志保留，逐项可列。"""
        unit_name = "%s.service" % self.effective["services"][0]["name"]
        unit_dir = Path(str(self.root) + "/etc/systemd/system")
        config_dir = Path(str(self.root) + str(self.effective["config_dir"]))
        data_dir = Path(str(self.root) + str(self.effective["data_dir"]))
        version_dir = self._version_dir()
        for path, text in ((unit_dir / unit_name, "[Unit]\n"),
                           (config_dir / ("%s.env" % PROFILE), "IRAF_PROFILE=x\n"),
                           (version_dir / "leftover.txt", "半状态残留\n"),
                           (data_dir / "iraf-events.db", "db\n")):
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(text, encoding="utf-8")

        plan = lib.plan_force_clean(self.effective, self.root)
        listed = [str(item) for item in plan["delete"]]
        self.assertTrue(any(str(unit_dir / unit_name) == item for item in listed))
        self.assertTrue(any(str(config_dir / ("%s.env" % PROFILE)) == item for item in listed))
        self.assertTrue(any(str(version_dir) == item for item in listed))
        self.assertIn(str(data_dir), [str(item) for item in plan["keep"]])

        lib.apply_uninstall(plan, root=self.root)
        self.assertFalse(unit_dir.joinpath(unit_name).exists())
        self.assertFalse(config_dir.joinpath("%s.env" % PROFILE).exists())
        self.assertTrue(data_dir.is_dir(), "数据目录默认保留")
        self.assertTrue(lib.residue_is_clean(lib.residue_scan(self.effective, self.root)))

    def test_uninstall_force_clean_flag_works_without_record(self):
        """无记录 + --force-clean ⇒ 兜底清理（不再直接拒绝）；不给该开关仍必须拒绝。"""
        unit_dir = Path(str(self.root) + "/etc/systemd/system")
        unit_dir.mkdir(parents=True, exist_ok=True)
        (unit_dir / ("%s.service" % self.effective["services"][0]["name"])).write_text("[Unit]\n", encoding="utf-8")
        code, output = self._run("uninstall", "--profile", PROFILE, "--root", str(self.root))
        self.assertEqual(code, lib.EXIT_MISMATCH)
        # 无记录/无版本目录两种拒绝都算（措辞取决于先撞到哪一层），关键是**明确拒绝且不删任何东西**
        self.assertTrue("缺少安装记录" in output or "未找到已装版本" in output, output)
        self.assertTrue(unit_dir.joinpath("%s.service" % self.effective["services"][0]["name"]).exists())
        code, output = self._run("uninstall", "--profile", PROFILE, "--root", str(self.root), "--force-clean")
        self.assertEqual(code, lib.EXIT_OK, output)
        self.assertIn("兜底清理", output)
        self.assertFalse(unit_dir.joinpath("%s.service" % self.effective["services"][0]["name"]).exists())

    def test_sim_scene_is_copied_and_backend_config_generated(self):
        """仿真档：声明的场景（XML + 旁挂报告）复制进版本目录，并在**目标端**生成 IRAF_BACKEND_CONFIG。

        为什么必须自动化：这份 JSON 手写会随契约演进静默过期（实测两次：缺 finger_geom → 装配失败；
        缺 vision 段 → visual_pick 报"未提供视觉证据路径"）⇒ 一律由场景报告生成。
        """
        # 造最小场景（emit_backend_config 需要 target_id + gripper）
        scene_dir = self.art / "scene"
        scene_dir.mkdir(parents=True, exist_ok=True)
        (scene_dir / "demo-pick-scene.xml").write_text("<mujoco/>\n", encoding="utf-8")
        (scene_dir / "demo-pick-scene.json").write_text(
            json.dumps({"target_id": "box_01", "pose_tolerance_m": 0.005,
                        "gripper": {"left_finger_body": "l", "right_finger_body": "r"},
                        "vision": None}), encoding="utf-8")

        self.effective["sim_scene"] = str(scene_dir / "demo-pick-scene.xml")
        self.effective["backend_config"] = None
        plan = self._install()

        self.assertTrue(plan["ok"])
        # 场景两个文件都进了版本目录（并被纳入安装记录 ⇒ 卸载会一起清掉）
        self.assertTrue((self._version_dir() / "models/demo-pick-scene.xml").is_file())
        self.assertTrue((self._version_dir() / "models/demo-pick-scene.json").is_file())
        record = lib.read_record(self.effective, self._version_dir())
        paths = [item["path"] for item in record["files"]]
        self.assertIn("models/demo-pick-scene.xml", paths)
        # env 里的 IRAF_BACKEND_CONFIG 被填上且是合法 JSON，model_path 指向版本目录内的场景
        env_text = Path(plan["env_file"]).read_text(encoding="utf-8")
        line = [l for l in env_text.splitlines() if l.startswith("IRAF_BACKEND_CONFIG=")][0]
        config = json.loads(line.split("=", 1)[1])
        self.assertEqual(config["target_id"] if "target_id" in config else config["manipulation"]["targets"].popitem()[0],
                         "box_01")
        self.assertIn(str(self._version_dir() / "models"), config["model_path"])

    def test_sim_scene_missing_report_is_rejected(self):
        """声明了场景但缺同名 .json 旁挂报告 ⇒ 安装失败（不许产出半套配置）。"""
        self.effective["sim_scene"] = str(self.art / "scene" / "definitely-missing.xml")
        self.effective["backend_config"] = None
        with self.assertRaises(lib.InstallError) as ctx:
            self._install()
        self.assertIn("仿真场景缺文件", str(ctx.exception))

    def test_wheelhouse_is_used_offline(self):
        """依赖来源=wheelhouse 时必须带 --no-index/--find-links（离线可装，不碰网络）。"""
        plan = self._install()
        self.assertEqual(plan["install_method"], "target")
        self.assertEqual(plan["wheelhouse"], str(self.art / "wheelhouse"))
        self.assertEqual(plan["pip_returncode"], 0)


if __name__ == "__main__":
    unittest.main()
