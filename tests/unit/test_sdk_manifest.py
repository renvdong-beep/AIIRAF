"""`deploy/sdk/build_sdk.sh` + `deploy/sdk/lib_manifest.py` 的契约测试（步骤 06）。

被测契约（fail-closed）：
  1. 版本只有**一个**来源：`config/sdk/package_matrix.yaml` 声明 `version_ref`，
     真实值来自 `pyproject.toml#project.version`，且与 `iraf_sdk.__version__` 一致；
     矩阵里不得再出现版本号字面量（铁律 5.3）。
  2. wheel 是**纯 Python、与架构无关**：文件名 `<名称>-<版本>-py3-none-any.whl`、
     METADATA 名称/版本正确、RECORD 覆盖全部成员、无二进制扩展、无 `Requires-Dist`
     （SDK 只依赖标准库，目标端才可独立安装）。
  3. 打包入口必须能把"绿灯 + 假产物"变成硬失败：实测本机 `pip wheel .`（setuptools
     59.6.0 不认 PEP 621）会退出码 0 却产出 `UNKNOWN-0.0.0-py3-none-any.whl`（961 字节空包），
     这里用同名同标签的合成包做**负向对照**，确保校验器真的会拒绝它。
  4. 产物**可复算**：同一棵源码树连续两次构建的 SHA-256 逐位相同（gzip/tar 时间戳必须钉死）。
  5. 产物只含**仓库相对路径**：manifest 中任何绝对路径都是失败（本步自己的门禁就抓到过
     `protoc` 绝对路径泄漏）。
  6. 篡改必须被发现：包内改一个字节、哪怕把外层 sha256/size 重新盖章，`build_sdk.sh --verify`
     也必须退出 4（见 `test_shell_verify_*`）。
  7. IDL 破坏性变更：proto 内容变了但 `idlv1_version` 没升 → 退出码 4。

运行：`PYTHONPATH=src /usr/bin/python3 -m unittest tests.unit.test_sdk_manifest -v`
"""

import contextlib
import gzip
import hashlib
import io
import json
import os
import subprocess
import sys
import tarfile
import tempfile
import unittest
import zipfile
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / "deploy" / "sdk"))
sys.path.insert(0, str(REPO_ROOT / "src"))

import lib_manifest  # noqa: E402
from lib_manifest import ExitCode, ManifestError  # noqa: E402

import iraf_sdk  # noqa: E402

SCRIPT = REPO_ROOT / "deploy" / "sdk" / "build_sdk.sh"
MATRIX = REPO_ROOT / "config" / "sdk" / "package_matrix.yaml"
SRC_PACKAGE = REPO_ROOT / "src" / "iraf_sdk"


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    digest.update(path.read_bytes())
    return digest.hexdigest()


class Fixture(unittest.TestCase):
    """构造一棵最小可打包的临时仓库（`src/iraf_sdk` + bundle 目录 + 可选 proto）。"""

    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory(prefix="iraf-sdk-manifest-")
        self.root = Path(self._tmp.name)
        package = self.root / "src" / "iraf_sdk"
        package.mkdir(parents=True)
        (package / "__init__.py").write_text('__version__ = "9.9.9"\n', encoding="utf-8")
        (package / "errors.py").write_text("VALUE = 1\n", encoding="utf-8")
        for name in lib_manifest.BUNDLE_DIRS:
            (self.root / name).mkdir(exist_ok=True)
        (self.root / "src" / "iraf_sdk" / "run.py").write_text("print(1)\n", encoding="utf-8")
        (self.root / "config" / "keep.yaml").write_text("a: 1\n", encoding="utf-8")

    def tearDown(self):
        self._tmp.cleanup()

    def build_wheel(self, out: Path, version: str = "0.2.0", dist_name: str = "iraf-sdk"):
        return lib_manifest.build_wheel(
            self.root,
            out,
            dist_name=dist_name,
            version=version,
            package="iraf_sdk",
            summary="test",
            requires_python=">=3.9",
        )

    def build_bundle(self, out: Path, version: str = "0.2.0"):
        return lib_manifest.build_runtime_bundle(
            self.root, out, name="iraf-runtime", version=version, arch="aarch64"
        )


# --- 1. 声明层：版本单一来源 / 矩阵契约 ----------------------------------------


class DeclarationTests(unittest.TestCase):
    def test_matrix_passes_its_own_contract(self):
        matrix = lib_manifest.load_matrix(REPO_ROOT, MATRIX)
        self.assertEqual("iraf.sdk-matrix/v1", matrix["schema_version"])
        self.assertEqual(
            ["aarch64-manylinux_2_28-cp310"], [t["id"] for t in matrix["targets"]]
        )
        target = lib_manifest.select_target(matrix, None)
        self.assertEqual("aarch64", target["arch"])
        self.assertTrue(target["index_url"].startswith("http"))

    def test_matrix_does_not_duplicate_the_version_literal(self):
        """版本唯一来源：矩阵只允许写 version_ref，不得出现版本号字面量。"""
        matrix = lib_manifest.load_matrix(REPO_ROOT, MATRIX)
        version = lib_manifest.read_declared_version(REPO_ROOT, matrix["sdk"]["version_ref"])
        raw = MATRIX.read_text(encoding="utf-8")
        self.assertNotIn(
            version,
            raw,
            f"矩阵里出现了版本号字面量 {version}：版本必须只有 pyproject.toml 一个来源",
        )

    def test_declared_version_matches_sdk_module(self):
        matrix = lib_manifest.load_matrix(REPO_ROOT, MATRIX)
        version = lib_manifest.read_declared_version(REPO_ROOT, matrix["sdk"]["version_ref"])
        self.assertEqual("0.2.0", version)
        self.assertEqual(version, iraf_sdk.__version__)

    def test_version_ref_parser_reads_only_the_named_section(self):
        """负向对照：别的段里有同名键时不得取错。"""
        root = Path(tempfile.mkdtemp(prefix="iraf-version-ref-"))
        (root / "pyproject.toml").write_text(
            '[build-system]\nrequires = ["setuptools"]\n\n'
            '[tool.other]\nversion = "1.1.1"\n\n'
            '[project]\nname = "x"\nversion = "2.2.2"\n',
            encoding="utf-8",
        )
        self.assertEqual(
            "2.2.2", lib_manifest.read_declared_version(root, "pyproject.toml#project.version")
        )

    def test_version_ref_missing_key_is_precheck_failure(self):
        root = Path(tempfile.mkdtemp(prefix="iraf-version-ref-"))
        (root / "pyproject.toml").write_text('[project]\nname = "x"\n', encoding="utf-8")
        with self.assertRaises(ManifestError) as ctx:
            lib_manifest.read_declared_version(root, "pyproject.toml#project.version")
        self.assertEqual(ExitCode.PRECHECK, ctx.exception.exit_code)

    def test_version_ref_bad_format_is_precheck_failure(self):
        with self.assertRaises(ManifestError) as ctx:
            lib_manifest.read_declared_version(REPO_ROOT, "pyproject.toml")
        self.assertEqual(ExitCode.PRECHECK, ctx.exception.exit_code)

    def test_unknown_target_is_precheck_failure(self):
        matrix = lib_manifest.load_matrix(REPO_ROOT, MATRIX)
        with self.assertRaises(ManifestError) as ctx:
            lib_manifest.select_target(matrix, "does-not-exist")
        self.assertEqual(ExitCode.PRECHECK, ctx.exception.exit_code)

    def test_missing_matrix_is_precheck_failure(self):
        with self.assertRaises(ManifestError) as ctx:
            lib_manifest.load_matrix(REPO_ROOT, REPO_ROOT / "config/sdk/nope.yaml")
        self.assertEqual(ExitCode.PRECHECK, ctx.exception.exit_code)

    def test_git_state_is_measured_not_guessed(self):
        state = lib_manifest.git_state(REPO_ROOT)
        self.assertRegex(state["commit"], r"^[0-9a-f]{40}$")
        self.assertIsInstance(state["dirty"], bool)
        self.assertEqual(state["dirty"], state["dirty_file_count"] > 0)

    def test_exit_code_contract(self):
        self.assertEqual(
            (0, 1, 2, 3, 4),
            (ExitCode.OK, ExitCode.USAGE, ExitCode.PRECHECK, ExitCode.BUILD, ExitCode.VERIFY),
        )


# --- 2/3. wheel：正向结构与负向对照 --------------------------------------------


class WheelTests(Fixture):
    def test_wheel_structure_is_pep427_and_zero_dependency(self):
        built = self.build_wheel(self.root / "out")
        path = built["path"]
        self.assertEqual("iraf_sdk-0.2.0-py3-none-any.whl", path.name)
        self.assertEqual([], lib_manifest.validate_wheel(
            path, dist_name="iraf-sdk", version="0.2.0", package="iraf_sdk",
            required_members=built["required_members"],
        ))
        with zipfile.ZipFile(path) as handle:
            names = handle.namelist()
            self.assertIn("iraf_sdk/__init__.py", names)
            self.assertIn("iraf_sdk/errors.py", names)
            self.assertIn("iraf_sdk/run.py", names)
            metadata = handle.read("iraf_sdk-0.2.0.dist-info/METADATA").decode()
            self.assertIn("Name: iraf-sdk", metadata)
            self.assertIn("Version: 0.2.0", metadata)
            self.assertNotIn("Requires-Dist", metadata)
            record = handle.read("iraf_sdk-0.2.0.dist-info/RECORD").decode()
            for name in names:
                if name.endswith("RECORD"):
                    continue
                self.assertIn(name, record, f"RECORD 未覆盖 {name}")
            self.assertIn("sha256=", record)

    def test_wheel_is_reproducible_bit_identical(self):
        first = self.build_wheel(self.root / "out1")["path"]
        second = self.build_wheel(self.root / "out2")["path"]
        self.assertEqual(sha256_file(first), sha256_file(second))

    def test_rejects_the_measured_unknown_placeholder_wheel(self):
        """负向对照（实测形状）：`pip wheel .` 在 setuptools 59.6 下的 UNKNOWN-0.0.0 空包。"""
        path = self.root / "UNKNOWN-0.0.0-py3-none-any.whl"
        with zipfile.ZipFile(path, "w") as handle:
            handle.writestr("iraf_sdk-0.0.0.dist-info/METADATA", "Name: UNKNOWN\nVersion: 0.0.0\n")
        violations = lib_manifest.validate_wheel(
            path, dist_name="iraf-sdk", version="0.2.0", package="iraf_sdk"
        )
        self.assertTrue(any("文件名与声明不符" in item for item in violations), violations)
        self.assertTrue(any("缺少必需成员" in item for item in violations), violations)

    def test_rejects_wrong_version(self):
        path = self.build_wheel(self.root / "out", version="0.2.0")["path"]
        violations = lib_manifest.validate_wheel(
            path, dist_name="iraf-sdk", version="0.3.0", package="iraf_sdk"
        )
        self.assertTrue(any("文件名与声明不符" in item for item in violations), violations)

    def test_rejects_platform_specific_name(self):
        built = self.build_wheel(self.root / "out")
        moved = built["path"].with_name("iraf_sdk-0.2.0-cp310-manylinux_2_28_aarch64.whl")
        built["path"].rename(moved)
        violations = lib_manifest.validate_wheel(
            moved, dist_name="iraf-sdk", version="0.2.0", package="iraf_sdk"
        )
        self.assertTrue(any("py3-none-any" in item for item in violations), violations)

    def test_rejects_missing_required_member(self):
        built = self.build_wheel(self.root / "out")
        violations = lib_manifest.validate_wheel(
            built["path"], dist_name="iraf-sdk", version="0.2.0", package="iraf_sdk",
            required_members=["iraf_sdk/not_there.py"],
        )
        self.assertTrue(any("缺少必需成员" in item for item in violations), violations)

    def test_rejects_requires_dist(self):
        built = self.build_wheel(self.root / "out")
        path = built["path"]
        members = {name: zipfile.ZipFile(path).read(name) for name in zipfile.ZipFile(path).namelist()}
        with zipfile.ZipFile(path, "w") as handle:
            for name, data in members.items():
                if name.endswith("dist-info/METADATA"):
                    data = data + b"Requires-Dist: mujoco>=3.2\n"
                handle.writestr(name, data)
        violations = lib_manifest.validate_wheel(
            path, dist_name="iraf-sdk", version="0.2.0", package="iraf_sdk"
        )
        self.assertTrue(any("第三方依赖" in item for item in violations), violations)

    def test_rejects_binary_extension(self):
        built = self.build_wheel(self.root / "out")
        path = built["path"]
        with zipfile.ZipFile(path, "a") as handle:
            handle.writestr("iraf_sdk/_native.so", b"\x7fELF")
        violations = lib_manifest.validate_wheel(
            path, dist_name="iraf-sdk", version="0.2.0", package="iraf_sdk"
        )
        self.assertTrue(any("二进制扩展" in item for item in violations), violations)

    def test_missing_package_dir_is_build_failure(self):
        with self.assertRaises(ManifestError) as ctx:
            lib_manifest.build_wheel(
                self.root / "empty", self.root / "out", dist_name="iraf-sdk", version="0.2.0",
                package="iraf_sdk", summary="x", requires_python=">=3.9",
            )
        self.assertEqual(ExitCode.BUILD, ctx.exception.exit_code)


# --- 4/5. bundle：确定性、成员过滤、相对路径 -----------------------------------


class BundleTests(Fixture):
    def test_bundle_is_reproducible_bit_identical(self):
        first = self.build_bundle(self.root / "out1")["path"]
        second = self.build_bundle(self.root / "out2")["path"]
        self.assertEqual(sha256_file(first), sha256_file(second))

    def test_bundle_excludes_bytecode_and_records_exclusions(self):
        cache = self.root / "src" / "__pycache__"
        cache.mkdir()
        (cache / "x.cpython-310.pyc").write_bytes(b"\x00")
        (self.root / "src" / "iraf_sdk" / "oracle.py.orig").write_text("old\n", encoding="utf-8")
        built = self.build_bundle(self.root / "out")
        members = [entry["path"] for entry in built["files"]]
        self.assertIn("src/iraf_sdk/__init__.py", members)  # 正向对照：真源码必须在
        self.assertNotIn("src/__pycache__/x.cpython-310.pyc", members)
        self.assertNotIn("src/iraf_sdk/oracle.py.orig", members)
        excluded = [entry["path"] for entry in built["excluded"]]
        self.assertIn("src/__pycache__/x.cpython-310.pyc", excluded)
        self.assertIn("src/iraf_sdk/oracle.py.orig", excluded)

    def test_bundle_members_are_relative_sorted_and_have_hashes(self):
        built = self.build_bundle(self.root / "out")
        members = [entry["path"] for entry in built["files"]]
        self.assertEqual(sorted(members), members)
        for entry in built["files"]:
            self.assertFalse(entry["path"].startswith("/"), entry)
            self.assertNotIn("..", Path(entry["path"]).parts, entry)
            self.assertRegex(entry["sha256"], r"^[0-9a-f]{64}$")
            self.assertGreater(entry["size_bytes"], 0)

    def test_missing_declared_bundle_dir_is_precheck_failure(self):
        (self.root / "skills").rmdir()
        with self.assertRaises(ManifestError) as ctx:
            lib_manifest.build_runtime_bundle(
                self.root, self.root / "out", name="iraf-runtime", version="0.2.0", arch="aarch64"
            )
        self.assertEqual(ExitCode.PRECHECK, ctx.exception.exit_code)


# --- manifest / 绝对路径门禁 / verify ------------------------------------------


class ManifestContractTests(Fixture):
    def _manifest(self, out: Path) -> dict:
        wheel = self.build_wheel(out)
        bundle = self.build_bundle(out)
        matrix = lib_manifest.load_matrix(REPO_ROOT, MATRIX)
        target = lib_manifest.select_target(matrix, None)
        stubs = lib_manifest.stub_status(REPO_ROOT)
        probe = {"ok": False, "reason": "fixture"}
        artifacts = [
            {"kind": "wheel", "path": lib_manifest.relpath_posix(wheel["path"], self.root),
             "sha256": sha256_file(wheel["path"]), "size_bytes": wheel["path"].stat().st_size,
             "files": wheel["files"]},
            {"kind": "runtime_bundle", "path": lib_manifest.relpath_posix(bundle["path"], self.root),
             "sha256": sha256_file(bundle["path"]), "size_bytes": bundle["path"].stat().st_size,
             "files": bundle["files"]},
        ]
        # 版本/IDL 用真实仓库的声明值（fixture 只是产物来源）
        return lib_manifest.build_manifest(
            matrix=matrix, target=target, version="0.2.0", repo_root=REPO_ROOT,
            artifacts=artifacts, bundle_excluded=bundle["excluded"],
            idl_digest_value=lib_manifest.idl_digest(REPO_ROOT),
            stubs=stubs, stub_probe=probe,
            local_stub_generation={"dir": "build/sdk/pystub-local", "protoc": "test",
                                   "files": [], "regenerable_locally": True, "grpc_included": False},
        )

    def test_manifest_required_fields_and_no_absolute_paths(self):
        out = self.root / "out"
        manifest = self._manifest(out)
        self.assertEqual("iraf.package-manifest/v1", manifest["schema_version"])
        self.assertEqual(2, len(manifest["artifacts"]))
        self.assertFalse(manifest["target"]["verified"], "未实测的目标标签不得标 verified")
        self.assertEqual("pending", manifest["signature"]["scheme"])
        self.assertIsNone(manifest["signature"]["value"])
        self.assertEqual("placeholder", manifest["sbom"]["status"])
        self.assertEqual("pending_fetch", manifest["wheelhouse"]["status"])
        self.assertEqual([], lib_manifest.absolute_path_violations(manifest))

    def test_absolute_path_gate_rejects_home_path(self):
        """本步自己的门禁：写盘内容里出现本机路径必须失败（曾抓到 protoc 绝对路径）。"""
        manifest = self._manifest(self.root / "out")
        manifest["stubs"]["local_generation"]["protoc"] = "/usr/bin/protoc"
        violations = lib_manifest.absolute_path_violations(manifest)
        self.assertTrue(any("绝对路径" in item for item in violations), violations)

    def test_write_and_verify_roundtrip(self):
        out = self.root / "out"
        manifest = self._manifest(out)
        self.assertEqual([], lib_manifest.absolute_path_violations(manifest))
        path = lib_manifest.write_manifest(out, manifest)
        self.assertTrue(path.is_file())
        self.assertEqual([], lib_manifest.verify_manifest(path, self.root))
        self.assertNotIn("/home/", path.read_text(encoding="utf-8"))

    def test_verify_detects_tampered_bundle_member_even_when_outer_hash_restamped(self):
        out = self.root / "out"
        manifest = self._manifest(out)
        path = lib_manifest.write_manifest(out, manifest)
        bundle = out / manifest["artifacts"][1]["path"].split("/")[-1]
        members = []
        with tarfile.open(bundle, "r:gz") as tar:
            for member in tar.getmembers():
                handle = tar.extractfile(member)
                members.append((member, handle.read() if handle else b""))
        index = next(i for i, (info, _) in enumerate(members) if info.isfile())
        info, payload = members[index]
        # 陷阱：tarfile 按 TarInfo.size 读取成员，重打包不更新 size 会让"篡改"在读侧完全不可见。
        info.size = len(payload) + len(b"# tampered\n")
        members[index] = (info, payload + b"# tampered\n")
        buffer = io.BytesIO()
        with gzip.GzipFile(filename="", mode="wb", fileobj=buffer, mtime=0) as gz:
            with tarfile.open(fileobj=gz, mode="w", format=tarfile.GNU_FORMAT) as tar:
                for info, payload in members:
                    tar.addfile(info, io.BytesIO(payload))
        bundle.write_bytes(buffer.getvalue())
        manifest["artifacts"][1]["sha256"] = sha256_file(bundle)
        manifest["artifacts"][1]["size_bytes"] = bundle.stat().st_size
        lib_manifest.write_manifest(out, manifest)
        violations = lib_manifest.verify_manifest(path, self.root)
        self.assertTrue(any("被篡改" in item for item in violations), violations)

    def test_verify_detects_tampered_wheel_member(self):
        out = self.root / "out"
        manifest = self._manifest(out)
        path = lib_manifest.write_manifest(out, manifest)
        wheel = out / manifest["artifacts"][0]["path"].split("/")[-1]
        with zipfile.ZipFile(wheel, "a") as handle:
            handle.writestr("iraf_sdk/extra.py", b"x = 1\n")
        manifest["artifacts"][0]["sha256"] = sha256_file(wheel)
        manifest["artifacts"][0]["size_bytes"] = wheel.stat().st_size
        lib_manifest.write_manifest(out, manifest)
        violations = lib_manifest.verify_manifest(path, self.root)
        self.assertTrue(any("未登记文件" in item for item in violations), violations)

    def test_verify_reports_missing_artifact(self):
        out = self.root / "out"
        manifest = self._manifest(out)
        path = lib_manifest.write_manifest(out, manifest)
        (out / manifest["artifacts"][1]["path"].split("/")[-1]).unlink()
        violations = lib_manifest.verify_manifest(path, self.root)
        self.assertTrue(any("产物缺失" in item for item in violations), violations)

    def test_verify_missing_manifest_is_precheck_failure(self):
        with self.assertRaises(ManifestError) as ctx:
            lib_manifest.verify_manifest(self.root / "nope.json", self.root)
        self.assertEqual(ExitCode.PRECHECK, ctx.exception.exit_code)


# --- IDL 门禁 ------------------------------------------------------------------


class IdlContractTests(unittest.TestCase):
    def test_idl_digest_is_content_sensitive(self):
        root = Path(tempfile.mkdtemp(prefix="iraf-idl-"))
        proto_root = root / "api" / "proto"
        for rel in lib_manifest.IDL_PROTO_FILES:
            path = proto_root / rel
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(f"syntax = \"proto3\";\n// {rel.name}\n", encoding="utf-8")
        before = lib_manifest.idl_digest(root, Path("api/proto"))
        target = proto_root / lib_manifest.IDL_PROTO_FILES[1]
        target.write_text(target.read_text(encoding="utf-8") + "// changed\n", encoding="utf-8")
        self.assertNotEqual(before, lib_manifest.idl_digest(root, Path("api/proto")))

    def test_idl_change_without_version_bump_is_verify_failure(self):
        out = Path(tempfile.mkdtemp(prefix="iraf-idl-manifest-"))
        (out / "manifest.json").write_text(
            json.dumps({"idl": {"version": "1.0.0", "digest": "a" * 64}}), encoding="utf-8"
        )
        with self.assertRaises(ManifestError) as ctx:
            lib_manifest.enforce_idl_contract(out / "manifest.json", "b" * 64, "1.0.0")
        self.assertEqual(ExitCode.VERIFY, ctx.exception.exit_code)

    def test_idl_change_with_version_bump_is_accepted(self):
        out = Path(tempfile.mkdtemp(prefix="iraf-idl-manifest-"))
        (out / "manifest.json").write_text(
            json.dumps({"idl": {"version": "1.0.0", "digest": "a" * 64}}), encoding="utf-8"
        )
        lib_manifest.enforce_idl_contract(out / "manifest.json", "b" * 64, "1.1.0")

    def test_no_previous_manifest_is_not_a_failure(self):
        lib_manifest.enforce_idl_contract(Path("/nonexistent/manifest.json"), "c" * 64, "1.0.0")

    def test_proto_services_are_measured_from_source(self):
        """protoc 只为声明了 service 的 proto 生成 grpc stub：必须从 IDL 实测，不能假设。"""
        services = {
            rel.stem: lib_manifest.proto_services(REPO_ROOT, lib_manifest.DEFAULT_PROTO_ROOT, rel)
            for rel in lib_manifest.IDL_PROTO_FILES
        }
        status = lib_manifest.stub_status(REPO_ROOT)
        self.assertEqual(sorted(status["grpc_expected"]),
                         sorted(name for name, names in services.items() if names))
        self.assertIn("SkillRuntimeService", services["runtime"])
        self.assertIn("EventService", services["events"])
        self.assertEqual([], services["common"])


# --- CLI / 入口脚本 -------------------------------------------------------------


class ShellEntryTests(unittest.TestCase):
    def run_script(self, *args, timeout=300):
        return subprocess.run(
            ["bash", str(SCRIPT), *args],
            cwd=str(REPO_ROOT), capture_output=True, text=True, timeout=timeout,
        )

    def test_help_exits_zero(self):
        result = self.run_script("--help")
        self.assertEqual(0, result.returncode, result.stderr)
        self.assertIn("退出码", result.stdout)

    def test_unknown_argument_is_usage_error(self):
        result = self.run_script("--nope")
        self.assertEqual(ExitCode.USAGE, result.returncode)

    def test_dry_run_writes_nothing(self):
        with tempfile.TemporaryDirectory(prefix="iraf-sdk-dry-") as tmp:
            out = Path(tmp) / "out"
            result = self.run_script("--dry-run", "--out", str(out))
            self.assertEqual(0, result.returncode, result.stderr)
            self.assertIn("dry-run 结束", result.stdout)
            self.assertFalse(out.exists(), "dry-run 不得创建产物目录")

    def test_verify_missing_manifest_exits_precheck(self):
        with tempfile.TemporaryDirectory(prefix="iraf-sdk-verify-") as tmp:
            result = self.run_script("--verify", "--manifest", f"{tmp}/nope.json")
            self.assertEqual(ExitCode.PRECHECK, result.returncode)

    def test_shell_verify_matches_lib_manifest(self):
        """入口脚本的退出码必须与 lib 契约一致（正向 0 / 篡改 4）。"""
        matrix = lib_manifest.load_matrix(REPO_ROOT, MATRIX)
        target = lib_manifest.select_target(matrix, None)
        with tempfile.TemporaryDirectory(prefix="iraf-sdk-shell-") as tmp:
            root = Path(tmp)
            source = root / "src" / "iraf_sdk"
            source.mkdir(parents=True)
            (source / "__init__.py").write_text("x = 1\n", encoding="utf-8")
            for name in lib_manifest.BUNDLE_DIRS:
                (root / name).mkdir(exist_ok=True)
            out = root / "out"
            wheel = lib_manifest.build_wheel(
                root, out, dist_name=matrix["sdk"]["name"], version="0.2.0",
                package="iraf_sdk", summary="t", requires_python=">=3.9",
            )
            bundle = lib_manifest.build_runtime_bundle(
                root, out, name="iraf-runtime", version="0.2.0", arch=target["arch"]
            )
            artifacts = [
                {"kind": "wheel", "path": lib_manifest.relpath_posix(wheel["path"], root),
                 "sha256": sha256_file(wheel["path"]),
                 "size_bytes": wheel["path"].stat().st_size, "files": wheel["files"]},
                {"kind": "runtime_bundle", "path": lib_manifest.relpath_posix(bundle["path"], root),
                 "sha256": sha256_file(bundle["path"]),
                 "size_bytes": bundle["path"].stat().st_size, "files": bundle["files"]},
            ]
            manifest = lib_manifest.build_manifest(
                matrix=matrix, target=target, version="0.2.0", repo_root=REPO_ROOT,
                artifacts=artifacts, bundle_excluded=bundle["excluded"],
                idl_digest_value=lib_manifest.idl_digest(REPO_ROOT),
                stubs=lib_manifest.stub_status(REPO_ROOT),
                stub_probe={"ok": False, "reason": "fixture"},
                local_stub_generation={"dir": "x", "protoc": "t", "files": [],
                                       "regenerable_locally": True, "grpc_included": False},
            )
            manifest_path = lib_manifest.write_manifest(out, manifest)

            # 正向对照：未篡改必须退出 0
            ok = subprocess.run(
                ["bash", str(SCRIPT), "--verify", "--repo-root", str(root),
                 "--manifest", str(manifest_path)],
                cwd=str(REPO_ROOT), capture_output=True, text=True, timeout=300,
            )
            self.assertEqual(ExitCode.OK, ok.returncode, ok.stderr)

            # 负向：整包翻转一个字节 → 必须退出 4
            raw = bytearray(bundle["path"].read_bytes())
            raw[-1] ^= 0x01
            bundle["path"].write_bytes(bytes(raw))
            bad = subprocess.run(
                ["bash", str(SCRIPT), "--verify", "--repo-root", str(root),
                 "--manifest", str(manifest_path)],
                cwd=str(REPO_ROOT), capture_output=True, text=True, timeout=300,
            )
            self.assertEqual(ExitCode.VERIFY, bad.returncode, bad.stdout + bad.stderr)
            self.assertIn("SHA-256", bad.stdout + bad.stderr)


class PlanCommandTests(unittest.TestCase):
    def test_plan_reports_declared_values_and_writes_nothing(self):
        with tempfile.TemporaryDirectory(prefix="iraf-sdk-plan-") as tmp:
            out = Path(tmp) / "out"
            buffer = io.StringIO()
            with contextlib.redirect_stdout(buffer):
                code = lib_manifest.main(
                    ["plan", "--repo-root", str(REPO_ROOT), "--out", str(out)]
                )
            text = buffer.getvalue()
            self.assertEqual(ExitCode.OK, code)
            self.assertIn("iraf_sdk-0.2.0-py3-none-any.whl", text)
            self.assertIn("iraf-runtime-0.2.0-aarch64.tar.gz", text)
            self.assertIn("verified=false", text)
            self.assertFalse(out.exists())

    def test_unknown_subcommand_is_usage_error(self):
        buffer = io.StringIO()
        with contextlib.redirect_stdout(buffer), contextlib.redirect_stderr(io.StringIO()):
            code = lib_manifest.main(["nope"])
        self.assertEqual(ExitCode.USAGE, code)

    def test_unknown_option_is_usage_error(self):
        with contextlib.redirect_stderr(io.StringIO()):
            code = lib_manifest.main(["plan", "--nope"])
        self.assertEqual(ExitCode.USAGE, code)


if __name__ == "__main__":
    unittest.main()
