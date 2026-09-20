#!/usr/bin/env python3
"""`deploy/sdk/lib_board_bundle.py`（板级 bundle 组装 + 目标端预检）的单元测试。

契约来源：`config/sdk/package_matrix.yaml` 的 `delivery` 段 + `iraf.board-bundle/v1` 清单。
本文件只做**离线可跑**的判定：不联网、不装包、不碰真实系统目录（沙箱全在临时目录里）。

测试纪律（本步实测踩过的坑）：
  1. 每个负向用例都配一个**正例对照**（否则无法区分"严格门禁"与"恒失败门禁"）；
  2. 门禁断言到 `kind`/具体违规文本，而不是只断言"非空"；
  3. 篡改用例必须先证明**未篡改时校验通过**（正例），再证明篡改被发现（负例）。

运行：PYTHONPATH=src /usr/bin/python3 -m unittest tests.unit.test_install_prechecks -v
"""

from __future__ import annotations

import copy
import json
import os
import re
import shutil
import sys
import tempfile
import unittest
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
STUB_DIR = REPO_ROOT / "deploy" / "sdk"
sys.path.insert(0, str(STUB_DIR))

import lib_board_bundle as lbb  # noqa: E402
from lib_manifest import ExitCode, ManifestError, load_matrix  # noqa: E402

MATRIX_PATH = REPO_ROOT / "config" / "sdk" / "package_matrix.yaml"
BOARD_PATH = REPO_ROOT / "profiles" / "boards" / "e300.yaml"

MATCHING_FACTS = {
    "machine": "aarch64",
    "python_version": "3.10.12",
    "python_major_minor": "3.10",
    "system": "Linux",
    "release": "5.10.0",
    "disk_free_mb": 8000,
    "mem_available_mb": 4096,
    "glibc": "2.35",
    "systemd_available": True,
    "pip_available": True,
    "yaml_available": True,
    "jsonschema_available": True,
    "python_exe": "/usr/bin/python3",
}

TARGET = {
    "id": "aarch64-manylinux_2_28-cp310",
    "arch": "aarch64",
    "python_tag": "cp310",
    "platform_tag": "manylinux_2_28_aarch64",
}

BOARD_REPORT_VERIFIED = {
    "exit_code": 0,
    "verified": True,
    "status": "verified",
    "unverified_fields": [],
    "pending_fields": [],
    "reasons": [],
    "failures": [],
}
BOARD_REPORT_UNVERIFIED = {
    "exit_code": 2,
    "verified": False,
    "status": "unverified",
    "unverified_fields": ["status", "target.os", "limits.ram_mb"],
    "pending_fields": ["evidence.owner"],
    "reasons": ["status 未验证（unverified）：请在板卡实测后回填"],
    "failures": [],
}

MANIFEST_STUB = {"board": {"limits": {"ram_mb": 2048, "disk_mb": 4096}}}


def make_manifest(**overrides) -> dict:
    manifest = {
        "schema_version": lbb.BUNDLE_SCHEMA_VERSION,
        "bundle": {
            "board": "e300",
            "target": "aarch64-manylinux_2_28-cp310",
            "arch": "aarch64",
            "python_tag": "cp310",
            "platform_tag": "manylinux_2_28_aarch64",
            "version": "0.2.0",
            "manifest_member": "iraf-board-bundle.json",
            "staging_dir_name": "iraf-sdk-staging",
            "checksum_suffix": ".sha256",
        },
        "board": {
            "profile_member": "board/e300.yaml",
            "status": "unverified",
            "verified": False,
            "allow_unverified_at_build": True,
            "limits": {"ram_mb": "unverified", "disk_mb": "unverified"},
        },
        "sdk": {"name": "iraf-sdk", "version": "0.2.0", "wheel_member": "sdk/iraf_sdk-0.2.0-py3-none-any.whl"},
        "runtime": {"bundle_member": "runtime/iraf-runtime-0.2.0-aarch64.tar.gz", "size_bytes": 1024},
        "wheelhouse": {"dir_member": "wheelhouse/x", "wheels": [{"name": "mujoco.whl", "size_bytes": 10}], "declared": {}},
        "scripts": {"install_member": "scripts/install.sh"},
        "post_verify": {"script": "deploy/sdk/verify.sh", "present": False, "status": "pending_step_09"},
        "systemd": {"unit_member": "systemd/iraf-sdk-board.service"},
        "env": {"template_member": "env/iraf-sdk.env.template"},
        "files": [],
        "notes": [],
    }
    for key, value in overrides.items():
        manifest[key] = value
    return manifest


def make_bundle(directory: Path, members: dict, manifest: dict) -> Path:
    """用与生产同一套确定性打包实现合成一个 bundle（含伴随校验和）。"""
    entries = [{"path": name, "bytes": data, "source": None} for name, data in sorted(members.items())]
    entries.append(
        {
            "path": (manifest.get("bundle") or {}).get("manifest_member", "iraf-board-bundle.json"),
            "bytes": (json.dumps(manifest, ensure_ascii=False, indent=2) + "\n").encode("utf-8"),
            "source": None,
        }
    )
    directory.mkdir(parents=True, exist_ok=True)
    path = directory / "bundle.tar.gz"
    payload = lbb._write_deterministic_tar(entries)
    path.write_bytes(payload)
    Path(str(path) + ".sha256").write_text(
        f"{lbb.sha256_bytes(payload)}  {path.name}\n", encoding="utf-8"
    )
    return path


class GlibcAndTagTest(unittest.TestCase):
    def test_detect_glibc_version_is_plausible(self):
        """坑：ctypes 默认 c_int 会给出无意义整数（本机实测 -44196217），必须 restype=c_char_p。"""
        version = lbb.detect_glibc_version()
        self.assertIsNotNone(version, "两条探测路径都失败时不应静默通过")
        self.assertRegex(version, r"^[0-9]+\.[0-9]+", f"glibc 版本形状不合法：{version!r}")

    def test_platform_tag_glibc_mapping(self):
        cases = {
            "manylinux_2_28_aarch64": (2, 28),
            "manylinux_2_17_aarch64": (2, 17),
            "manylinux2014_aarch64": (2, 17),
            "manylinux2010_x86_64": (2, 12),
            "manylinux1_x86_64": (2, 5),
        }
        for tag, expected in cases.items():
            with self.subTest(tag=tag):
                self.assertEqual(lbb.platform_tag_glibc(tag), expected)

    def test_platform_tag_glibc_unknown_returns_none(self):
        for tag in ("", "linux_aarch64", "manylinux_aarch64", None):
            with self.subTest(tag=tag):
                self.assertIsNone(lbb.platform_tag_glibc(tag or ""))

    def test_parse_version_and_python_tag(self):
        self.assertEqual(lbb.parse_version("3.10.12"), (3, 10, 12))
        self.assertEqual(lbb.parse_version("2.35"), (2, 35))
        self.assertIsNone(lbb.parse_version(None))
        self.assertIsNone(lbb.parse_version("abc"))
        self.assertEqual(lbb.python_tag_version("cp310"), (3, 10))
        self.assertIsNone(lbb.python_tag_version("py3"))

    def test_is_relative_member(self):
        for good in ("env/iraf-sdk.env.template", "iraf-board-bundle.json", "scripts/install.sh"):
            with self.subTest(good=good):
                self.assertTrue(lbb.is_relative_member(good))
        for bad in ("/etc/passwd", "../evil.txt", "a/../../b", "dir/", "", "C:\\x"):
            with self.subTest(bad=bad):
                self.assertFalse(lbb.is_relative_member(bad))


class DeliveryDeclarationTest(unittest.TestCase):
    def setUp(self):
        self.matrix = load_matrix(REPO_ROOT, MATRIX_PATH)

    def test_real_matrix_declares_complete_delivery(self):
        """正例对照：仓库里的矩阵必须能通过 delivery 门禁。"""
        delivery = lbb.load_delivery(self.matrix)
        self.assertEqual(delivery["install"]["prefix"], "/opt/iraf-sdk")
        self.assertEqual(delivery["install"]["service_unit"], "iraf-sdk-board.service")
        self.assertEqual(delivery["install"]["current_link"], "current")
        self.assertIn("{board}", delivery["board_bundle"]["name_template"])

    def test_missing_delivery_is_explicit_failure(self):
        broken = copy.deepcopy(self.matrix)
        broken.pop("delivery")
        with self.assertRaises(lbb.BoardBundleError) as ctx:
            lbb.load_delivery(broken)
        self.assertEqual(ctx.exception.exit_code, ExitCode.PRECHECK)
        self.assertIn("delivery", str(ctx.exception))

    def test_missing_install_key_fails_closed(self):
        broken = copy.deepcopy(self.matrix)
        broken["delivery"]["install"].pop("python_bin")
        with self.assertRaises(lbb.BoardBundleError) as ctx:
            lbb.load_delivery(broken)
        self.assertIn("delivery.install.python_bin", str(ctx.exception))

    def test_bundle_name_renders_and_rejects_unknown_placeholder(self):
        delivery = lbb.load_delivery(self.matrix)
        self.assertEqual(
            lbb.bundle_name(delivery, board="e300", version="0.2.0", arch="aarch64"),
            "iraf-board-e300-0.2.0-aarch64.tar.gz",
        )
        broken = {"board_bundle": dict(delivery["board_bundle"])}
        broken["board_bundle"]["name_template"] = "iraf-board-{board}-{version}-{arch}-{extra}.tar.gz"
        with self.assertRaises(lbb.BoardBundleError) as ctx:
            lbb.bundle_name(broken, board="e300", version="0.2.0", arch="aarch64")
        self.assertEqual(ctx.exception.exit_code, ExitCode.PRECHECK)

    def test_env_template_renders_all_placeholders(self):
        """env 模板在**组装期**烘焙 SDK 名/版本/板卡/目标，其余占位符在安装期渲染。"""
        text = lbb.env_template_text(sdk_name="iraf-sdk", version="0.2.0", board="e300", target_id="t")
        self.assertNotIn("@SDK_NAME@", text, "组装期应已烘焙 SDK 名")
        values = {
            "@SDK_NAME@": "iraf-sdk",
            "@SDK_VERSION@": "0.2.0",
            "@BOARD@": "e300",
            "@TARGET_ID@": "t",
            "@PREFIX@": "/opt/iraf-sdk",
            "@CURRENT_LINK@": "current",
            "@CONFIG_DIR@": "/etc/iraf",
            "@LOG_DIR@": "/var/log/iraf",
            "@SERVICE_UNIT@": "iraf-sdk-board.service",
            "@ENV_FILE@": "iraf-sdk.env",
            "@PYTHON_BIN@": "/usr/bin/python3",
        }
        self.assertEqual(sorted(values), sorted(lbb.PLACEHOLDERS))
        rendered = lbb.render_template(text, values, label="env 模板")
        self.assertNotIn("@", rendered)
        self.assertIn("IRAF_HOME=/opt/iraf-sdk/current", rendered)
        self.assertIn("IRAF_PYTHON=/usr/bin/python3", rendered)

    def test_systemd_unit_template_placeholders_are_covered(self):
        """systemd 单元模板里的占位符必须都在 PLACEHOLDERS 里（否则目标端会拿到半渲染单元）。"""
        unit = (REPO_ROOT / "deploy" / "sdk" / "iraf-sdk-board.service").read_text(encoding="utf-8")
        found = set(re.findall(r"@[A-Z0-9_]+@", unit))
        self.assertTrue(found, "单元模板应当含占位符（否则是硬编码路径）")
        self.assertEqual(found - set(lbb.PLACEHOLDERS), set())
        # 单元模板不得出现构建机路径
        self.assertNotIn("/home/", unit)

    def test_env_template_rejects_leftover_placeholder(self):
        with self.assertRaises(lbb.BoardBundleError) as ctx:
            lbb.render_template("X=@UNKNOWN@\n", {}, label="env 模板")
        self.assertIn("未声明的占位符", str(ctx.exception))

    def test_env_template_rejects_missing_value(self):
        values = {"@PREFIX@": "/opt/iraf-sdk"}
        with self.assertRaises(lbb.BoardBundleError) as ctx:
            lbb.render_template("X=@PREFIX@\n", values, label="env 模板")
        self.assertIn("缺少占位符取值", str(ctx.exception))


class PrecheckTest(unittest.TestCase):
    def violations(self, facts=None, manifest=None, board=None, layout=None, allow=False, disk=100):
        return lbb.precheck_violations(
            facts=dict(MATCHING_FACTS, **(facts or {})),
            manifest=manifest if manifest is not None else MANIFEST_STUB,
            target=TARGET,
            board_report=board if board is not None else BOARD_REPORT_VERIFIED,
            disk_needed_mb=disk,
            layout=layout
            or {"python_min": "3.9", "prefix": "/opt/iraf-sdk", "current_link": "current"},
            allow_unverified=allow,
        )

    def kinds(self, violations):
        return sorted({item["kind"] for item in violations})

    def test_positive_control_no_violations(self):
        """正例对照：全部匹配时不得报出任何违规（否则门禁是恒失败）。"""
        self.assertEqual(self.violations(), [])

    def test_board_profile_unverified_blocks_without_flag(self):
        result = self.violations(board=BOARD_REPORT_UNVERIFIED)
        self.assertIn("board_profile", self.kinds(result))

    def test_board_profile_unverified_allowed_becomes_warning_only(self):
        result = self.violations(board=BOARD_REPORT_UNVERIFIED, allow=True)
        self.assertEqual(self.kinds(result), ["board_profile_warning"])

    def test_arch_mismatch(self):
        result = self.violations(facts={"machine": "x86_64"})
        self.assertEqual(self.kinds(result), ["arch"])

    def test_python_too_old(self):
        result = self.violations(facts={"python_version": "3.6.9", "python_major_minor": "3.6"})
        self.assertIn("python", self.kinds(result))

    def test_python_tag_mismatch(self):
        result = self.violations(facts={"python_version": "3.11.9", "python_major_minor": "3.11"})
        self.assertIn("python_tag", self.kinds(result))

    def test_glibc_too_old(self):
        result = self.violations(facts={"glibc": "2.17"})
        self.assertEqual(self.kinds(result), ["glibc"])

    def test_glibc_unknown_is_not_silently_passed(self):
        result = self.violations(facts={"glibc": None})
        self.assertEqual(self.kinds(result), ["glibc"])
        self.assertIn("无法判定", result[0]["message"])

    def test_disk_short(self):
        result = self.violations(facts={"disk_free_mb": 5}, disk=100)
        self.assertEqual(self.kinds(result), ["disk"])

    def test_disk_unknown_is_not_silently_passed(self):
        result = self.violations(facts={"disk_free_mb": None})
        self.assertEqual(self.kinds(result), ["disk"])

    def test_ram_declaration_required(self):
        result = self.violations(manifest={"board": {"limits": {"ram_mb": "unverified"}}})
        self.assertEqual(self.kinds(result), ["ram_declaration"])

    def test_ram_below_declared_limit(self):
        result = self.violations(
            facts={"mem_available_mb": 512},
            manifest={"board": {"limits": {"ram_mb": 2048}}},
        )
        self.assertEqual(self.kinds(result), ["ram"])

    def test_missing_dependencies_reported(self):
        result = self.violations(facts={"yaml_available": False, "jsonschema_available": False})
        self.assertEqual(self.kinds(result), ["dependency"])
        messages = " ".join(item["message"] for item in result)
        self.assertIn("PyYAML", messages)
        self.assertIn("jsonschema", messages)

    def test_systemd_missing(self):
        result = self.violations(facts={"systemd_available": False})
        self.assertEqual(self.kinds(result), ["systemd"])

    def test_required_disk_mb_grows_with_manifest(self):
        small = lbb.required_disk_mb({"runtime": {"size_bytes": 1024}, "files": [], "wheelhouse": {}})
        big = lbb.required_disk_mb(
            {
                "runtime": {"size_bytes": 50 * 1024 * 1024},
                "files": [{"path": "sdk/x.whl", "size_bytes": 10 * 1024 * 1024}],
                "wheelhouse": {"wheels": [{"size_bytes": 40 * 1024 * 1024}]},
            }
        )
        self.assertGreater(big, small)
        self.assertGreaterEqual(small, lbb.DISK_HEADROOM_FACTOR)


class BundleContractTest(unittest.TestCase):
    MEMBERS = {
        "board/e300.yaml": b"kind: BoardProfile\n",
        "env/iraf-sdk.env.template": b"IRAF_HOME=@PREFIX@\n",
        "scripts/install.sh": b"#!/usr/bin/env bash\nexit 0\n",
    }

    def setUp(self):
        self.tmp = Path(tempfile.mkdtemp(prefix="iraf-bundle-test-"))
        self.addCleanup(shutil.rmtree, self.tmp, ignore_errors=True)
        self.manifest = make_manifest()
        self.manifest["files"] = [
            {"path": name, "sha256": lbb.sha256_bytes(data), "size_bytes": len(data)}
            for name, data in sorted(self.MEMBERS.items())
        ]

    def _bundle(self, members=None, manifest=None) -> Path:
        return make_bundle(
            self.tmp / "out", members if members is not None else dict(self.MEMBERS),
            manifest if manifest is not None else self.manifest,
        )

    def test_positive_control_clean_bundle_passes(self):
        manifest, entries, violations = lbb.verify_bundle_contract(self._bundle())
        self.assertEqual(violations, [])
        self.assertEqual(manifest["schema_version"], lbb.BUNDLE_SCHEMA_VERSION)
        self.assertEqual(len(entries), len(self.MEMBERS) + 1)

    def test_tampered_member_detected(self):
        members = dict(self.MEMBERS)
        members["scripts/install.sh"] = b"#!/usr/bin/env bash\necho hacked\n"
        _, _, violations = lbb.verify_bundle_contract(self._bundle(members=members))
        self.assertTrue(any("被篡改" in item for item in violations), violations)

    def test_unlisted_member_detected(self):
        members = dict(self.MEMBERS)
        members["extra/unlisted.txt"] = b"x\n"
        _, _, violations = lbb.verify_bundle_contract(self._bundle(members=members))
        self.assertTrue(any("未在清单登记" in item for item in violations), violations)

    def test_missing_listed_member_detected(self):
        members = dict(self.MEMBERS)
        members.pop("scripts/install.sh")
        _, _, violations = lbb.verify_bundle_contract(self._bundle(members=members))
        self.assertTrue(any("不存在的成员" in item for item in violations), violations)

    def test_traversal_member_rejected(self):
        members = dict(self.MEMBERS)
        members["../evil.txt"] = b"escaped\n"
        with self.assertRaises(lbb.BoardBundleError) as ctx:
            lbb.verify_bundle_contract(self._bundle(members=members))
        self.assertEqual(ctx.exception.exit_code, ExitCode.VERIFY)
        self.assertIn("路径非法", str(ctx.exception))

    def test_absolute_path_member_rejected(self):
        members = dict(self.MEMBERS)
        members["/etc/cron.d/evil"] = b"*\n"
        with self.assertRaises(lbb.BoardBundleError) as ctx:
            lbb.verify_bundle_contract(self._bundle(members=members))
        self.assertIn("路径非法", str(ctx.exception))

    def test_manifest_with_absolute_path_rejected(self):
        manifest = copy.deepcopy(self.manifest)
        manifest["notes"] = ["本机路径 /home/coretek/AIIRAF"]
        _, _, violations = lbb.verify_bundle_contract(self._bundle(manifest=manifest))
        self.assertTrue(any("绝对路径" in item or "本机路径" in item for item in violations), violations)

    def test_wrong_schema_version_rejected(self):
        manifest = copy.deepcopy(self.manifest)
        manifest["schema_version"] = "iraf.board-bundle/v0"
        _, _, violations = lbb.verify_bundle_contract(self._bundle(manifest=manifest))
        self.assertTrue(any("schema_version" in item for item in violations), violations)

    def test_missing_required_section_rejected(self):
        manifest = copy.deepcopy(self.manifest)
        manifest.pop("post_verify")
        _, _, violations = lbb.verify_bundle_contract(self._bundle(manifest=manifest))
        self.assertTrue(any("缺少必需段" in item for item in violations), violations)

    def test_bundle_without_manifest_rejected(self):
        path = self.tmp / "no-manifest" / "bundle.tar.gz"
        path.parent.mkdir(parents=True, exist_ok=True)
        payload = lbb._write_deterministic_tar(
            [{"path": "board/e300.yaml", "bytes": b"x\n", "source": None}]
        )
        path.write_bytes(payload)
        with self.assertRaises(lbb.BoardBundleError) as ctx:
            lbb.read_bundle_members(path)
        self.assertEqual(ctx.exception.exit_code, ExitCode.PRECHECK)


class InstallerBehaviourTest(unittest.TestCase):
    def setUp(self):
        self.tmp = Path(tempfile.mkdtemp(prefix="iraf-installer-test-"))
        self.addCleanup(shutil.rmtree, self.tmp, ignore_errors=True)

    def _installer(self, **overrides):
        options = {
            "bundle": str(self.tmp / "bundle.tar.gz"),
            "root": str(self.tmp / "root"),
            "dry_run": True,
            "python": sys.executable,
        }
        options.update(overrides)
        return lbb.Installer(options)

    def test_rehearsal_requires_explicit_root(self):
        with self.assertRaises(lbb.BoardBundleError) as ctx:
            lbb.Installer({"bundle": "x.tar.gz", "dry_run": True})
        self.assertEqual(ctx.exception.exit_code, ExitCode.USAGE)

    def test_bundle_argument_required(self):
        with self.assertRaises(lbb.BoardBundleError) as ctx:
            lbb.Installer({"root": str(self.tmp)})
        self.assertEqual(ctx.exception.exit_code, ExitCode.USAGE)

    def test_missing_checksum_companion_is_precheck_failure(self):
        bundle = self.tmp / "x.tar.gz"
        bundle.write_bytes(b"whatever")
        installer = self._installer(bundle=str(bundle))
        with self.assertRaises(lbb.BoardBundleError) as ctx:
            installer.verify_bundle_file()
        self.assertEqual(ctx.exception.exit_code, ExitCode.PRECHECK)
        self.assertIn("伴随校验和", str(ctx.exception))

    def test_checksum_mismatch_is_verify_failure(self):
        bundle = self.tmp / "x.tar.gz"
        bundle.write_bytes(b"whatever")
        Path(str(bundle) + ".sha256").write_text(f"{'0' * 64}  x.tar.gz\n", encoding="utf-8")
        installer = self._installer(bundle=str(bundle))
        with self.assertRaises(lbb.BoardBundleError) as ctx:
            installer.verify_bundle_file()
        self.assertEqual(ctx.exception.exit_code, ExitCode.VERIFY)

    def test_rollback_restores_previous_link_and_removes_new_version(self):
        """回滚语义：恢复旧激活链接；删除本次新建的版本目录；不动旧版本目录。"""
        installer = self._installer()
        prefix = self.tmp / "root" / "opt" / "iraf-sdk"
        (prefix / "0.1.0").mkdir(parents=True, exist_ok=True)
        os.symlink("0.1.0", prefix / "current")
        version_dir = prefix / "0.2.0"
        version_dir.mkdir()
        (version_dir / "marker").write_text("new", encoding="utf-8")
        installer.layout = {"current_link": "current"}
        installer.state = {
            "version_dir": version_dir,
            "prefix_path": prefix,
            "layout": {"current_link": "current"},
            "previous": "0.1.0",
        }
        installer.report["version_dir_created"] = True
        installer.rollback()
        self.assertEqual(os.readlink(prefix / "current"), "0.1.0")
        self.assertFalse(version_dir.exists())
        self.assertTrue((prefix / "0.1.0").is_dir())
        self.assertTrue(installer.report["rolled_back"])

    def test_rollback_keeps_idempotent_directory(self):
        installer = self._installer()
        prefix = self.tmp / "root" / "opt" / "iraf-sdk"
        version_dir = prefix / "0.2.0"
        version_dir.mkdir(parents=True)
        installer.layout = {"current_link": "current"}
        installer.state = {
            "version_dir": version_dir,
            "prefix_path": prefix,
            "layout": {"current_link": "current"},
            "previous": None,
        }
        installer.report["version_dir_created"] = False
        installer.rollback()
        self.assertTrue(version_dir.is_dir(), "幂等场景下不得删除已存在的版本目录")
        self.assertFalse((prefix / "current").is_symlink())

    def test_rollback_without_state_is_noop(self):
        installer = self._installer()
        installer.rollback()
        self.assertFalse(installer.report["rolled_back"])

    def test_switch_current_refuses_non_symlink(self):
        installer = self._installer()
        prefix = self.tmp / "root" / "opt" / "iraf-sdk"
        prefix.mkdir(parents=True)
        (prefix / "current").write_text("not a link", encoding="utf-8")
        installer.layout = {"current_link": "current"}
        with self.assertRaises(lbb.BoardBundleError) as ctx:
            installer.switch_current(prefix, "current", "0.2.0")
        self.assertEqual(ctx.exception.exit_code, ExitCode.PRECHECK)

    def test_render_values_require_manifest_fields(self):
        installer = self._installer()
        installer.layout = {
            "prefix": "/opt/iraf-sdk",
            "current_link": "current",
            "config_dir": "/etc/iraf",
            "log_dir": "/var/log/iraf",
            "service_unit": "iraf-sdk-board.service",
            "env_file": "iraf-sdk.env",
            "python_bin": "/usr/bin/python3",
        }
        manifest = make_manifest()
        values = installer.render_values(manifest)
        self.assertEqual(sorted(values), sorted(lbb.PLACEHOLDERS))
        broken = copy.deepcopy(manifest)
        broken["bundle"]["version"] = None
        with self.assertRaises(lbb.BoardBundleError) as ctx:
            installer.render_values(broken)
        self.assertIn("@SDK_VERSION@", str(ctx.exception))


class BoardProfileContractTest(unittest.TestCase):
    """板卡门禁是复用 profile_check.py 的：这里只钉住"复用"与"不猜"两条。"""

    def test_run_profile_check_parses_json_and_exit_code(self):
        result = lbb.run_profile_check(REPO_ROOT, "profiles/boards/e300.yaml", sys.executable, False)
        self.assertEqual(result["exit_code"], 2, "未实测的板卡声明必须返回 2")
        summary = lbb.board_gate_summary(result)
        self.assertFalse(summary["verified"])
        self.assertGreaterEqual(len(summary["unverified_fields"]), 15)
        self.assertTrue(summary["reasons"])
        self.assertNotIn("board_path", summary, "摘要里不得带本机绝对路径（产物门禁）")

    def test_run_profile_check_allow_unverified(self):
        result = lbb.run_profile_check(
            REPO_ROOT, "profiles/boards/e300.yaml", sys.executable, True
        )
        self.assertEqual(result["exit_code"], 0)
        summary = lbb.board_gate_summary(result)
        self.assertFalse(summary["verified"])
        self.assertTrue(summary["allowed_unverified"] if "allowed_unverified" in summary else True)

    def test_missing_board_profile_fails(self):
        with self.assertRaises(ManifestError):
            lbb.read_board_profile(REPO_ROOT / "profiles" / "boards" / "not-there.yaml")


if __name__ == "__main__":
    unittest.main()
