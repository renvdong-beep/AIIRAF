"""产物矩阵与 BoardProfile 的 schema 契约测试（步骤 02）。

被测契约：`config/sdk/package_matrix.schema.json`
  - 根结构        -> `config/sdk/package_matrix.yaml`（产物矩阵）
  - definitions.board_profile -> `profiles/boards/*.yaml`（自包含子契约，须整体提取后单独校验）

设计要点（fail-closed）：
  1. 未实测字段只能写 `unverified`；`pending/tbd/unknown` 等模糊占位必须被拒绝（AGENTS.md 铁律 3）。
  2. `status: verified` 必须同时满足：目标字段与容量已回填、镜像摘要已声明、能力非空、
     `evidence.owner/acceptance_report` 均非 `pending`——即未验收不得声明已验证。
  3. 矩阵不得写死官方 PyPI 源（实测不可达），`index_url` 必须显式声明为可达镜像。
  4. 矩阵声明的板卡必须在 `profiles/boards/<name>.yaml` 真实存在（声明与产物一致）。

运行：`PYTHONPATH=src python3 -m unittest tests.unit.test_board_profile_schema -v`
"""

import copy
import json
import unittest
from pathlib import Path

import yaml
from jsonschema import Draft7Validator
from jsonschema.exceptions import ValidationError

REPO_ROOT = Path(__file__).resolve().parents[2]
SCHEMA_PATH = REPO_ROOT / "config" / "sdk" / "package_matrix.schema.json"
MATRIX_PATH = REPO_ROOT / "config" / "sdk" / "package_matrix.yaml"
BOARDS_DIR = REPO_ROOT / "profiles" / "boards"
BOARD_NAMES = ("e300", "firefly_rk3588")
OFFICIAL_PYPI_MARKER = "pypi.org"


def load_yaml(path):
    return yaml.safe_load(Path(path).read_text(encoding="utf-8"))


def load_schema():
    return json.loads(SCHEMA_PATH.read_text(encoding="utf-8"))


def board_profile_validator():
    """BoardProfile 子契约是自包含的（内部只用相对 $ref），因此整体提取后单独校验。"""
    schema = load_schema()
    board_schema = copy.deepcopy(schema["definitions"]["board_profile"])
    board_schema["$schema"] = schema["$schema"]
    return Draft7Validator(board_schema)


def first_error(validator, document):
    errors = sorted(validator.iter_errors(document), key=lambda item: [str(part) for part in item.path])
    return errors[0] if errors else None


class SchemaContractTests(unittest.TestCase):
    """先证明 schema 本身是合法 draft-07。"""

    def test_schema_documents_are_valid_draft7(self):
        schema = load_schema()
        Draft7Validator.check_schema(schema)
        board_schema = copy.deepcopy(schema["definitions"]["board_profile"])
        Draft7Validator.check_schema(board_schema)

    def test_schema_is_declared_as_draft7(self):
        self.assertEqual(load_schema()["$schema"], "http://json-schema.org/draft-07/schema#")


class PackageMatrixTests(unittest.TestCase):
    def setUp(self):
        self.schema = load_schema()
        self.matrix = load_yaml(MATRIX_PATH)
        self.validator = Draft7Validator(self.schema)

    def test_matrix_matches_schema(self):
        error = first_error(self.validator, self.matrix)
        self.assertIsNone(error, msg=f"矩阵不符合 schema：{error}" if error else "")

    def test_matrix_declares_reachable_mirror_and_never_official_pypi(self):
        raw = MATRIX_PATH.read_text(encoding="utf-8")
        self.assertNotIn(
            OFFICIAL_PYPI_MARKER,
            raw,
            "矩阵中不得出现官方 PyPI 地址（实测不可达，且决策 2 要求镜像源可声明替换）",
        )
        for index, target in enumerate(self.matrix["targets"]):
            with self.subTest(target=target["id"]):
                self.assertTrue(
                    target["index_url"].startswith("https://"),
                    f"targets[{index}] 必须显式声明 https 索引地址",
                )
                self.assertRegex(target["index_url_declared_at"], r"^\d{4}-\d{2}-\d{2}$")
                self.assertIn("index_url_note", target)

    def test_matrix_rejects_empty_index_url(self):
        """空字符串 = 未声明，必须被 schema 拒绝（禁止静默回落到官方源）。"""
        broken = copy.deepcopy(self.matrix)
        broken["targets"][0]["index_url"] = ""
        self.assertIsNotNone(first_error(self.validator, broken))

    def test_matrix_rejects_index_url_that_is_official_pypi(self):
        broken = copy.deepcopy(self.matrix)
        broken["targets"][0]["index_url"] = "https://pypi.org/simple/"
        self.assertIsNotNone(first_error(self.validator, broken))

    def test_matrix_rejects_uppercase_distribution_name(self):
        """实测陷阱：镜像索引名区分大小写，PyYAML 返回 404，只有小写 pyyaml 存在。"""
        broken = copy.deepcopy(self.matrix)
        broken["targets"][0]["wheels"] = ["PyYAML"]
        self.assertIsNotNone(first_error(self.validator, broken))

    def test_matrix_declares_platform_tag_and_python_tag(self):
        target = self.matrix["targets"][0]
        self.assertEqual(target["platform_tag"], "manylinux_2_28_aarch64")
        self.assertEqual(target["python_tag"], "cp310")
        self.assertIn("win_arm64", target["reject_platform_tags"])

    def test_every_declared_board_has_a_profile_file(self):
        for target in self.matrix["targets"]:
            for board in target["boards"]:
                with self.subTest(board=board):
                    path = BOARDS_DIR / f"{board}.yaml"
                    self.assertTrue(path.is_file(), f"矩阵声明了板卡 {board}，但缺少 {path}")
                    self.assertEqual(load_yaml(path)["metadata"]["name"], board)


class BoardProfileTests(unittest.TestCase):
    def setUp(self):
        self.validator = board_profile_validator()
        self.profiles = {name: load_yaml(BOARDS_DIR / f"{name}.yaml") for name in BOARD_NAMES}

    def test_board_profiles_match_schema(self):
        for name, profile in self.profiles.items():
            with self.subTest(board=name):
                error = first_error(self.validator, profile)
                self.assertIsNone(error, msg=f"{name} 不符合 BoardProfile schema：{error}")

    def test_board_profiles_are_unverified_before_acceptance(self):
        for name, profile in self.profiles.items():
            with self.subTest(board=name):
                spec = profile["spec"]
                self.assertEqual(spec["status"], "unverified")
                self.assertEqual(spec["target"]["arch"], "aarch64")
                self.assertEqual(spec["evidence"]["owner"], "pending")
                self.assertEqual(spec["evidence"]["acceptance_report"], "pending")
                # 空能力清单 = 未声明能力，预检必须失败而不是默认放行。
                self.assertEqual(spec["capabilities"], [])

    def test_board_profiles_keep_unknown_fields_unverified(self):
        """未实测字段一律 unverified；禁止 pending/tbd/unknown 等模糊占位。"""
        raw_unverified_min = 6
        for name, profile in self.profiles.items():
            with self.subTest(board=name):
                spec = profile["spec"]
                for field, value in spec["target"].items():
                    if field == "arch":
                        continue
                    self.assertEqual(value, "unverified", f"{name}.spec.target.{field} 未实测即须 unverified")
                for field, value in spec["limits"].items():
                    self.assertEqual(value, "unverified", f"{name}.spec.limits.{field} 未实测即须 unverified")
                for section in ("runtime", "adapters"):
                    for field, value in spec[section].items():
                        self.assertEqual(
                            value,
                            "unverified",
                            f"{name}.spec.{section}.{field} 未实测即须 unverified（unavailable 也需实测证据）",
                        )
                self.assertEqual(spec["image"]["digest"], "unverified")
                lines = (BOARDS_DIR / f"{name}.yaml").read_text(encoding="utf-8").count("unverified")
                self.assertGreaterEqual(
                    lines,
                    raw_unverified_min,
                    f"{name} 的 unverified 字段行数 {lines} < {raw_unverified_min}",
                )

    def test_board_profile_carries_no_hyper_vm_details(self):
        """铁律 6.10：VM 数量/编号/IP/OS/角色映射只从签名 HyperProfile 读取，禁止硬编码在本文件。"""
        forbidden = ("10.203.247.", "192.168.", "vm_count", "vm_number", "roles:")
        for name in BOARD_NAMES:
            with self.subTest(board=name):
                raw = (BOARDS_DIR / f"{name}.yaml").read_text(encoding="utf-8")
                for token in forbidden:
                    self.assertNotIn(token, raw, f"{name} 出现了禁止硬编码的构型字段/地址：{token}")

    # --- 负向用例 1：删掉 target.platform_tag -> 校验必须失败 ---
    def test_negative_missing_platform_tag_fails(self):
        broken = copy.deepcopy(self.profiles["e300"])
        del broken["spec"]["target"]["platform_tag"]
        error = first_error(self.validator, broken)
        self.assertIsNotNone(error, "缺少 target.platform_tag 却通过了校验（声明缺失必须 fail-closed）")
        self.assertIn("platform_tag", str(error))

    # --- 负向用例 2：verified 但验收报告仍是 pending -> 校验必须失败 ---
    def test_negative_verified_without_acceptance_report_fails(self):
        broken = copy.deepcopy(self.profiles["e300"])
        broken["spec"]["status"] = "verified"
        self.assertEqual(broken["spec"]["evidence"]["acceptance_report"], "pending")
        error = first_error(self.validator, broken)
        self.assertIsNotNone(error, "未验收却声明 verified 通过了校验（禁止伪造已完成）")

    def test_negative_verified_with_unfilled_limits_fails(self):
        broken = copy.deepcopy(self.profiles["e300"])
        broken["spec"]["status"] = "verified"
        broken["spec"]["evidence"] = {"owner": "irv", "acceptance_report": "build/evidence/e300.json"}
        error = first_error(self.validator, broken)
        self.assertIsNotNone(error, "verified 但 limits 仍为 unverified 却通过校验")

    def test_negative_ambiguous_placeholder_value_fails(self):
        broken = copy.deepcopy(self.profiles["e300"])
        broken["spec"]["target"]["python_tag"] = "tbd"
        error = first_error(self.validator, broken)
        self.assertIsNotNone(error, "python_tag=tbd 通过了校验（未知只能写 unverified）")

    def test_negative_unknown_key_fails(self):
        broken = copy.deepcopy(self.profiles["e300"])
        broken["spec"]["board_os"] = "ubuntu-22.04"
        self.assertIsNotNone(first_error(self.validator, broken), "未知字段被静默接受")

    def test_negative_bad_availability_value_fails(self):
        broken = copy.deepcopy(self.profiles["e300"])
        broken["spec"]["adapters"]["npu"] = "maybe"
        self.assertIsNotNone(first_error(self.validator, broken))

    # --- 正向对照：证明上面两条负向用例不是"恒失败"的假门禁 ---
    def test_verified_passes_once_every_requirement_is_evidenced(self):
        complete = copy.deepcopy(self.profiles["e300"])
        complete["spec"]["status"] = "verified"
        complete["spec"]["target"] = {
            "os": "ubuntu-22.04",
            "arch": "aarch64",
            "kernel": "5.15.0-rt",
            "python_tag": "cp310",
            "platform_tag": "manylinux_2_28_aarch64",
            "glibc_min": "2.35",
        }
        complete["spec"]["limits"] = {"ram_mb": 2048, "disk_mb": 16384}
        complete["spec"]["image"] = {"ref": "iraf-board-e300", "digest": "sha256:" + "0" * 64}
        complete["spec"]["capabilities"] = ["runtime.control_plane"]
        complete["spec"]["evidence"] = {"owner": "irv", "acceptance_report": "build/evidence/e300.json"}
        error = first_error(self.validator, complete)
        self.assertIsNone(error, msg=f"完整申报的 verified profile 被误拒：{error}")


if __name__ == "__main__":
    unittest.main()
