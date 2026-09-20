"""`scripts/profile_check.py --board` 的契约测试（步骤 03）。

被测契约（fail-closed）：
  1. BoardProfile 必须符合 `config/sdk/package_matrix.schema.json` 的
     `definitions.board_profile`；缺字段/未知字段/模糊占位 `tbd` 一律契约层失败（退出码 1）；
  2. `spec` 内任何 `unverified`/`pending`（含空 `capabilities`）都必须被拒绝（退出码 2），
     且中文原因必须指到**具体字段路径**，便于板卡实测后逐项回填；
  3. `--allow-unverified` 只放行第 2 类，摘要写 `verified: false`；
     不能放行契约层失败——否则它就成了"放宽门禁"的后门；
  4. `target.arch` 与产物矩阵里声明该板卡的构建目标架构必须一致；
  5. 每条负向用例都配一条**正向对照**（完整申报的 profile 必须通过），
     否则无法区分"严格门禁"与"恒失败门禁"。

运行：`PYTHONPATH=src /usr/bin/python3 -m unittest tests.unit.test_profile_check_board -v`
"""

import json
import os
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / "scripts"))

import profile_check  # noqa: E402

BOARDS_DIR = REPO_ROOT / "profiles" / "boards"
E300_PATH = BOARDS_DIR / "e300.yaml"
FIREFLY_PATH = BOARDS_DIR / "firefly_rk3588.yaml"

#: e300.yaml 当前（板卡未实测）应当被拒的字段全集（路径以 spec 为根）。
E300_EXPECTED_UNVERIFIED = [
    "status",
    "target.os",
    "target.kernel",
    "target.python_tag",
    "target.platform_tag",
    "target.glibc_min",
    "runtime.oci",
    "runtime.ros2",
    "runtime.systemd",
    "adapters.agentos_bridge",
    "adapters.npu",
    "adapters.master",
    "limits.ram_mb",
    "limits.disk_mb",
    "image.ref",
    "image.digest",
    "capabilities",
]
E300_EXPECTED_PENDING = [
    "evidence.owner",
    "evidence.acceptance_report",
]


def load_board(path):
    return yaml.safe_load(Path(path).read_text(encoding="utf-8"))


def complete_board():
    """完整申报的 BoardProfile（正向对照用）：所有字段都已实测回填。"""
    board = load_board(E300_PATH)
    spec = board["spec"]
    spec["status"] = "verified"
    spec["target"] = {
        "os": "ubuntu-22.04",
        "arch": "aarch64",
        "kernel": "5.15.0-rt",
        "python_tag": "cp310",
        "platform_tag": "manylinux_2_28_aarch64",
        "glibc_min": "2.35",
    }
    spec["runtime"] = {"oci": "available", "ros2": "available", "systemd": "available"}
    spec["adapters"] = {
        "agentos_bridge": "available",
        "npu": "unavailable",
        "master": "available",
    }
    spec["capabilities"] = ["runtime.control_plane"]
    spec["limits"] = {"ram_mb": 2048, "disk_mb": 16384}
    spec["image"] = {"ref": "iraf-board-e300", "digest": "sha256:" + "0" * 64}
    spec["evidence"] = {"owner": "irv", "acceptance_report": "build/evidence/e300.json"}
    return board


class BoardCheckTestBase(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory(prefix="iraf-board-check-")
        self.addCleanup(self._tmp.cleanup)
        self.tmpdir = Path(self._tmp.name)
        self._seq = 0

    def write_board(self, board_document):
        self._seq += 1
        path = self.tmpdir / ("board-%02d.yaml" % self._seq)
        path.write_text(
            yaml.safe_dump(board_document, allow_unicode=True, sort_keys=False),
            encoding="utf-8",
        )
        return path

    def check(self, board_document, allow_unverified=False):
        path = self.write_board(board_document)
        return profile_check.check_board(path, allow_unverified=allow_unverified)


class PositiveControlTests(BoardCheckTestBase):
    """先证明门禁能通过，否则后面的负向断言只是假门禁。"""

    def test_fully_evidenced_profile_passes_with_exit_code_0(self):
        report = self.check(complete_board())
        self.assertEqual(report["failures"], [], "完整申报的 profile 不应有契约层失败")
        self.assertEqual(report["unverified_fields"], [])
        self.assertEqual(report["pending_fields"], [])
        self.assertEqual(report["exit_code"], 0)
        self.assertTrue(report["verified"])
        self.assertTrue(report["passed"])
        self.assertFalse(report["allowed_unverified"])

    def test_board_branch_does_not_require_mujoco_at_import(self):
        """`--board` 是纯声明校验：板卡上不装仿真器也必须能跑。"""
        self.assertFalse(
            hasattr(profile_check, "mujoco"),
            "模块导入期不应绑定 mujoco（--board 不得要求仿真器）",
        )

    def test_matrix_target_is_cross_checked_and_reported(self):
        report = self.check(complete_board())
        self.assertEqual(len(report["matrix_targets"]), 1)
        self.assertEqual(report["matrix_targets"][0]["id"], "aarch64-manylinux_2_28-cp310")
        self.assertEqual(report["matrix_targets"][0]["arch"], "aarch64")


class UnverifiedRejectionTests(BoardCheckTestBase):
    """负向：未实测/未确定的声明必须被拒（退出码 2），且原因指到具体字段。"""

    def unsigned_board(self):
        """实测已回填、但尚未签字的 profile（status 仍 unverified）。

        必须用这个作变异基底：schema 自己有一条 `if status == verified then
        各字段不得为 unverified` 的约束，直接在 verified 的 profile 上把字段改回
        unverified 会同时触发契约层失败（见 ContractFailureTests 的对应用例），
        那样就测不出"未实测 -> 退出码 2"这条判据了。
        """
        board = complete_board()
        board["spec"]["status"] = "unverified"
        return board

    def test_shipped_board_profiles_are_rejected(self):
        for path in (E300_PATH, FIREFLY_PATH):
            with self.subTest(board=path.name):
                report = profile_check.check_board(path)
                self.assertEqual(report["exit_code"], 2)
                self.assertFalse(report["verified"])
                self.assertFalse(report["passed"])

    def test_e300_enumerates_exactly_the_expected_fields(self):
        report = profile_check.check_board(E300_PATH)
        self.assertEqual(sorted(report["unverified_fields"]), sorted(E300_EXPECTED_UNVERIFIED))
        self.assertEqual(sorted(report["pending_fields"]), sorted(E300_EXPECTED_PENDING))
        self.assertEqual(len(report["reasons"]), 19)

    def test_allow_unverified_passes_but_reports_verified_false(self):
        report = profile_check.check_board(E300_PATH, allow_unverified=True)
        self.assertEqual(report["exit_code"], 0)
        self.assertTrue(report["passed"])
        self.assertFalse(report["verified"])
        self.assertTrue(report["allowed_unverified"])
        # 放行不等于数据变成可信：字段清单仍要如实列出。
        self.assertEqual(sorted(report["unverified_fields"]), sorted(E300_EXPECTED_UNVERIFIED))

    def test_single_field_regression_keeps_exit_code_2_and_names_the_field(self):
        # (字段路径, 写回的值)：每个字段单独回归，证明拒绝是逐字段可定位的。
        mutations = [
            (("status",), "unverified"),
            (("target", "os"), "unverified"),
            (("target", "python_tag"), "unverified"),
            (("target", "platform_tag"), "unverified"),
            (("runtime", "oci"), "unverified"),
            (("adapters", "npu"), "unverified"),
            (("adapters", "master"), "unverified"),
            (("limits", "ram_mb"), "unverified"),
            (("image", "digest"), "unverified"),
            (("evidence", "owner"), "pending"),
            (("evidence", "acceptance_report"), "pending"),
            (("capabilities",), []),
        ]
        for path, value in mutations:
            field = ".".join(path)
            with self.subTest(field=field):
                board = self.unsigned_board()
                node = board["spec"]
                for key in path[:-1]:
                    node = node[key]
                node[path[-1]] = value
                report = self.check(board)
                self.assertEqual(report["failures"], [], "本用例只应触发未实测拒绝，不应有契约失败")
                self.assertEqual(report["exit_code"], 2, "%s 未实测却未被拒绝" % field)
                self.assertFalse(report["verified"])
                self.assertTrue(
                    any(reason.startswith(field) for reason in report["reasons"]),
                    "原因未指到字段 %s：%s" % (field, report["reasons"]),
                )


class ContractFailureTests(BoardCheckTestBase):
    """负向：契约层失败必须是 1，且 --allow-unverified 不得把它放行。"""

    def assert_contract_failure(self, board, token, allow_unverified=False):
        report = self.check(board, allow_unverified=allow_unverified)
        self.assertEqual(report["exit_code"], 1)
        self.assertFalse(report["passed"])
        joined = " | ".join(report["failures"])
        self.assertIn(token, joined, "契约失败信息未指到 %s：%s" % (token, joined))
        return report

    def test_missing_platform_tag(self):
        board = complete_board()
        del board["spec"]["target"]["platform_tag"]
        self.assert_contract_failure(board, "platform_tag")

    def test_ambiguous_placeholder_is_not_unverified(self):
        board = complete_board()
        board["spec"]["target"]["python_tag"] = "tbd"
        self.assert_contract_failure(board, "python_tag")

    def test_unknown_field(self):
        board = complete_board()
        board["spec"]["board_os"] = "ubuntu-22.04"
        self.assert_contract_failure(board, "board_os")

    def test_bad_availability_value(self):
        board = complete_board()
        board["spec"]["runtime"]["oci"] = "maybe"
        self.assert_contract_failure(board, "oci")

    def test_arch_conflicting_with_declared_matrix_target(self):
        board = complete_board()
        board["spec"]["target"]["arch"] = "x86_64"
        self.assert_contract_failure(board, "arch")

    def test_verified_status_with_a_field_fallen_back_to_unverified(self):
        """schema 自带约束：status=verified 时字段不得回退为 unverified/pending。

        这条是"未实测 -> 退出码 2"之外的第二道防线：一旦声明了 verified，
        再出现 unverified 就是契约自相矛盾，必须退出码 1 而不是被放行。
        """
        board = complete_board()
        board["spec"]["target"]["python_tag"] = "unverified"
        self.assert_contract_failure(board, "python_tag")

    def test_verified_status_with_empty_capabilities(self):
        board = complete_board()
        board["spec"]["capabilities"] = []
        self.assert_contract_failure(board, "capabilities")

    def test_allow_unverified_does_not_bypass_contract_failure(self):
        """关键安全属性：放行开关只针对未实测，不针对契约违反（否则就是放宽门禁）。"""
        board = complete_board()
        board["spec"]["target"]["python_tag"] = "tbd"
        self.assert_contract_failure(board, "python_tag", allow_unverified=True)

    def test_missing_file_is_a_contract_failure(self):
        with self.assertRaises(profile_check.BoardCheckError):
            profile_check.check_board(self.tmpdir / "not_exist.yaml")

    def test_board_name_absent_from_matrix_is_only_a_note(self):
        board = complete_board()
        board["metadata"]["name"] = "unknown_board"
        report = self.check(board)
        self.assertEqual(report["exit_code"], 0)
        self.assertEqual(report["matrix_targets"], [])
        self.assertTrue(any("产物矩阵未声明板卡" in note for note in report["notes"]))


class CliContractTests(unittest.TestCase):
    """命令行退出码契约（步骤 03 验收块里实际跑的三条）。"""

    def run_cli(self, *argv):
        env = dict(os.environ)
        env["PYTHONPATH"] = "src"
        completed = subprocess.run(
            [sys.executable, "scripts/profile_check.py", *argv],
            cwd=str(REPO_ROOT),
            env=env,
            capture_output=True,
            text=True,
        )
        return completed

    def test_board_rejected_with_exit_code_2(self):
        completed = self.run_cli("--board", "profiles/boards/e300.yaml")
        self.assertEqual(completed.returncode, 2, completed.stderr)
        report = json.loads(completed.stdout)
        self.assertFalse(report["verified"])
        self.assertEqual(report["field_scope"], "spec")
        self.assertIn("BOARD_PROFILE_CHECK_REJECTED_UNVERIFIED", completed.stderr)
        self.assertIn("target.python_tag", completed.stderr)

    def test_board_allowed_with_exit_code_0_and_verified_false(self):
        completed = self.run_cli("--board", "profiles/boards/e300.yaml", "--allow-unverified")
        self.assertEqual(completed.returncode, 0, completed.stderr)
        report = json.loads(completed.stdout)
        self.assertEqual(report["exit_code"], 0)
        self.assertFalse(report["verified"])
        self.assertTrue(report["allowed_unverified"])
        self.assertIn("BOARD_PROFILE_CHECK_PASSED_ALLOW_UNVERIFIED", completed.stderr)

    def test_allow_unverified_without_board_is_a_usage_error(self):
        completed = self.run_cli("--allow-unverified")
        self.assertEqual(completed.returncode, 1)
        self.assertIn("只对 --board 生效", completed.stderr)

    def test_no_arguments_is_a_usage_error(self):
        completed = self.run_cli()
        self.assertEqual(completed.returncode, 1)
        self.assertIn("请给出 --baseline", completed.stderr)

    def test_missing_board_file_exits_1(self):
        completed = self.run_cli("--board", "profiles/boards/not_exist.yaml")
        self.assertEqual(completed.returncode, 1)
        self.assertIn("BOARD_PROFILE_CHECK_FAILED", completed.stderr)


if __name__ == "__main__":
    unittest.main()
