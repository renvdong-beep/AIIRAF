"""wheelhouse 传递依赖闭包解析的正/负路径测试（离线；不访问网络）。

覆盖：
1. 目标环境推导（cp310/aarch64）与不可推导时的显式失败；
2. 可接受 wheel 标签集（cp310 + manylinux_2_28_aarch64 必须含精确标签与 py3-none-any）；
3. Requires-Dist 解析：extras 默认不纳入、--include-extra 纳入、非法条目与未知 marker 变量必须失败；
4. wheel 缺 METADATA / 多 METADATA 必须失败；
5. 递归闭包：一级缺失 → 从索引取候选 → 读其 METADATA 继续展开（用桩 fetcher，不联网）；
6. **候选清单模式保证**：矩阵与正式 wheelhouse 一个字节都不改。
"""

from __future__ import annotations

import hashlib
import io
import json
import sys
import tempfile
import unittest
import zipfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "deploy" / "sdk"))
sys.path.insert(0, str(ROOT / "src"))

import lib_wheelhouse_deps as deps  # noqa: E402


def make_wheel(path: Path, name: str, version: str, requires=(), python_tag="cp310",
               platform_tag="manylinux_2_28_aarch64", with_metadata=True, extra_metadata=False):
    dist = f"{name.replace('-', '_')}-{version}.dist-info"
    lines = [
        "Metadata-Version: 2.1",
        f"Name: {name}",
        f"Version: {version}",
    ]
    for item in requires:
        lines.append(f"Requires-Dist: {item}")
    body = "\n".join(lines) + "\n"
    with zipfile.ZipFile(path, "w") as zf:
        if with_metadata:
            zf.writestr(f"{dist}/METADATA", body)
            if extra_metadata:
                zf.writestr(f"{dist}/METADATA.orig", body)
        zf.writestr(f"{name.replace('-', '_')}/__init__.py", "")
    if with_metadata and extra_metadata:
        # 制造"同一 wheel 内两个 *.dist-info/METADATA 条目"的异常形态
        with zipfile.ZipFile(path, "a") as zf:
            zf.writestr(f"{name.replace('-', '_')}-{version}-dup.dist-info/METADATA", body)
    return path


class FakeFetcher:
    """桩 fetcher：按预设返回候选与合成 wheel，用于离线验证递归逻辑。"""

    def __init__(self, mapping, work_dir: Path):
        self.mapping = mapping          # name -> [(version, requires)]
        self.work_dir = work_dir
        self.work_dir.mkdir(parents=True, exist_ok=True)   # 与真实 fetcher 一致：暂存目录由 fetcher 建
        self.calls = []

    def candidates(self, package):
        self.calls.append(package)
        out = []
        for version, _requires in self.mapping.get(package, []):
            filename = f"{package}-{version}-cp310-cp310-manylinux_2_28_aarch64.whl"
            out.append((version, filename, f"https://example.invalid/{filename}"))
        return out

    def download(self, package, url, filename):
        version, requires = self.mapping[package][0]
        path = self.work_dir / filename
        make_wheel(path, package, version, requires)
        return path


class EnvironmentTests(unittest.TestCase):
    def test_cp310_aarch64_derives_expected_marker_environment(self):
        env = deps.build_environment({"python_tag": "cp310", "arch": "aarch64"})
        self.assertEqual(env["python_version"], "3.10")
        self.assertEqual(env["platform_machine"], "aarch64")
        self.assertEqual(env["sys_platform"], "linux")
        self.assertEqual(env["extra"], "")

    def test_py3_tag_is_refused_instead_of_guessed(self):
        with self.assertRaises(deps.ResolveError):
            deps.build_environment({"python_tag": "py3", "arch": "aarch64"})

    def test_unknown_arch_is_refused_instead_of_guessed(self):
        with self.assertRaises(deps.ResolveError):
            deps.build_environment({"python_tag": "cp310", "arch": "riscv64"})

    def test_acceptable_tags_include_declared_platform_and_pure_python(self):
        tags = deps.acceptable_tags({"python_tag": "cp310", "platform_tag": "manylinux_2_28_aarch64"})
        triples = {(t.interpreter, t.abi, t.platform) for t in tags}
        self.assertIn(("cp310", "cp310", "manylinux_2_28_aarch64"), triples)
        self.assertIn(("py3", "none", "any"), triples)
        self.assertNotIn(("cp310", "cp310", "win_arm64"), triples)


class MetadataTests(unittest.TestCase):
    def test_missing_metadata_fails_explicitly(self):
        with tempfile.TemporaryDirectory() as tmp:
            whl = make_wheel(Path(tmp) / "a-1.0.whl", "a", "1.0", with_metadata=False)
            with self.assertRaises(deps.ResolveError) as ctx:
                deps.wheel_metadata_text(whl)
            self.assertIn("METADATA", str(ctx.exception))

    def test_multiple_metadata_entries_are_refused(self):
        with tempfile.TemporaryDirectory() as tmp:
            whl = make_wheel(Path(tmp) / "a-1.0.whl", "a", "1.0", extra_metadata=True)
            with self.assertRaises(deps.ResolveError):
                deps.wheel_metadata_text(whl)

    def test_invalid_requirement_text_fails_explicitly(self):
        text = "Metadata-Version: 2.1\nName: a\nVersion: 1.0\nRequires-Dist: >>>not a spec<<<\n"
        with self.assertRaises(deps.ResolveError):
            deps.parse_requirements(text, "a-1.0.whl", {"python_version": "3.10"}, set())

    def test_unknown_marker_variable_fails_instead_of_being_dropped(self):
        text = ("Metadata-Version: 2.1\nName: a\nVersion: 1.0\n"
                'Requires-Dist: b; python_full_version >= "3.8"\n')
        with self.assertRaises(deps.ResolveError) as ctx:
            deps.parse_requirements(text, "a-1.0.whl", {"python_version": "3.10"}, set())
        self.assertIn("python_full_version", str(ctx.exception))

    def test_extras_are_excluded_by_default_and_included_on_request(self):
        text = ("Metadata-Version: 2.1\nName: a\nVersion: 1.0\n"
                "Requires-Dist: hard-dep>=1.0\n"
                'Requires-Dist: extra-dep; extra == "fmt"\n')
        env = {"python_version": "3.10", "sys_platform": "linux", "platform_machine": "aarch64", "extra": ""}
        active, skipped = deps.parse_requirements(text, "a-1.0.whl", env, set())
        self.assertEqual([r.name for r in active], ["hard-dep"])
        self.assertEqual(len(skipped), 1)

        active, skipped = deps.parse_requirements(text, "a-1.0.whl", env, {"fmt"})
        self.assertEqual(sorted(r.name for r in active), ["extra-dep", "hard-dep"])
        self.assertEqual(skipped, [])


class ClosureTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.base = Path(self.tmp.name)
        self.wheelhouse = self.base / "wheelhouse"
        self.wheelhouse.mkdir()
        # 现有 wheelhouse:左轮 a 依赖 b 与缺失的 c；b 无依赖
        make_wheel(self.wheelhouse / "a-1.0.whl", "a", "1.0", requires=["b>=1.0", "c>=2.0"])
        make_wheel(self.wheelhouse / "b-1.0.whl", "b", "1.0")
        self.target = {"id": "t", "arch": "aarch64", "python_tag": "cp310",
                       "platform_tag": "manylinux_2_28_aarch64"}
        self.env = deps.build_environment(self.target)

    def tearDown(self):
        self.tmp.cleanup()

    def test_missing_dependency_is_recursed_through_candidate_metadata(self):
        # c 依赖 d；d 也缺失 → 递归必须把 d 也解析出来
        fetcher = FakeFetcher({"c": [("2.5", ["d>=3.0"])], "d": [("3.1", [])]}, self.base / "cand")
        report = deps.resolve_closure(self.wheelhouse, self.target, self.env, fetcher,
                                      max_depth=5, include_extras=set())
        self.assertEqual(sorted(report["missing"]), ["c", "d"])
        self.assertEqual(report["missing"]["c"]["status"], "resolved")
        self.assertEqual(report["missing"]["c"]["candidate"]["version"], "2.5")
        self.assertEqual(report["missing"]["d"]["status"], "resolved")
        self.assertEqual(report["summary"]["provided_count"], 2)
        self.assertTrue(fetcher.calls)  # 确实问了索引

    def test_offline_mode_reports_missing_without_probing_index(self):
        report = deps.resolve_closure(self.wheelhouse, self.target, self.env, None,
                                      max_depth=5, include_extras=set())
        self.assertEqual(sorted(report["missing"]), ["c"])
        self.assertEqual(report["missing"]["c"]["status"], "unresolved")
        self.assertIn("offline", report["missing"]["c"]["reason"])

    def test_depth_limit_is_reported_not_silently_ignored(self):
        fetcher = FakeFetcher({"c": [("2.5", ["d>=3.0"])], "d": [("3.1", ["e>=1.0"])]}, self.base / "cand2")
        report = deps.resolve_closure(self.wheelhouse, self.target, self.env, fetcher,
                                      max_depth=1, include_extras=set())
        self.assertIn("c", report["missing"])
        self.assertIn("d", report["missing"])
        self.assertNotEqual(report["missing"]["d"]["status"], "resolved")

    def test_empty_wheelhouse_fails_explicitly(self):
        empty = self.base / "empty"
        empty.mkdir()
        with self.assertRaises(deps.ResolveError):
            deps.resolve_closure(empty, self.target, self.env, None, max_depth=1, include_extras=set())


class CandidateOnlyGuaranteeTests(unittest.TestCase):
    """核心契约：只产出候选，不改矩阵、不写正式 wheelhouse。"""

    def test_matrix_and_wheelhouse_are_untouched(self):
        with tempfile.TemporaryDirectory() as tmp:
            base = Path(tmp)
            matrix = base / "package_matrix.yaml"
            matrix.write_text(
                "schema_version: iraf.sdk-matrix/v1\n"
                "targets:\n  - id: t\n    arch: aarch64\n    python_tag: cp310\n"
                "    platform_tag: manylinux_2_28_aarch64\n    wheels: [a]\n"
                "    index_url: https://example.invalid/simple/\n",
                encoding="utf-8",
            )
            wheelhouse = base / "build" / "wheelhouse" / "t"
            wheelhouse.mkdir(parents=True)
            whl = make_wheel(wheelhouse / "a-1.0.whl", "a", "1.0", requires=["c>=2.0"])
            before_matrix = hashlib.sha256(matrix.read_bytes()).hexdigest()
            before_whl = hashlib.sha256(whl.read_bytes()).hexdigest()
            before_listing = sorted(p.name for p in wheelhouse.iterdir())

            out = base / "out"
            rc = deps.main([
                "--matrix", str(matrix), "--target", "t", "--offline",
                "--wheelhouse", str(wheelhouse),
                "--output", str(out), "--work-dir", str(base / "cand"),
            ])
            self.assertEqual(rc, 0)

            self.assertEqual(hashlib.sha256(matrix.read_bytes()).hexdigest(), before_matrix,
                             "矩阵被改写了：候选模式禁止回填")
            self.assertEqual(hashlib.sha256(whl.read_bytes()).hexdigest(), before_whl,
                             "正式 wheelhouse 内的 wheel 被改写")
            self.assertEqual(sorted(p.name for p in wheelhouse.iterdir()), before_listing,
                             "正式 wheelhouse 被写入新文件")

            report = json.loads((out / "candidates.json").read_text(encoding="utf-8"))
            self.assertEqual(report["schema_version"], "iraf.wheelhouse-deps/v1")
            self.assertTrue(report["candidate_only"])
            self.assertFalse(report["matrix_modified"])
            self.assertFalse(report["wheelhouse_written"])
            self.assertIn("c", report["missing"])
            summary = (out / "summary.md").read_text(encoding="utf-8")
            self.assertIn("未回填矩阵", summary)
            self.assertIn("回填流程", summary)

    def test_unknown_target_is_refused(self):
        with tempfile.TemporaryDirectory() as tmp:
            matrix = Path(tmp) / "m.yaml"
            matrix.write_text("schema_version: iraf.sdk-matrix/v1\ntargets:\n  - id: other\n", encoding="utf-8")
            with self.assertRaises(deps.ResolveError):
                deps.target_config(deps.load_matrix(matrix), "t")


class CandidateSelectionTests(unittest.TestCase):
    """择优语义必须与 pip 一致：预发布默认不选；平台标签走兼容阶梯。"""

    def test_platform_ladder_accepts_older_manylinux(self):
        ladder = deps.compatible_platform_tags("manylinux_2_28_aarch64")
        self.assertEqual(ladder[0], "manylinux_2_28_aarch64")
        self.assertIn("manylinux_2_17_aarch64", ladder)
        self.assertIn("manylinux2014_aarch64", ladder)
        tags = deps.acceptable_tags({"python_tag": "cp310", "platform_tag": "manylinux_2_28_aarch64"})
        triples = {(t.interpreter, t.abi, t.platform) for t in tags}
        self.assertIn(("cp310", "cp310", "manylinux_2_17_aarch64"), triples)

    def test_non_manylinux_tag_is_passed_through_unchanged(self):
        self.assertEqual(deps.compatible_platform_tags("macosx_11_0_arm64"), ["macosx_11_0_arm64"])

    def test_prerelease_is_not_selected_when_stable_exists(self):
        candidates = [("4.0.0a5", "a.whl", "u"), ("3.1.10", "b.whl", "u")]
        chosen, note = deps.select_candidate(candidates, [])
        self.assertEqual(chosen[0], "3.1.10")
        self.assertIn("排除", note)

    def test_prerelease_only_is_selected_but_flagged_for_human(self):
        candidates = [("4.16.0rc2", "a.whl", "u")]
        chosen, note = deps.select_candidate(candidates, [])
        self.assertEqual(chosen[0], "4.16.0rc2")
        self.assertIn("人工确认", note)

    def test_specifier_filters_candidates(self):
        candidates = [("3.0", "a.whl", "u"), ("2.5", "b.whl", "u"), ("2.0", "c.whl", "u")]
        chosen, _ = deps.select_candidate(candidates, [">=2.1", "<3"])
        self.assertEqual(chosen[0], "2.5")

    def test_no_candidate_matching_specifier_is_reported(self):
        candidates = [("1.0", "a.whl", "u")]
        chosen, note = deps.select_candidate(candidates, [">=2.0"])
        self.assertIsNone(chosen)
        self.assertIn("不满足版本约束", note)

    def test_pep440_sorting_beats_naive_string_sort(self):
        self.assertGreater(deps._version_key("3.1.10"), deps._version_key("3.1.9"))


if __name__ == "__main__":
    unittest.main()
