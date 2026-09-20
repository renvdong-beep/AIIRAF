"""`deploy/sdk/fetch_wheelhouse.sh` + `deploy/sdk/check_wheel_tags.py` 的契约测试（步骤 07）。

被测契约（fail-closed，全部**离线**可跑，不访问网络）：
  1. 平台标签过滤：实测镜像索引混有 `win_arm64` / `macosx` 干扰项，
     `numpy-…-win_arm64.whl` 必须被拒绝、`mujoco-…-manylinux_2_28_aarch64.whl` 必须通过；
     此外 `x86_64`（架构不符）、`musllinux_1_1_aarch64`（libc 不符）、
     `manylinux_2_34_aarch64`（glibc 要求高于目标）也必须被拒绝。
  2. 解释器标签：`cp39` 不得当作 `cp310` 的候选；`py3-none-any` 只在 univeral wheel 语义下通过。
  3. 声明 fail-closed：`index_url` 为空/非 http(s)、`platform_tag` 仍为 `unverified`、
     缺 `reject_platform_tags`、缺 `wheels`、目标 id 不存在、多目标未指定 —— 全部**退出码 2**，
     且不得回落到任何默认源（不回落官方 PyPI）。
  4. 候选选择：版本优先（更新的纯 Python wheel 优于更旧的二进制 wheel）、正式版优于预发布版，
     同一版本内平台精确匹配优于兼容 manylinux；`pins` 生效且非法 pin 形式被拒绝。
  5. 索引页解析：相对 href（`../../packages/<hash>/<file>.whl`）必须按页面 URL 解析为绝对地址。
  6. 下载与清单：写 `.part` 再改名、校验 ZIP 结构（必须含 `dist-info/METADATA`）、
     记录 SHA-256/字节数/选择理由，并写 `wheelhouse.json`（`iraf.wheelhouse/v1`）。
  7. 目录校验与 shell 入口：混入错标签的目录必须退出 4；全对必须退出 0（正向对照，避免
     "恒失败的门禁"）；`--dry-run` 不写盘；用法错误退出 1。

运行：`PYTHONPATH=src /usr/bin/python3 -m unittest tests.unit.test_wheel_tags -v`
"""

import hashlib
import json
import shutil
import subprocess
import sys
import tempfile
import unittest
import zipfile
from pathlib import Path
from unittest import mock

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / "deploy" / "sdk"))

import check_wheel_tags as cwt  # noqa: E402

SCRIPT = REPO_ROOT / "deploy" / "sdk" / "fetch_wheelhouse.sh"
MATRIX = REPO_ROOT / "config" / "sdk" / "package_matrix.yaml"

GOOD_MUJOCO = "mujoco-3.9.0-cp310-cp310-manylinux_2_28_aarch64.whl"
GOOD_MUJOCO_OLD = "mujoco-3.5.0-cp310-cp310-manylinux_2_17_aarch64.whl"
BAD_WIN = "numpy-2.3.0-cp310-cp310-win_arm64.whl"
BAD_MACOS = "Pillow-10.0.0-cp310-cp310-macosx_11_0_arm64.whl"
BAD_X86 = "mujoco-3.9.0-cp310-cp310-manylinux_2_28_x86_64.whl"
BAD_MUSL = "Pillow-10.0.0-cp310-cp310-musllinux_1_1_aarch64.whl"
BAD_GLIBC = "mujoco-3.9.0-cp310-cp310-manylinux_2_34_aarch64.whl"
BAD_PY39 = "mujoco-3.9.0-cp39-cp39-manylinux_2_28_aarch64.whl"
PURE_PY = "protobuf-7.36.1-py3-none-any.whl"

MATRIX_DOC = {
    "schema_version": "iraf.sdk-matrix/v1",
    "targets": [
        {
            "id": "aarch64-manylinux_2_28-cp310",
            "arch": "aarch64",
            "platform_tag": "manylinux_2_28_aarch64",
            "python_tag": "cp310",
            "pure_python": ["jsonschema"],
            "wheels": ["mujoco", "numpy"],
            "index_url": "https://mirrors.example.invalid/pypi/simple/",
            "reject_platform_tags": ["win_arm64", "win32", "win_amd64", "macosx"],
            "boards": ["e300"],
        }
    ],
}


def make_spec(**overrides) -> cwt.TargetSpec:
    return cwt.resolve_target({**MATRIX_DOC, "targets": [{**MATRIX_DOC["targets"][0], **overrides}]})


def make_wheel(path: Path, filename: str) -> Path:
    """造一个结构合法（含 dist-info/METADATA）的假 wheel，供下载/校验路径离线使用。"""
    path.parent.mkdir(parents=True, exist_ok=True)
    stem = filename[: -len(".whl")]
    dist, version = stem.split("-")[0], stem.split("-")[1]
    with zipfile.ZipFile(path, "w") as archive:
        archive.writestr(f"{dist}-{version}.dist-info/METADATA", f"Name: {dist}\nVersion: {version}\n")
        archive.writestr(f"{dist}/__init__.py", "")
    return path


def index_html(*hrefs: str) -> str:
    """仿 PEP 503 simple 索引页（aliyun 镜像实测形态：相对 href + 混入他平台条目）。"""
    body = "".join(f'    <a href="{href}">{href.rsplit("/", 1)[-1]}</a><br/>\n' for href in hrefs)
    return (
        "<!DOCTYPE html>\n<html>\n<head><title>Links for pkg</title></head>\n<body>\n"
        f"{body}</body></html>\n"
    )


class TestWheelFilename(unittest.TestCase):
    """文件名解析（PEP 427/425）。"""

    def test_解析普通文件名(self):
        wheel = cwt.parse_wheel_filename(GOOD_MUJOCO)
        self.assertEqual(wheel.distribution, "mujoco")
        self.assertEqual(wheel.version, "3.9.0")
        self.assertEqual(wheel.python_tags, ("cp310",))
        self.assertEqual(wheel.platform_tags, ("manylinux_2_28_aarch64",))
        self.assertIsNone(wheel.build_tag)

    def test_解析压缩标签集(self):
        wheel = cwt.parse_wheel_filename(
            "numpy-2.2.6-cp310-cp310-manylinux_2_17_aarch64.manylinux2014_aarch64.whl"
        )
        self.assertEqual(
            wheel.platform_tags, ("manylinux_2_17_aarch64", "manylinux2014_aarch64")
        )
        self.assertFalse(wheel.is_pure_python)

    def test_解析带_build_段(self):
        wheel = cwt.parse_wheel_filename("mujoco-3.9.0-1-cp310-cp310-manylinux_2_28_aarch64.whl")
        self.assertEqual(wheel.build_tag, "1")
        self.assertEqual(wheel.version, "3.9.0")

    def test_归一化名称(self):
        self.assertEqual(cwt.normalize_name("PyYAML"), "pyyaml")
        self.assertEqual(cwt.normalize_name("jsonschema_specifications"), "jsonschema-specifications")

    def test_sdist_被拒绝(self):
        with self.assertRaises(cwt.TagViolation) as ctx:
            cwt.parse_wheel_filename("mujoco-3.9.0.tar.gz")
        self.assertIn("只接受 .whl", str(ctx.exception))

    def test_结构非法被拒绝(self):
        with self.assertRaises(cwt.TagViolation):
            cwt.parse_wheel_filename("mujoco-3.9.0-cp310.whl")


class TestPlatformAndPythonTags(unittest.TestCase):
    """标签过滤门禁：错平台/错解释器/glibc 过高都必须被拒（步骤「负向单测」要求）。"""

    def setUp(self):
        self.spec = make_spec()

    def test_正例_目标平台精确匹配通过(self):
        verdict = cwt.classify_wheel(GOOD_MUJOCO, self.spec)
        self.assertTrue(verdict["accepted"], verdict["reason"])
        self.assertEqual(verdict["kind"], "exact")

    def test_负例_win_arm64_必须被拒绝(self):
        """步骤明文要求：喂入 numpy-…-win_arm64.whl 必须失败。"""
        verdict = cwt.classify_wheel(BAD_WIN, self.spec)
        self.assertFalse(verdict["accepted"])
        self.assertIn("win_arm64", verdict["reason"])

    def test_负例_macosx_必须被拒绝(self):
        verdict = cwt.classify_wheel(BAD_MACOS, self.spec)
        self.assertFalse(verdict["accepted"])
        self.assertIn("macosx", verdict["reason"])

    def test_负例_x86_64_架构不符被拒绝(self):
        verdict = cwt.classify_wheel(BAD_X86, self.spec)
        self.assertFalse(verdict["accepted"])
        self.assertIn("架构", verdict["reason"])

    def test_负例_musllinux_被拒绝(self):
        verdict = cwt.classify_wheel(BAD_MUSL, self.spec)
        self.assertFalse(verdict["accepted"])

    def test_负例_glibc_要求高于目标被拒绝(self):
        verdict = cwt.classify_wheel(BAD_GLIBC, self.spec)
        self.assertFalse(verdict["accepted"])
        self.assertIn("平台标签与声明不符", verdict["reason"])

    def test_正例_低版本_glibc_同架构可安装(self):
        verdict = cwt.classify_wheel(GOOD_MUJOCO_OLD, self.spec)
        self.assertTrue(verdict["accepted"], verdict["reason"])
        self.assertEqual(verdict["kind"], "compatible")

    def test_负例_解释器标签不匹配被拒绝(self):
        verdict = cwt.classify_wheel(BAD_PY39, self.spec)
        self.assertFalse(verdict["accepted"])
        self.assertIn("解释器标签不匹配", verdict["reason"])

    def test_正例_纯_python_标记为_pure_python(self):
        verdict = cwt.classify_wheel(PURE_PY, self.spec)
        self.assertTrue(verdict["accepted"], verdict["reason"])
        self.assertEqual(verdict["kind"], "pure_python")

    def test_正向对照_门禁不是恒失败(self):
        """三条正例同时通过：确保上面的拒绝不是因为"门禁永远失败"。"""
        verdicts = [cwt.classify_wheel(name, self.spec) for name in (GOOD_MUJOCO, GOOD_MUJOCO_OLD, PURE_PY)]
        self.assertEqual([v["accepted"] for v in verdicts], [True, True, True])


class TestIndexParsingAndSelection(unittest.TestCase):
    """索引页解析与候选选择（离线 fixture，不联网）。"""

    PAGE_URL = "https://mirrors.example.invalid/pypi/simple/numpy/"

    def test_相对_href_解析为绝对地址(self):
        html = index_html("../../packages/64/1d/abc/numpy-2.2.6-cp310-cp310-manylinux_2_17_aarch64.whl")
        urls = cwt.parse_index_links(html, self.PAGE_URL)
        self.assertEqual(
            urls,
            [
                "https://mirrors.example.invalid/pypi/packages/64/1d/abc/"
                "numpy-2.2.6-cp310-cp310-manylinux_2_17_aarch64.whl"
            ],
        )

    def test_非_wheel_链接被忽略且片段被剥离(self):
        html = index_html(
            "../../packages/x/numpy-2.2.6.tar.gz",
            "../../packages/x/numpy-2.2.6-cp310-cp310-manylinux_2_28_aarch64.whl#sha256=deadbeef",
            "https://example.invalid/readme.html",
        )
        urls = cwt.parse_index_links(html, self.PAGE_URL)
        self.assertEqual(len(urls), 1)
        self.assertNotIn("#sha256", urls[0])

    def test_选择结果绝不落在干扰平台上(self):
        """实测形态：更新的 win_arm64 版本 + 较旧的 aarch64 版本 ⇒ 必须选 aarch64。"""
        html = index_html(
            "../../packages/a/numpy-9.9.9-cp310-cp310-win_arm64.whl",
            "../../packages/b/numpy-2.2.6-cp310-cp310-manylinux_2_17_aarch64.whl",
        )
        urls = cwt.parse_index_links(html, self.PAGE_URL)
        candidate, rejected = cwt.select_candidate(urls, make_spec(), "numpy")
        self.assertIsNotNone(candidate)
        self.assertEqual(candidate.filename, "numpy-2.2.6-cp310-cp310-manylinux_2_17_aarch64.whl")
        self.assertTrue(any("win_arm64" in item["reason"] for item in rejected))

    def test_版本优先_更新的纯_python_优于更旧二进制(self):
        html = index_html(
            "../../packages/a/protobuf-4.25.7-cp310-cp310-manylinux_2_28_aarch64.whl",
            "../../packages/b/protobuf-7.36.1-py3-none-any.whl",
        )
        urls = cwt.parse_index_links(html, self.PAGE_URL)
        candidate, _ = cwt.select_candidate(urls, make_spec(), "protobuf")
        self.assertEqual(candidate.filename, "protobuf-7.36.1-py3-none-any.whl")

    def test_同版本内平台精确优先于纯_python(self):
        html = index_html(
            "../../packages/a/mujoco-3.9.0-py3-none-any.whl",
            "../../packages/b/mujoco-3.9.0-cp310-cp310-manylinux_2_28_aarch64.whl",
        )
        urls = cwt.parse_index_links(html, self.PAGE_URL)
        candidate, _ = cwt.select_candidate(urls, make_spec(), "mujoco")
        self.assertEqual(candidate.kind, "exact")
        self.assertEqual(candidate.filename, GOOD_MUJOCO)

    def test_预发布版本被降级(self):
        html = index_html(
            "../../packages/a/mujoco-3.9.1rc1-cp310-cp310-manylinux_2_28_aarch64.whl",
            "../../packages/b/mujoco-3.9.0-cp310-cp310-manylinux_2_28_aarch64.whl",
        )
        urls = cwt.parse_index_links(html, self.PAGE_URL)
        candidate, _ = cwt.select_candidate(urls, make_spec(), "mujoco")
        self.assertEqual(candidate.version, "3.9.0")

    def test_pins_约束版本(self):
        spec = make_spec(pins={"mujoco": "==3.5.0"})
        html = index_html(
            "../../packages/a/mujoco-3.9.0-cp310-cp310-manylinux_2_28_aarch64.whl",
            "../../packages/b/mujoco-3.5.0-cp310-cp310-manylinux_2_17_aarch64.whl",
        )
        urls = cwt.parse_index_links(html, self.PAGE_URL)
        candidate, rejected = cwt.select_candidate(urls, spec, "mujoco")
        self.assertEqual(candidate.version, "3.5.0")
        self.assertTrue(all("pins" in item["reason"] for item in rejected))

    def test_非法_pin_形式被拒绝(self):
        with self.assertRaises(cwt.DeclarationError):
            make_spec(pins={"mujoco": ">=3.5.0"})

    def test_无可选候选返回_None(self):
        html = index_html("../../packages/a/numpy-2.3.0-cp310-cp310-win_arm64.whl")
        urls = cwt.parse_index_links(html, self.PAGE_URL)
        candidate, rejected = cwt.select_candidate(urls, make_spec(), "numpy")
        self.assertIsNone(candidate)
        self.assertEqual(len(rejected), 1)


class TestMatrixDeclaration(unittest.TestCase):
    """声明读取的 fail-closed 行为（真实矩阵 + 临时矩阵）。"""

    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory(prefix="iraf-wheel-tags-")
        self.tmp = Path(self._tmp.name)

    def tearDown(self):
        self._tmp.cleanup()

    def write_matrix(self, document: dict) -> Path:
        import yaml

        path = self.tmp / "package_matrix.yaml"
        path.write_text(yaml.safe_dump(document, allow_unicode=True), encoding="utf-8")
        return path

    def test_真实矩阵可解析且与步骤_02_声明一致(self):
        document = cwt.load_matrix(MATRIX)
        spec = cwt.resolve_target(document)
        self.assertEqual(spec.id, "aarch64-manylinux_2_28-cp310")
        self.assertEqual(spec.platform_tag, "manylinux_2_28_aarch64")
        self.assertEqual(spec.python_tag, "cp310")
        self.assertTrue(spec.index_url.startswith("https://"))
        self.assertIn("win_arm64", spec.reject_platform_tags)
        self.assertIn("macosx", spec.reject_platform_tags)
        self.assertIn("mujoco", spec.wheels)

    def test_缺文件即预检失败(self):
        with self.assertRaises(cwt.DeclarationError):
            cwt.load_matrix(self.tmp / "不存在.yaml")

    def test_schema_版本不符即失败(self):
        path = self.write_matrix({"schema_version": "x", "targets": []})
        with self.assertRaises(cwt.DeclarationError):
            cwt.load_matrix(path)

    def test_index_url_为空即预检失败(self):
        path = self.write_matrix(
            {**MATRIX_DOC, "targets": [{**MATRIX_DOC["targets"][0], "index_url": ""}]}
        )
        with self.assertRaises(cwt.DeclarationError) as ctx:
            cwt.resolve_target(cwt.load_matrix(path))
        message = str(ctx.exception)
        self.assertIn("index_url", message)
        self.assertIn("内网私有源", message)

    def test_index_url_非_http_即预检失败(self):
        path = self.write_matrix(
            {**MATRIX_DOC, "targets": [{**MATRIX_DOC["targets"][0], "index_url": "file:///tmp/pypi"}]}
        )
        with self.assertRaises(cwt.DeclarationError):
            cwt.resolve_target(cwt.load_matrix(path))

    def test_目标标签仍为_unverified_即预检失败(self):
        for key in ("platform_tag", "python_tag", "arch"):
            with self.subTest(key=key):
                with self.assertRaises(cwt.DeclarationError) as ctx:
                    make_spec(**{key: "unverified"})
                self.assertIn(key, str(ctx.exception))

    def test_缺_reject_platform_tags_即失败(self):
        document = {**MATRIX_DOC, "targets": [{**MATRIX_DOC["targets"][0]}]}
        document["targets"][0].pop("reject_platform_tags")
        with self.assertRaises(cwt.DeclarationError) as ctx:
            cwt.resolve_target(document)
        self.assertIn("reject_platform_tags", str(ctx.exception))

    def test_缺_wheels_即失败(self):
        document = {**MATRIX_DOC, "targets": [{**MATRIX_DOC["targets"][0]}]}
        document["targets"][0].pop("wheels")
        with self.assertRaises(cwt.DeclarationError):
            cwt.resolve_target(document)

    def test_未知_target_id_即失败(self):
        with self.assertRaises(cwt.DeclarationError):
            cwt.resolve_target(MATRIX_DOC, "不存在的目标")

    def test_多目标未指定_target_即失败(self):
        document = {
            **MATRIX_DOC,
            "targets": [MATRIX_DOC["targets"][0], {**MATRIX_DOC["targets"][0], "id": "second"}],
        }
        with self.assertRaises(cwt.DeclarationError) as ctx:
            cwt.resolve_target(document)
        self.assertIn("--target", str(ctx.exception))

    def test_包清单保序去重并归一化(self):
        spec = make_spec(wheels=["PyYAML", "pyyaml", "mujoco"], pure_python=["jsonschema"])
        self.assertEqual(spec.packages, ("pyyaml", "mujoco", "jsonschema"))


class TestDownloadAndManifest(unittest.TestCase):
    """下载路径（用 file:// 本地 wheel 离线驱动）、ZIP 校验与清单字段。"""

    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory(prefix="iraf-wheel-fetch-")
        self.tmp = Path(self._tmp.name)
        self.spec = make_spec(wheels=["mujoco"], pure_python=[])

    def tearDown(self):
        self._tmp.cleanup()

    def test_下载写入_sha256_与清单(self):
        source = make_wheel(self.tmp / "src" / GOOD_MUJOCO, GOOD_MUJOCO)
        candidate = cwt.Candidate(
            filename=GOOD_MUJOCO,
            url=source.as_uri(),
            version="3.9.0",
            kind="exact",
            kind_rank=0,
            version_key=(3, 9, 0, 0, 0, 0),
            reason="平台标签与声明一致",
            prerelease=False,
        )
        dest = self.tmp / "dest"
        dest.mkdir()
        entry = cwt.download_candidate(candidate, dest, timeout=10, retries=0, deadline=1e9)
        self.assertEqual(entry["name"], "mujoco")
        self.assertEqual(entry["version"], "3.9.0")
        self.assertEqual(entry["size_bytes"], (dest / GOOD_MUJOCO).stat().st_size)
        self.assertEqual(entry["sha256"], hashlib.sha256((dest / GOOD_MUJOCO).read_bytes()).hexdigest())
        self.assertEqual(list(dest.glob("*.part")), [])

    def test_非_wheel_下载物被拒绝(self):
        source = self.tmp / "src" / GOOD_MUJOCO
        source.parent.mkdir(parents=True, exist_ok=True)
        source.write_bytes(b"not a zip")
        candidate = cwt.Candidate(
            filename=GOOD_MUJOCO,
            url=source.as_uri(),
            version="3.9.0",
            kind="exact",
            kind_rank=0,
            version_key=(3, 9, 0, 0, 0, 0),
            reason="",
            prerelease=False,
        )
        dest = self.tmp / "dest"
        dest.mkdir()
        with self.assertRaises(cwt.TagViolation):
            cwt.download_candidate(candidate, dest, timeout=10, retries=0, deadline=1e9)
        self.assertEqual(list(dest.glob("*.part")), [])

    def test_缺少_METADATA_的_zip_被拒绝(self):
        path = self.tmp / "fake.whl"
        with zipfile.ZipFile(path, "w") as archive:
            archive.writestr("mujoco/__init__.py", "")
        with self.assertRaises(cwt.TagViolation) as ctx:
            cwt._check_zip_is_wheel(path)
        self.assertIn("METADATA", str(ctx.exception))

    def test_端到端抓取写_wheelhouse_json(self):
        """mock 索引页 + file:// wheel：完整走通 解析→选择→下载→写清单→标签校验。"""
        source = make_wheel(self.tmp / "src" / GOOD_MUJOCO, GOOD_MUJOCO)
        html = index_html(f"{source.as_uri()}")
        dest = self.tmp / "dest"
        with mock.patch.object(cwt, "_http_get", return_value=html.encode("utf-8")):
            document = cwt.fetch(
                self.spec,
                dest,
                dry_run=False,
                resolve=True,
                timeout=10,
                deadline_seconds=60,
                retries=0,
            )
        manifest = json.loads((dest / "wheelhouse.json").read_text(encoding="utf-8"))
        self.assertEqual(manifest["schema_version"], "iraf.wheelhouse/v1")
        self.assertEqual(len(manifest["entries"]), 1)
        self.assertEqual(manifest["simulation_note"], cwt.SIMULATION_EVIDENCE_NOTE)
        self.assertEqual(document["entries"][0]["kind"], "exact")
        report = cwt.verify_dir(dest, self.spec)
        self.assertEqual(report["violations"], [])
        self.assertEqual(report["checked"], 1)

    def test_声明包缺可用_wheel_时硬失败且不写盘(self):
        html = index_html("../../packages/a/numpy-2.3.0-cp310-cp310-win_arm64.whl")
        dest = self.tmp / "dest"
        with mock.patch.object(cwt, "_http_get", return_value=html.encode("utf-8")):
            with self.assertRaises(cwt.DeclarationError) as ctx:
                cwt.fetch(self.spec, dest, dry_run=False, resolve=True, timeout=10, deadline_seconds=60, retries=0)
        self.assertIn("没有可用", str(ctx.exception))
        self.assertFalse(dest.exists(), "硬失败前不得创建目标目录（不留下半成品）")

    def test_dry_run_不写盘(self):
        html = index_html("../../packages/a/mujoco-3.9.0-cp310-cp310-manylinux_2_28_aarch64.whl")
        dest = self.tmp / "dest"
        with mock.patch.object(cwt, "_http_get", return_value=html.encode("utf-8")):
            document = cwt.fetch(
                self.spec, dest, dry_run=True, resolve=True, timeout=10, deadline_seconds=60, retries=0
            )
        self.assertTrue(document["dry_run"])
        self.assertFalse(dest.exists())


class TestVerifyDir(unittest.TestCase):
    """目录级校验：混入错标签必须被发现（否则 wheelhouse 会把错 wheel 带进 bundle）。"""

    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory(prefix="iraf-wheel-verify-")
        self.tmp = Path(self._tmp.name)
        self.spec = make_spec()

    def tearDown(self):
        self._tmp.cleanup()

    def test_全对目录通过(self):
        for name in (GOOD_MUJOCO, GOOD_MUJOCO_OLD, PURE_PY):
            make_wheel(self.tmp / name, name)
        report = cwt.verify_dir(self.tmp, self.spec)
        self.assertEqual(report["violations"], [])
        self.assertEqual(report["checked"], 3)
        self.assertEqual(report["accepted"], 3)

    def test_混入错标签目录被拒绝并列出文件名(self):
        make_wheel(self.tmp / GOOD_MUJOCO, GOOD_MUJOCO)
        make_wheel(self.tmp / BAD_WIN, BAD_WIN)
        report = cwt.verify_dir(self.tmp, self.spec)
        self.assertEqual([item["filename"] for item in report["violations"]], [BAD_WIN])
        self.assertIn("win_arm64", report["violations"][0]["reason"])

    def test_空目录被拒绝(self):
        with self.assertRaises(cwt.TagViolation):
            cwt.verify_dir(self.tmp, self.spec)

    def test_目录不存在被拒绝(self):
        with self.assertRaises(cwt.TagViolation):
            cwt.verify_dir(self.tmp / "nope", self.spec)


class TestCliAndShellEntry(unittest.TestCase):
    """CLI 与 shell 入口的退出码契约（全部离线）。"""

    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory(prefix="iraf-wheel-cli-")
        self.tmp = Path(self._tmp.name)

    def tearDown(self):
        self._tmp.cleanup()

    def run_py(self, *args):
        return subprocess.run(
            [sys.executable, str(REPO_ROOT / "deploy" / "sdk" / "check_wheel_tags.py"), *args],
            capture_output=True,
            text=True,
            cwd=REPO_ROOT,
        )

    def run_sh(self, *args):
        return subprocess.run(
            ["bash", str(SCRIPT), *args], capture_output=True, text=True, cwd=REPO_ROOT
        )

    def test_plan_退出码_0(self):
        result = self.run_py("--matrix", str(MATRIX), "plan")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(json.loads(result.stdout)["target"]["id"], "aarch64-manylinux_2_28-cp310")

    def test_classify_正负例退出码(self):
        good = self.run_py("--matrix", str(MATRIX), "classify", GOOD_MUJOCO)
        self.assertEqual(good.returncode, 0, good.stderr)
        bad = self.run_py("--matrix", str(MATRIX), "classify", GOOD_MUJOCO, BAD_WIN)
        self.assertEqual(bad.returncode, 4, bad.stdout)
        self.assertIn("win_arm64", bad.stderr)

    def test_verify_子命令退出码(self):
        for name in (GOOD_MUJOCO, GOOD_MUJOCO_OLD):
            make_wheel(self.tmp / name, name)
        ok = self.run_py("--matrix", str(MATRIX), "verify", "--dir", str(self.tmp))
        self.assertEqual(ok.returncode, 0, ok.stderr)
        make_wheel(self.tmp / BAD_MACOS, BAD_MACOS)
        bad = self.run_py("--matrix", str(MATRIX), "verify", "--dir", str(self.tmp))
        self.assertEqual(bad.returncode, 4)

    def test_shell_帮助退出码_0(self):
        result = self.run_sh("--help")
        self.assertEqual(result.returncode, 0)
        self.assertIn("退出码", result.stdout)

    def test_shell_未知参数退出码_1(self):
        result = self.run_sh("--不存在")
        self.assertEqual(result.returncode, 1)
        self.assertIn("无法识别的参数", result.stderr)

    def test_shell_no_resolve_必须配_dry_run(self):
        result = self.run_sh("--no-resolve")
        self.assertEqual(result.returncode, 1)
        self.assertIn("--dry-run", result.stderr)

    def test_shell_dry_run_不写盘且退出码_0(self):
        dest = self.tmp / "wheelhouse"
        result = self.run_sh("--dry-run", "--no-resolve", "--dest", str(dest))
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertFalse(dest.exists())
        self.assertIn("dry_run", result.stdout)

    def test_shell_空_index_url_退出码_2(self):
        """步骤明文要求：index_url 为空时 --dry-run 必须退出码 2 并提示声明源。"""
        import yaml

        broken = self.tmp / "matrix-empty-index.yaml"
        document = json.loads(json.dumps(MATRIX_DOC))
        document["targets"][0]["index_url"] = ""
        broken.write_text(yaml.safe_dump(document, allow_unicode=True), encoding="utf-8")
        result = self.run_sh("--dry-run", "--matrix", str(broken))
        self.assertEqual(result.returncode, 2, result.stdout)
        self.assertIn("index_url", result.stderr)

    def test_shell_verify_目录不存在退出码_2(self):
        result = self.run_sh("--verify", "--dest", str(self.tmp / "nope"))
        self.assertEqual(result.returncode, 2)

    def test_shell_verify_混入错标签退出码_4(self):
        for name in (GOOD_MUJOCO, BAD_WIN):
            make_wheel(self.tmp / name, name)
        result = self.run_sh("--verify", "--dest", str(self.tmp))
        self.assertEqual(result.returncode, 4)
        self.assertIn(BAD_WIN, result.stderr)

    def test_shell_verify_全对退出码_0(self):
        make_wheel(self.tmp / GOOD_MUJOCO, GOOD_MUJOCO)
        result = self.run_sh("--verify", "--dest", str(self.tmp))
        self.assertEqual(result.returncode, 0, result.stderr)


if __name__ == "__main__":
    unittest.main()
