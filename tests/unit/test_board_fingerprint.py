# -*- coding: utf-8 -*-
"""板卡指纹提取与交叉校验的测试（AGENTS 6.10：声明是权威，指纹只做防呆）。

被测：deploy/sdk/lib_fingerprint.py。
夹具全是**自造**的采集目录与板卡声明（不依赖真机、不依赖 build/ 下的历史证据），因此可重复。
覆盖：唯一匹配 / 字段不符 / {contains: …} 子集语义 / 无声明可判定 / 多声明区分度不足 / 采集缺项。
"""
from __future__ import annotations

import importlib.util
import json
import tempfile
import unittest
from pathlib import Path

import yaml

REPO = Path(__file__).resolve().parents[2]

_spec = importlib.util.spec_from_file_location("lib_fingerprint", REPO / "deploy" / "sdk" / "lib_fingerprint.py")
lib = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(lib)  # type: ignore[union-attr]

RAW_FIXTURE = {
    "domains.txt": (
        "[    8.548552] virtio_blk_probe: vm1 vdev=virtio-blk0 probe\n"
        "[    8.565211] virtio_vdisk_attach_block_device: vdisk=vm-0-virtio-blk0 "
        "blkfile=/download/image/vdisk/rootfs_vm1.img\n"
        "[    8.602022] virtio_net_probe: vm1 vdev=virtio-net0 net_name:vm-0-net0 probe success\n"
        "[    8.665885] virtio_blk_probe: vm2 vdev=virtio-blk0 probe\n"
        "[    8.675093] virtio_vdisk_attach_block_device: vdisk=vm-1-virtio-blk0 "
        "blkfile=/download/image/vdisk/rootfs_vm2.img\n"
        "[    8.761934] virtio_net_probe: vm2 vdev=virtio-net0 net_name:vm-1-net0 probe success\n"
    ),
    "identity.txt": (
        "uid=0(root) gid=0(root)\n---\n"
        "Linux localhost.localdomain 6.6.10 #1 SMP PREEMPT … aarch64\n---\nnone\n---\n"
        "rw fw_devlink=off root=/dev/nvme0n1p2 rootwait smmu=off iommu.passthrough=1 maxcpus=4\n"
    ),
    "iaros_version.txt": (
        "套件版本: Intewell AROS-R1.2.1\n"
        "Hyper: Intewell AROS-Hypervisor-V3.1.2-R1.2.1 (mainline_pkg: V3.1.2)\n"
        "RTOS: Intewell AROS-RTOS-V3.1.1-R1.2.1 (mainline_pkg: V3.1.1)\n"
    ),
    "npu_media.txt": "bin\nhoumo-drv-xh2_v1.4.0\nm1000\nmusa-5.1.0\nmtc-toolkit\nmt_vaapi\n---\ndri\n",
    "host.txt": "ID=Intewell\n",
}


class FingerprintTest(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.root = Path(self._tmp.name)
        self.collect = self.root / "collect"
        (self.collect / "raw").mkdir(parents=True)
        for name, text in RAW_FIXTURE.items():
            (self.collect / "raw" / name).write_text(text, encoding="utf-8")
        (self.root / "profiles" / "boards").mkdir(parents=True)

    def tearDown(self):
        self._tmp.cleanup()

    def _declare(self, name: str, fingerprint: dict, status: str = "unverified"):
        data = {
            "apiVersion": "iraf.intewell.io/v1", "kind": "BoardProfile",
            "metadata": {"name": name, "version": "0.1.0", "hyper_models": ["unverified"]},
            "spec": {"status": status, "fingerprint": fingerprint},
        }
        (self.root / "profiles" / "boards" / ("%s.yaml" % name)).write_text(
            yaml.safe_dump(data, allow_unicode=True, sort_keys=False), encoding="utf-8")

    # ---------- 提取 ----------
    def test_extract_from_raw_evidence(self):
        fp = lib.extract_fingerprint(self.collect)
        self.assertEqual(fp["hyper_guests"], ["vm1", "vm2"])
        self.assertEqual(fp["cpu_cores"], 4)
        self.assertIn("/download/image/vdisk/rootfs_vm1.img", fp["guest_vdisks"])
        self.assertIn("vm-0-net0", fp["guest_net_names"])
        self.assertIn("houmo-drv-xh2_v1.4.0", fp["npu_stack_tokens"])
        self.assertEqual(fp["iaros_components"]["套件版本"], "Intewell AROS-R1.2.1")

    # ---------- 比对 ----------
    def test_exact_match_single_board(self):
        self._declare("board_a", {"cpu_cores": 4, "hyper_guests": ["vm1", "vm2"]})
        report = lib.identify(self.root, self.collect)
        self.assertEqual(report["exit_code"], lib.EXIT_OK)
        self.assertEqual(report["matches"], ["board_a"])

    def test_field_mismatch_is_fail_closed(self):
        self._declare("board_a", {"cpu_cores": 8})
        report = lib.identify(self.root, self.collect)
        self.assertEqual(report["exit_code"], lib.EXIT_MISMATCH)
        self.assertIn("cpu_cores", report["verdict"])

    def test_contains_semantics_allows_extra_measured_tokens(self):
        # 声明"这些标记必须出现"，实测多出 mtc-toolkit 不应判不符
        self._declare("board_a", {"npu_stack_tokens": {"contains": ["houmo-drv-xh2_v1.4.0", "m1000"]}})
        report = lib.identify(self.root, self.collect)
        self.assertEqual(report["exit_code"], lib.EXIT_OK)

    def test_list_field_is_exact_not_subset(self):
        # 拓扑类字段用全等：少一个来宾就是不匹配
        self._declare("board_a", {"hyper_guests": ["vm1"]})
        report = lib.identify(self.root, self.collect)
        self.assertEqual(report["exit_code"], lib.EXIT_MISMATCH)

    def test_unverified_declared_field_is_not_decisive(self):
        self._declare("board_a", {"cpu_cores": "unverified", "hyper_guests": "unverified"})
        report = lib.identify(self.root, self.collect)
        self.assertEqual(report["exit_code"], lib.EXIT_UNDECIDABLE)

    def test_no_fingerprint_declared_is_undecidable(self):
        self._declare("board_a", {})
        data = yaml.safe_load((self.root / "profiles" / "boards" / "board_a.yaml").read_text(encoding="utf-8"))
        data["spec"].pop("fingerprint")
        (self.root / "profiles" / "boards" / "board_a.yaml").write_text(
            yaml.safe_dump(data, allow_unicode=True, sort_keys=False), encoding="utf-8")
        report = lib.identify(self.root, self.collect)
        self.assertEqual(report["exit_code"], lib.EXIT_UNDECIDABLE)
        self.assertIn("没有任何 BoardProfile 声明", report["verdict"])

    def test_two_matching_boards_is_undecidable(self):
        self._declare("board_a", {"cpu_cores": 4})
        self._declare("board_b", {"cpu_cores": 4})
        report = lib.identify(self.root, self.collect)
        self.assertEqual(report["exit_code"], lib.EXIT_UNDECIDABLE)
        self.assertIn("区分度不足", report["verdict"])

    def test_missing_raw_dir_is_usage_error(self):
        code = lib.main(["--collect-dir", str(self.root / "nope"), "--repo-root", str(self.root), "--json"])
        self.assertEqual(code, lib.EXIT_USAGE)

    def test_json_output_is_serialisable(self):
        self._declare("board_a", {"cpu_cores": 4})
        report = lib.identify(self.root, self.collect)
        text = json.dumps(report, ensure_ascii=False, default=str)
        self.assertIn("board_a", text)


if __name__ == "__main__":
    unittest.main()
