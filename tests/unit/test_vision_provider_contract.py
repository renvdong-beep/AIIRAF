"""视觉 Provider 的声明式契约：解析、占位符渲染与显式失败路径。

背景：后端曾把证据文件路径与检测器脚本路径写死
（build/calibration/piper-vision-target.json、scripts/detect_piper_target*.py），
违反"只依赖能力，不依赖具体模型/设备路径"。现在一切来自 `vision` 声明，
且任何字段都可缺省；缺省时行为必须明确（不刷新 / 要求请求显式给出证据）。
"""

import unittest

from iraf_adapters.mujoco.mujoco_backend import (
    DEFAULT_DETECTION_CONFIG_OUTPUT,
    VISION_REFRESH_MODES,
    _parse_vision_config,
    _render_detector_command,
    _resolve_project_path,
)


class VisionConfigParseTests(unittest.TestCase):
    def test_absent_section_means_no_vision_declaration(self):
        self.assertIsNone(_parse_vision_config(None))

    def test_parses_minimal_declaration(self):
        parsed = _parse_vision_config({"evidence_file": "build/calibration/x.json"})
        self.assertEqual("build/calibration/x.json", parsed["evidence_file"])
        self.assertEqual("always", parsed["refresh"])
        self.assertIsNone(parsed["detector"])

    def test_parses_full_declaration(self):
        parsed = _parse_vision_config(
            {
                "evidence_file": "build/calibration/x.json",
                "refresh": "on_missing",
                "detector": {
                    "command": ["{python}", "scripts/detect.py", "--output", "{evidence}"],
                    "config_file": "config/piper_multi_target.yaml",
                },
            }
        )
        self.assertEqual("on_missing", parsed["refresh"])
        self.assertEqual(
            ["{python}", "scripts/detect.py", "--output", "{evidence}"],
            parsed["detector"]["command"],
        )
        self.assertEqual(
            "config/piper_multi_target.yaml", parsed["detector"]["config_file"]
        )
        self.assertEqual(
            DEFAULT_DETECTION_CONFIG_OUTPUT, parsed["detector"]["config_output"]
        )

    def test_rejects_unknown_fields(self):
        with self.assertRaisesRegex(ValueError, "vision 含未知字段"):
            _parse_vision_config({"evidence_file": "a.json", "provider": "piper"})
        with self.assertRaisesRegex(ValueError, "vision.detector 含未知字段"):
            _parse_vision_config({"detector": {"command": ["x"], "script": "y"}})

    def test_rejects_bad_types_and_values(self):
        cases = (
            ("非对象", "vision 配置必须是对象"),
            ({"refresh": "sometimes"}, "vision.refresh"),
            ({"evidence_file": ""}, "vision.evidence_file"),
            ({"detector": []}, "vision.detector 必须是对象"),
            ({"detector": {"command": []}}, "必须是非空命令数组"),
            ({"detector": {"command": [""]}}, "不能为空字符串"),
            ({"detector": {"command": ["x"], "config_file": ""}}, "config_file"),
            ({"detector": {"command": ["x"], "config_output": ""}}, "config_output"),
        )
        for value, pattern in cases:
            with self.subTest(value=value):
                with self.assertRaisesRegex(ValueError, pattern):
                    _parse_vision_config(value)

    def test_rejects_unknown_placeholder_in_command(self):
        """占位符写错必须装配期失败：否则检测器会带着默认参数跑出错误证据。"""
        with self.assertRaisesRegex(ValueError, "未知占位符"):
            _parse_vision_config(
                {"detector": {"command": ["detect.py", "--model", "{modle}"]}}
            )

    def test_accepts_all_documented_placeholders(self):
        parsed = _parse_vision_config(
            {
                "detector": {
                    "command": [
                        "{python}",
                        "d.py",
                        "{model}",
                        "{evidence}",
                        "{config}",
                        "{target_id}",
                    ]
                }
            }
        )
        self.assertEqual(6, len(parsed["detector"]["command"]))

    def test_refresh_modes_are_closed_set(self):
        self.assertEqual(("always", "on_missing", "never"), VISION_REFRESH_MODES)


class DetectorCommandRenderTests(unittest.TestCase):
    def test_substitutes_every_placeholder(self):
        rendered = _render_detector_command(
            ["{python}", "d.py", "--model", "{model}", "--out", "{evidence}"],
            {
                "python": "/usr/bin/python3",
                "model": "/tmp/scene.xml",
                "evidence": "build/calibration/x.json",
            },
        )
        self.assertEqual(
            [
                "/usr/bin/python3",
                "d.py",
                "--model",
                "/tmp/scene.xml",
                "--out",
                "build/calibration/x.json",
            ],
            rendered,
        )

    def test_unknown_placeholder_fails_explicitly(self):
        with self.assertRaisesRegex(ValueError, "未知占位符"):
            _render_detector_command(["d.py", "{scene}"], {"model": "m"})

    def test_plain_arguments_pass_through(self):
        self.assertEqual(
            ["d.py", "--flag"], _render_detector_command(["d.py", "--flag"], {})
        )


class ProjectPathResolutionTests(unittest.TestCase):
    def test_relative_path_resolves_against_project_root(self):
        resolved = _resolve_project_path("config/piper_multi_target.yaml")
        self.assertTrue(resolved.is_absolute())
        self.assertEqual("config/piper_multi_target.yaml", str(resolved).split("AIIRAF/")[-1])

    def test_absolute_path_kept_as_is(self):
        self.assertEqual("/tmp/x.json", str(_resolve_project_path("/tmp/x.json")))


if __name__ == "__main__":
    unittest.main()
