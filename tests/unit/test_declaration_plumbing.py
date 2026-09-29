"""声明链"贯通"回归测试（AGENTS.md 2.8：契约与声明变更必须有回归测试）。

背景（2026-09-29 §11.23(48)）：同类缺陷在本会话出现 **7 次** —— 新声明的键在某一层被**静默丢弃**，
运行期才以"参数不生效 / 夹到空气 / 缺字段"暴露。声明从"基线 YAML"到"后端能读到"要穿过 **4 个显式枚举点**：

  ① `scripts/build_piper_baseline.py`     参考姿态求解器：读基线并**枚举**哪些键进 reference
  ② `scripts/build_piper_pick_scene.py`   臂侧场景 builder：枚举哪些键进报告 `gripper`
  ③ `src/iraf_adapters/unitree/scene_builder.py` 联合继承：`SEMANTIC_GRIPPER_KEYS`（顶层语义键原样继承）
  ④ `src/iraf_adapters/mujoco/mujoco_backend.py` 后端配置解析层：枚举哪些键进后端 config

**每层处理哪些键是不一样的**（例如 ① 只管 reference 需要的键、③ 只管顶层语义键）——
所以本文件用**逐层键表**把当前已验证的贯通关系钉住：任何一层漏掉某个键即红。
它不依赖构建产物（磁盘上的报告可能是上一轮的），只读源码；代价是"键名出现在文件里"这种检查偏弱，
但正是它能抓住本轮真实发生的那类"漏一行枚举"。
"""

import unittest
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[2]
BASELINE = ROOT / "config" / "piper_simulation_baseline.yaml"

#: ① 参考姿态求解器必须枚举的键（它只负责 reference 里要用的那些）
LAYER_REFERENCE_KEYS = ("enabled", "pre_lift_m", "depth_m", "open_m", "hold_pre_lift")

#: ② 臂侧场景 builder 必须写进报告 `gripper` 的键
LAYER_BUILDER_KEYS = ("lift_path", "lift_gripper", "carry_constraint", "regrasp",
                      "place_settle_ms", "place_pose_correction",
                      "lift_constraint", "require_friction_lift", "lift_anchor_body")

#: ③ 联合构建器 `SEMANTIC_GRIPPER_KEYS` 必须包含的顶层语义键
LAYER_UNION_KEYS = ("lift_path", "lift_gripper", "carry_constraint", "regrasp",
                    "place_settle_ms", "place_pose_correction",
                    "lift_constraint", "require_friction_lift", "lift_anchor_body")

#: ④ 后端解析层必须透传的键
LAYER_BACKEND_KEYS = ("lift_path", "lift_gripper", "carry_constraint", "place_settle_ms",
                      "place_pose_correction", "lift_constraint", "require_friction_lift",
                      "hold_pre_lift")

LAYERS = (
    ("scripts/build_piper_baseline.py", LAYER_REFERENCE_KEYS),
    ("scripts/build_piper_pick_scene.py", LAYER_BUILDER_KEYS),
    ("src/iraf_adapters/unitree/scene_builder.py", LAYER_UNION_KEYS),
    ("src/iraf_adapters/mujoco/mujoco_backend.py", LAYER_BACKEND_KEYS),
)


def _text(relative):
    return (ROOT / relative).read_text(encoding="utf-8")


class DeclarationPlumbingTests(unittest.TestCase):
    def test_baseline_declares_the_keys_we_check(self):
        """被检查的键必须在基线里确实声明（防止测试自己漂移成"检查不存在的键"）。"""
        document = yaml.safe_load(_text("config/piper_simulation_baseline.yaml"))
        grasp = document["grasp"]
        for key in ("lift_path", "lift_gripper", "carry_gripper", "carry_constraint", "regrasp",
                    "place_settle_ms", "place_pose_correction"):
            self.assertIn(key, grasp, "基线 grasp 段未声明 %s（测试清单需同步）" % key)
        for key in LAYER_REFERENCE_KEYS:
            self.assertIn(key, grasp["regrasp"], "基线 grasp.regrasp 未声明 %s" % key)
        # `require_friction_lift` 在 acceptance 段；`lift_constraint` 由 target_id 派生（不写进基线）
        self.assertIn("require_friction_lift", document["acceptance"])

    def test_every_layer_enumerates_its_keys(self):
        """逐层断言：某层少枚举一个键 ⇒ 该键到不了后端（运行时才暴露，本会话已踩 7 次）。"""
        for relative, keys in LAYERS:
            text = _text(relative)
            for key in keys:
                self.assertIn(key, text,
                              "%s 没有枚举 %s ⇒ 该键会在这一层被静默丢弃（§11.23(48)）"
                              % (relative, key))

    def test_union_semantic_keys_are_declared_verbatim(self):
        """③ 的语义键必须**原样继承**（不做前缀改写）：落到通用分支会按名字查模型而 fail-closed。"""
        text = _text("src/iraf_adapters/unitree/scene_builder.py")
        start = text.index("SEMANTIC_GRIPPER_KEYS = (")
        end = text.index(")", start)
        block = text[start:end]
        for key in LAYER_UNION_KEYS:
            self.assertIn('"%s"' % key, block,
                          "SEMANTIC_GRIPPER_KEYS 里缺 %s（原样继承的语义键）" % key)


if __name__ == "__main__":
    unittest.main()
