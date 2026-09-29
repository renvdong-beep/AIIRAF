"""`accept_payload`（载荷确认）的单测：成功路径 + 拒绝路径（AGENTS.md 2.8）。

场景级验收已在 `build/acceptance/handoff_lab/nominal/report.json`（s01–s05 全绿）覆盖；本文件补单元层：

  1. **几何事实**（共享测量 `iraf_adapters.mujoco.payload_facts.confirm_payload_on_target`）：
     载荷落在接收体上 / 悬空 / 落在旁边（无接触）三种情形，判据必须分别是 True/False/False。
  2. **Provider 成功路径**：输出必须通过 `skills/accept_payload/accept_payload.output.json` 校验
     （`additionalProperties: false` ⇒ 少一个诊断键、多一个未知键都会红）。
  3. **拒绝路径**（每条都要显式失败，不得返回伪造确认）：
     后端未实现该方法 / `payload_on_target=False` / 入参缺失。
"""

import json
import math
import unittest
from pathlib import Path

import jsonschema
import mujoco

from iraf_adapters.mujoco.payload_facts import confirm_payload_on_target
from iraf_skills.common.motion import SkillRejected
from iraf_skills.quadruped import AcceptPayloadProvider, SkillContractError

ROOT = Path(__file__).resolve().parents[2]
OUTPUT_CONTRACT = json.loads(
    (ROOT / "skills" / "accept_payload" / "accept_payload.output.json").read_text(encoding="utf-8"))

#: 最小几何：地面 + 接收体（盒，顶面 z=0.065）+ 载荷（0.05 立方体，自由体）
SCENE = """<mujoco>
  <option gravity="0 0 -9.81"/>
  <worldbody>
    <geom name="floor" type="plane" size="2 2 0.1"/>
    <body name="tray_01" pos="0 0 0.06">
      <geom name="tray_01_geom" type="box" size="0.06 0.04 0.005" friction="1.4 0.03 0.001"/>
    </body>
    <body name="box_01" pos="{bx} {by} {bz}" quat="{bq}">
      <freejoint/>
      <geom name="box_01_geom" type="box" size="0.025 0.025 0.025" friction="2.0 0.05 0.001"/>
    </body>
  </worldbody>
</mujoco>"""

TRAY_TOP_Z = 0.065          # 0.06 + 0.005（几何事实，写清来源而不是散落魔数）
BOX_HALF = 0.025


def _facts(bx=0.0, by=0.0, bz=None, bq="1 0 0 0", margin=None):
    """在最小场景里量一次事实；默认把载荷放在承载面上（压入 0.1 mm 以建立接触）。

    `margin` 非空时给**载荷与接收体的 geom**都加上声明化的接触 margin（模拟厂商 Go2 模型
    在 `<default class="go2">` 里声明的 `margin="0.001"`）。
    """
    if bz is None:
        bz = TRAY_TOP_Z + BOX_HALF - 0.0001
    xml = SCENE.format(bx=bx, by=by, bz=bz, bq=bq)
    if margin is not None:
        for friction in ('friction="1.4 0.03 0.001"', 'friction="2.0 0.05 0.001"'):
            xml = xml.replace(friction, '%s margin="%s"' % (friction, margin))
    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)
    return confirm_payload_on_target(mujoco, model, data, "box_01", "tray_01")


class PayloadFactsTests(unittest.TestCase):
    """共享测量的几何判据（臂侧 place_object 与四足侧 accept_payload 同源）。"""

    def test_payload_resting_on_target_is_confirmed(self):
        facts = _facts()
        self.assertTrue(facts["payload_on_target"], msg=facts)
        self.assertTrue(facts["contact_geoms"], msg=facts)
        # 落位间隙：载荷最低点 = 承载面 −0.1 mm（接触；不是悬空）
        self.assertAlmostEqual(facts["resting_gap_m"], -0.0001, places=4)
        self.assertAlmostEqual(facts["target_top_z_m"], TRAY_TOP_Z, places=6)
        self.assertLess(facts["offset_from_target_center_m"], 0.06)

    def test_tilted_payload_resting_on_target_is_confirmed(self):
        """**倾斜**载荷压在承载面上：最低点必须按世界顶点算（不得用 center−half_z）。

        2026-09-29 实测缺陷：`center[2] - half_z` 只对轴对齐盒成立。托盘挂到狗背上后，
        载荷会随狗身姿态在托盘上轻微倾斜（实测 0.72 mm / 0.05 m ≈ 0.83°）⇒ 该式**高估**最低点，
        明明已接触（`contact_geoms=['box_01_geom']`）却报 `resting_gap=+0.000722398 m`，
        s05 因此误报"载荷未确认落在接收体上"。承载面侧一直用世界顶点，两侧口径必须一致。
        """
        angle = math.radians(10.0)
        quat = "%.12f %.12f 0 0" % (math.cos(angle / 2.0), math.sin(angle / 2.0))
        # 绕 x 倾斜 θ 后，盒的竖直半高变成 half·(cosθ + sinθ)；据此把最低点压在承载面下 0.1 mm
        lowest_half = BOX_HALF * (math.cos(angle) + math.sin(angle))
        facts = _facts(bz=TRAY_TOP_Z + lowest_half - 0.0001, bq=quat)
        self.assertTrue(facts["contact_geoms"], msg=facts)
        self.assertAlmostEqual(facts["resting_gap_m"], -0.0001, places=4)
        self.assertTrue(facts["payload_on_target"], msg=facts)

    def test_payload_within_contact_margin_is_confirmed(self):
        """**接触 margin（"软垫"）之内**的贴合必须算"落在承载面上"。

        2026-09-29 实测（§11.23(47)）：厂商 Go2 模型在 `<default class="go2">` 里声明
        `margin="0.001"`，MuJoCo 在 `dist < margin` 时即把接触纳入约束集 ⇒ 载荷**稳定停在**
        几何表面上方 0.489 mm（4 个接触点、每点 0.1007~0.1174 N、合计 ≈0.436 N ≈ 载荷重量）。
        写死 `resting_gap <= 0` 会让**任何带 margin 的模型永远判"悬空"**（旧的世界固定托盘
        恰好不受影响，掩盖了这条）。判据口径必须取自模型自身的 margin。
        """
        # 载荷底距承载面 0.5 mm：几何上"悬空"，但 < margin 0.001 ⇒ 接触存在且应判"落在承载面上"
        facts = _facts(bz=TRAY_TOP_Z + BOX_HALF + 0.0005, margin=0.001)
        self.assertTrue(facts["contact_geoms"], msg=facts)
        self.assertAlmostEqual(facts["contact_margin_m"], 0.001, places=9)
        self.assertGreater(facts["resting_gap_m"], 0.0)
        self.assertTrue(facts["payload_on_target"], msg=facts)

    def test_floating_payload_is_rejected(self):
        """悬空 4 cm：即使水平位置正确也不得确认。"""
        facts = _facts(bz=TRAY_TOP_Z + BOX_HALF + 0.04)
        self.assertFalse(facts["payload_on_target"], msg=facts)
        self.assertEqual(facts["contact_geoms"], [])
        self.assertGreater(facts["resting_gap_m"], 0.0)

    def test_payload_beside_target_is_rejected(self):
        """落在旁边（无接触）：即使高度贴着承载面也不得确认。"""
        facts = _facts(bx=0.2)
        self.assertFalse(facts["payload_on_target"], msg=facts)
        self.assertEqual(facts["contact_geoms"], [])

    def test_chain_speed_is_reported(self):
        """整链末速必须是有限非负数（模型里所有自由关节的线速度上界）。"""
        facts = _facts()
        self.assertGreaterEqual(facts["last_speed_mps"], 0.0)
        self.assertTrue(facts["last_speed_mps"] < 1e-6, msg=facts)


class _FakeBackend:
    """最小替身：只提供 Provider 需要的那一个方法，并记录调用参数。"""

    def __init__(self, report=None):
        self.report = report
        self.calls = []

    def accept_payload(self, payload_id, place_target_id, lease):
        self.calls.append((payload_id, place_target_id, lease))
        return self.report


class _BackendWithoutCapability:
    """**没有** `accept_payload` 方法的后端替身：Provider 必须显式拒绝（不伪造确认）。"""


def _report_from(facts, payload_id="box_01", place_target_id="tray_01"):
    return {"payload_id": payload_id, "place_target_id": place_target_id,
            "confirmation": "payload_confirmed", **facts,
            "runtime_source": "live_fk", "phase_trace": [{"note": "单测"}]}


class AcceptPayloadProviderTests(unittest.TestCase):
    """Provider 层：成功输出要过契约；三条拒绝路径都要显式失败。"""

    def setUp(self):
        self.lease = {"lease_id": "test", "expires_at": 0}

    def test_success_output_matches_contract(self):
        backend = _FakeBackend(_report_from(_facts()))
        result = AcceptPayloadProvider(None, backend).execute(
            {"payload_id": "box_01", "place_target_id": "tray_01"}, self.lease)
        jsonschema.validate(result, OUTPUT_CONTRACT)     # additionalProperties: false ⇒ 严格
        self.assertEqual(result["skill"], "accept_payload")
        self.assertTrue(result["accepted"])
        self.assertEqual(result["confirmation"], "payload_confirmed")
        self.assertTrue(result["evidence"]["payload_on_target"])
        self.assertEqual(backend.calls, [("box_01", "tray_01", self.lease)])

    def test_backend_without_capability_is_rejected(self):
        backend = _BackendWithoutCapability()
        with self.assertRaises(SkillRejected):
            AcceptPayloadProvider(None, backend).execute(
                {"payload_id": "box_01", "place_target_id": "tray_01"}, self.lease)

    def test_payload_not_on_target_is_rejected(self):
        facts = _facts(bz=TRAY_TOP_Z + BOX_HALF + 0.04)
        backend = _FakeBackend(_report_from(facts))
        with self.assertRaises(SkillRejected) as ctx:
            AcceptPayloadProvider(None, backend).execute(
                {"payload_id": "box_01", "place_target_id": "tray_01"}, self.lease)
        message = str(ctx.exception)
        self.assertIn("载荷未确认落在接收体上", message)
        # 报错必须带**可用数字**（不是只给字段名）：悬空 4 cm ⇒ 落位间隙 +0.04 m、无接触
        self.assertIn("0.04", message)
        self.assertIn("接触 geom []", message)

    def test_missing_inputs_are_contract_errors(self):
        backend = _FakeBackend(_report_from(_facts()))
        provider = AcceptPayloadProvider(None, backend)
        for bad in ({"payload_id": "box_01"}, {"place_target_id": "tray_01"}, {}):
            with self.assertRaises(SkillContractError):
                provider.execute(bad, self.lease)
        self.assertEqual(backend.calls, [], "入参不合法时不得调用后端")


if __name__ == "__main__":
    unittest.main()
