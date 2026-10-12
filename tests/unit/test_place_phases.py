# -*- coding: utf-8 -*-
"""放置相位**单点声明**的回归测试（P4 / B1+B2，2026-10-12）。

锁住三件事（任一被破坏即单独抓到，不必跑整链）：
1. `PLACE_PHASES` 的顺序/可选性/必需性与既有行为一致（`via`/`transit` 可选，其余必需）；
2. 派生视图（names/keys/required/optional）与主表**同源**，不各自维护；
3. `place_phase_names` 按规范顺序映射，且**不静默丢弃**未登记的键。
"""
import unittest

from iraf_skills.common.place_phases import (
    OPTIONAL_PLACE_POSITIONS_KEYS,
    PLACE_PHASE_BY_NAME,
    PLACE_PHASE_NAMES,
    PLACE_PHASE_NAME_BY_KEY,
    PLACE_PHASES,
    PLACE_POSITIONS_KEYS,
    REQUIRED_PLACE_POSITIONS_KEYS,
    place_phase_names,
)


class PlacePhaseTableTests(unittest.TestCase):
    def test_canonical_order_is_stable(self):
        """规范顺序 = transit → via → above → descend → retreat（与后端回放/构建器求解一致）。"""
        self.assertEqual(PLACE_PHASE_NAMES,
                         ("transit", "via", "above", "descend", "retreat"))

    def test_optionality_matches_legacy_behaviour(self):
        """`via`/`transit` 可选（缺席即不插入），其余必需（缺席即显式失败）。"""
        self.assertEqual(set(OPTIONAL_PLACE_POSITIONS_KEYS),
                         {"place_transit_positions", "place_via_positions"})
        self.assertEqual(set(REQUIRED_PLACE_POSITIONS_KEYS),
                         {"place_above_positions", "place_descend_positions",
                          "place_retreat_positions"})

    def test_derived_views_are_same_source(self):
        """派生视图与主表同源：names/keys 由 PLACE_PHASES 导出，不各自维护。"""
        self.assertEqual(PLACE_PHASE_NAMES, tuple(p.name for p in PLACE_PHASES))
        self.assertEqual(PLACE_POSITIONS_KEYS, tuple(p.positions_key for p in PLACE_PHASES))
        self.assertEqual(set(PLACE_PHASE_BY_NAME), set(PLACE_PHASE_NAMES))
        self.assertEqual(set(PLACE_PHASE_NAME_BY_KEY), set(PLACE_POSITIONS_KEYS))
        # 必需 + 可选 = 全体，无重叠、无遗漏
        self.assertEqual(set(REQUIRED_PLACE_POSITIONS_KEYS) | set(OPTIONAL_PLACE_POSITIONS_KEYS),
                         set(PLACE_POSITIONS_KEYS))
        self.assertFalse(set(REQUIRED_PLACE_POSITIONS_KEYS) & set(OPTIONAL_PLACE_POSITIONS_KEYS))

    def test_positions_key_prefix_is_conventional(self):
        """positions 键一律以 `place_` 开头、`_positions` 结尾（探针按此识别相位键）。"""
        for key in PLACE_POSITIONS_KEYS:
            self.assertTrue(key.startswith("place_") and key.endswith("_positions"), key)

    def test_phase_names_follow_canonical_order_and_keep_unknown(self):
        """子集映射按规范顺序；未登记键**保留原名**（不静默丢弃）。"""
        names = place_phase_names(["place_descend_positions", "place_above_positions"])
        self.assertEqual(names, ("above", "descend"))
        names = place_phase_names(["place_above_positions", "place_mystery_positions"])
        self.assertEqual(names, ("above", "place_mystery_positions"))


if __name__ == "__main__":
    unittest.main()
