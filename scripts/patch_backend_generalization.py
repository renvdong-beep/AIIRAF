"""把 mujoco_backend 里的 Piper 硬编码改为读配置声明（最小侵入补丁）。

三个必须改的点，全部是"关节/几何名写死"导致的泛化缺口：

1. `_set_gripper_controls` 写死 `{"joint7","joint8"}`，并用它们做过滤。
   UR5 的夹爪是**执行器名**（`rq2f85_fingers_actuator`），不是 joint7/joint8，
   过滤后集合为空 → 报"夹爪开合配置必须包含 joint7/joint8"。
   改为：不再按名字过滤，直接使用调用方传入的夹爪开合字典
   （它来自场景配置，本来就只含夹爪通道）。

2. `_grasp_alignment_evidence` 写死 `piper_left_finger` / `piper_right_finger`
   **geom 名**。找不到时回退到 body 位置。2F-85 的 pad body 原点在铰链处，
   与 pad box 中心相差约 2cm，导致对齐门禁把"正确的抓取"误报成 94mm 偏差。
   改为：优先读 `gripper.left_finger_geom` / `right_finger_geom`，
   缺失时再回退到旧的 `piper_*` 名（保持 Piper 行为逐位不变）。

3. 同函数的 `joint_qpos` 取证写死 `joint1..joint6`。
   改为用 profile 声明的手臂关节（从 profile.joint_roles 推 arm 角色），
   这样自检信息对任何构型都有意义。

补丁用"精确文本替换 + 命中计数断言"实现：命中数不符即失败，
避免静默改错（这是本工程反复强调的"失败即显式"）。
"""

import argparse
import sys
from pathlib import Path

PATCHES = [
    (
        "set_gripper_controls",
        '''    def _set_gripper_controls(self, positions):
        """只更新夹爪指关节，避免开合动作覆盖机械臂 1-6 号关节。"""
        finger_positions = {
            name: value
            for name, value in positions.items()
            if name in {"joint7", "joint8"}
        }
        if not finger_positions:
            raise ValueError("夹爪开合配置必须包含 joint7/joint8")
        self._set_controls(finger_positions)
''',
        '''    def _set_gripper_controls(self, positions):
        """只更新夹爪通道，避免开合动作覆盖机械臂关节。

        泛化说明（原实现写死 joint7/joint8）：
        夹爪驱动通道的**名字由构型决定**——Piper 是 joint7/joint8 两个位置
        关节，Robotiq 2F-85 是单个 tendon 执行器 `rq2f85_fingers_actuator`。
        调用方传入的 `positions` 来自场景配置的 `gripper.open/closed`，
        本来就**只含夹爪通道**，因此直接使用即可，不能再按写死的名字过滤
        （否则 UR5 会被过滤成空集并误报"必须包含 joint7/joint8"）。
        """
        finger_positions = {
            str(name): float(value) for name, value in positions.items()
        }
        if not finger_positions:
            raise ValueError("夹爪开合配置不能为空")
        self._set_controls(finger_positions)
''',
    ),
    (
        "alignment_geom_source",
        '''            left_geom = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, "piper_left_finger")
            right_geom = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, "piper_right_finger")
''',
        '''            # 接触面 geom 名必须由配置声明，不能写死 Piper 的名字：
            # 2F-85 的 pad body 原点在铰链处，与 pad box 中心相差约 2cm，
            # 回退到 body 位置会让对齐门禁把正确抓取误报成 94mm 偏差。
            # 缺省值仍指向 Piper 的指腹网格，保证 Piper 行为逐位不变。
            gripper_cfg = self._manipulation.get("gripper") or {}
            left_geom_name = gripper_cfg.get("left_finger_geom", "piper_left_finger")
            right_geom_name = gripper_cfg.get("right_finger_geom", "piper_right_finger")
            left_geom = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_GEOM, str(left_geom_name)
            )
            right_geom = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_GEOM, str(right_geom_name)
            )
''',
    ),
    (
        "alignment_joint_qpos",
        '''            "joint_qpos": {
                name: self._joint_qpos(name)
                for name in ("joint1", "joint2", "joint3", "joint4", "joint5", "joint6")
            },
''',
        '''            # 取证用的关节列表取自 profile 声明的 arm 角色关节，
            # 这样换构型后自检信息仍然可读（原实现写死 joint1..joint6）。
            "joint_qpos": {
                name: self._joint_qpos(name)
                for name in self._arm_joint_names()
            },
''',
    ),
    (
        "arm_joint_names_helper",
        '''    def _joint_qpos(self, name):
        actuator = self._actuators.get(name)
        if actuator is None:
            return None
        joint_id = int(self.model.actuator_trnid[actuator, 0])
        return float(self.data.qpos[self.model.jnt_qposadr[joint_id]])
''',
        '''    def _arm_joint_names(self):
        """返回 profile 声明为 arm 角色的关节名（按 profile.joints 顺序）。

        语义说明：这里返回的是**关节名**，而 `_joint_qpos` 按 actuator 名
        查 ctrl 通道；两者在 Piper 上同名，在 UR5 上不同名
        （joint: shoulder_pan_joint / actuator: shoulder_pan）。
        因此 `_joint_qpos` 需同时支持两种命名：先按 actuator 名查，
        查不到再按同名关节查 qpos。
        """
        roles = getattr(self.profile, "joint_roles", None) or {}
        names = [name for name in self.profile.joints if roles.get(name) == "arm"]
        return names or list(self.profile.joints)

    def _joint_qpos(self, name):
        """读取关节角：既接受执行器名，也接受关节名。

        - 执行器名：走 actuator_trnid 反查它驱动的关节；
        - 关节名：直接查 jnt_qposadr（UR5 的 actuator 名与关节名不同，
          只查 actuator 会全部返回 None，自检信息失去意义）。
        """
        actuator = self._actuators.get(name)
        if actuator is not None:
            joint_id = int(self.model.actuator_trnid[actuator, 0])
            return float(self.data.qpos[self.model.jnt_qposadr[joint_id]])
        joint_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_JOINT, str(name)
        )
        if joint_id < 0:
            return None
        return float(self.data.qpos[int(self.model.jnt_qposadr[joint_id])])
''',
    ),
    (
        "initial_keyframe_apply",
        '''        self.model = mujoco.MjModel.from_xml_path(self._model_path)
        self.data = mujoco.MjData(self.model)
        mujoco.mj_forward(self.model, self.data)
''',
        '''        self.model = mujoco.MjModel.from_xml_path(self._model_path)
        self.data = mujoco.MjData(self.model)
        # **按模型自带的关键帧初始化位形**（存在时）。
        # 为什么必须做：MjData 的默认 qpos 是**全零**，而全零位形对多数
        # 6 轴臂意味着"手臂竖直向上 / 夹爪水平伸出"。pick_object 的
        # HOME_HOLD 段是从**当前位形**插值到 HOME 的：
        # 从全零位插到"夹爪朝下"的 HOME，中途 pad 会降到台面以下
        # （实测 UR5e：t=0.33 处 pad_z=-0.0047m），沿途把方块顶飞
        # （实测方块被推到 z=5.3m，对齐门禁随即报 59m 偏差）。
        # MJCF 的 keyframe[0] 是模型作者声明的"合理初始位形"
        # （UR5e 官方 home / 我们的场景会把它写成抓取起始位形），
        # 用它初始化即可消除跨台面扫掠。
        # 无 keyframe 时保持全零，行为与改动前一致（Piper 不受影响）。
        # 用 getattr 探测而非直接取属性：单测会用 SimpleNamespace 替身，
        # 没有 nkey 字段（直接取会在装配期抛 AttributeError）。
        key_count = int(getattr(self.model, "nkey", 0) or 0)
        if key_count > 0:
            mujoco.mj_resetDataKeyframe(self.model, self.data, 0)
        mujoco.mj_forward(self.model, self.data)
''',
    ),
    (
        "parse_gripper_extra_fields",
        '''            anchor_body = raw_gripper.get("lift_anchor_body")
            if anchor_body is not None:
                if not isinstance(anchor_body, str) or not anchor_body:
                    raise ValueError("lift_anchor_body 必须是非空字符串")
                gripper["lift_anchor_body"] = anchor_body
        return {"targets": targets, "gripper": gripper}
''',
        '''            anchor_body = raw_gripper.get("lift_anchor_body")
            if anchor_body is not None:
                if not isinstance(anchor_body, str) or not anchor_body:
                    raise ValueError("lift_anchor_body 必须是非空字符串")
                gripper["lift_anchor_body"] = anchor_body
            # 构型相关的名字必须**原样保留**：本解析器只对已知字段做
            # 类型校验，但下面的字段是"由配置声明替代写死名字"的载体，
            # 不在这里透传就会在运行时被静默丢弃——
            # 实测表现为 UR5 场景下报 "body not found: link6"
            # （配置里明明写了 wrist_body=wrist_3_link）。
            for key in (
                "wrist_body",
                "left_finger_geom",
                "right_finger_geom",
            ):
                value = raw_gripper.get(key)
                if value is None:
                    continue
                if not isinstance(value, str) or not value:
                    raise ValueError(key + " 必须是非空字符串")
                gripper[key] = value
        return {"targets": targets, "gripper": gripper}
''',
    ),
    (
        "wrist_body_lookup",
        '''            wrist = self._body_id("link6")
''',
        '''            # 腕部 body 名由配置声明（缺省仍是 Piper 的 link6），
            # 避免换构型后诊断日志直接抛"缺少 body: link6"。
            wrist = self._body_id(
                (self._manipulation.get("gripper") or {}).get(
                    "wrist_body", "link6"
                )
            )
''',
    ),
    (
        "calibrate_grasp_bodies",
        '''        wrist, left, right = body("link6"), body("link7"), body("link8")
''',
        '''        gripper_cfg = (self._manipulation.get("gripper") or {})
        wrist = body(gripper_cfg.get("wrist_body", "link6"))
        left = body(gripper_cfg.get("left_finger_body", "link7"))
        right = body(gripper_cfg.get("right_finger_body", "link8"))
''',
    ),
]


def apply_patch(path, dry_run=False):
    text = Path(path).read_text(encoding="utf-8")
    report = []
    for name, old, new in PATCHES:
        count = text.count(old)
        if count == 0:
            if new in text:
                report.append((name, "already-applied"))
                continue
            report.append((name, "NOT-FOUND"))
            continue
        if count != 1:
            report.append((name, "AMBIGUOUS(%d)" % count))
            continue
        text = text.replace(old, new, 1)
        report.append((name, "patched"))
    failures = [item for item in report if item[1] not in ("patched", "already-applied")]
    if not dry_run and not failures:
        Path(path).write_text(text, encoding="utf-8")
    for name, status in report:
        print("  %-26s %s" % (name, status))
    return failures


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--path", default="src/iraf_adapters/mujoco/mujoco_backend.py"
    )
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()
    failures = apply_patch(args.path, dry_run=args.dry_run)
    if failures:
        print("FAILED:", failures)
        return 1
    print("OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
