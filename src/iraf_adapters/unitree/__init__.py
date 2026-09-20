"""宇树（Unitree）本体适配层：场景构建、四足适配器与 loopback 验收归这里。

本包只放**声明驱动**的代码：机型差异来自 `profiles/<robot>_mujoco.yaml`，
场景差异来自 `scenes/<scene>/scene.yaml`，控制数字来自 `config/<robot>_loopback.yaml`。

命名约定（步骤 16）：
- `quadruped.py` 是**通用契约层**：能力词表、标准反馈键、执行台账与安全闭锁、
  租约/fencing token 守卫。这一层不得出现任何厂家标识（关节名、DDS domain、CRC 等），
  由 `quadruped.assert_portable_surface()` 作为门禁而不是口号。
- `<vendor>_<model>.py`（如 `unitree_go2.py`）是**机型实现层**：厂家关节名 ↔ 执行器名
  的解析只在那一处（`resolve_joint_bindings`），且只允许从模型读力矩上限。
- `scene_builder.py` / `scene_sensor_evidence.py` / `loopback.py` 是场景与验收路径；
  它们的结论一律带 `simulation: true`，真机/目标端验收不在本包职责内（板卡不在场 → DEFERRED）。
"""
