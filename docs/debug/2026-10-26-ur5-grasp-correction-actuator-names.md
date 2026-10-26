# UR5 抓取回归：纠偏只按**关节名**解析 ctrl 通道键 ⇒「臂关节键一个都解不出来」（2026-10-26）

## 1. 现象

本机跑官方 UR5 抓取验收（同一 baseline、同一场景路径，2026-09-20 是 SUCCEEDED）：

```
PYTHONPATH=src build/mcp-venv/bin/python scripts/verify_pick.py \
    --baseline config/ur5_simulation_baseline.yaml
execution_id = cc274700-ce86-5091-8145-4b06e3775528
status = FAILED   error_code = IRAF-EXECUTION-FAILED
reason = grasp_positions 里没有可解的臂关节键（纠偏无从下手）
profile = ur5_mujoco 0.1.0 (digest 3da10bd1…)
```

## 2. 根因（不是残差、不是模型能力）

抛点：`src/iraf_adapters/mujoco/mujoco_backend.py:3222-3229`（`_correct_grasp_column`）。
它拿 `positions_by_phase["grasp"]` 的键去 `mj_name2id(mjOBJ_JOINT, key)` 找关节，全部 <0 就拒绝纠偏。

而场景报告 `build/models/ur5-pick-scene.json` 的 `gripper.grasp_positions` 是**ctrl 通道（执行器）口径**：

```
shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3, rq2f85_fingers_actuator
```

模型里的**关节**名是 `shoulder_pan_joint, shoulder_lift_joint, … wrist_3_joint`；
`rq2f85_fingers_actuator` 是执行器名（不是关节）。

⇒ 6 个臂键 + 1 个夹爪键全部查不到关节 ⇒ `arm_joints=[]` ⇒ 报错。

为什么以前能跑、piper 也从没暴露：`scripts/build_robot_pick_scene.py:605-613` 用 `to_actuator_names()` 把
参考姿态写成 ctrl 通道名；**Piper 的关节名与执行器名同名**（joint1…joint8），所以按"关节名"解析碰巧成立；
UR5e 两者不同名（`shoulder_pan` vs `shoulder_pan_joint`），2026-10-12 重新生成场景报告后就必失败。

## 3. 修法（最小、无机型特例）

`_correct_grasp_column` 解析每个键时：先按关节名查，查不到就按**执行器名**用 `actuator_trnid` 反查它驱动的关节
（与同文件 `_joint_qpos` 已有口径一致，见 `mujoco_backend.py:3551-3567`）。不新增任何机型分支。

## 4. 复跑证据

```
PYTHONPATH=src build/mcp-venv/bin/python scripts/verify_pick.py \
    --baseline config/ur5_simulation_baseline.yaml --output build/acceptance/ur5-pick-live2
status = SUCCEEDED（error_code 空；policy_decision_id fc574d17-…；resource ur5_mujoco-mujoco）
报告：build/acceptance/ur5-pick-live2/report.json
```

对照：同一个 baseline 在 2026-09-20 的历史报告 `build/acceptance/ur5-pick/report.json` 也是 SUCCEEDED
⇒「UR5 能抓 box」这条能力没退化，退化的是**键名口径**，本次修复把它接回来。

## 5. 覆盖面与未做

- 现在由 UR5 官方验收（verify_pick）覆盖；**尚未**补单测（`_correct_grasp_column` 需要真模型实例，
  现有 `tests/unit/test_grasp_pose_correction.py` 只覆盖声明解析层）。下一步可加一条"执行器名键也能解出关节"的用例。
- 联合世界（`build/scenes/handoff_lab/handoff_lab_joint.json`）仍按 `config/machines/ur5e_joint.yaml` 的
  诚实登记：联合报告的 manipulation 只描述主臂（piper），UR5e 在联合世界里跑技能属 I3/I4 增量，本次不动。
