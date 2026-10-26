# 联合世界：UR5 开始动作时方块"从背上跳到头上"，并导致 s06 卸载失败（2026-10-26）

## 1. 现象（使用者目视）
狗走到 UR5 旁边时方块在背上托盘；UR5 一开始动作，方块**瞬间**出现在头上。
同一次运行里 `nominal --world joint` 的 s06_unload_at_b 失败、s07 连带失败（夹爪里是空的）。

## 2. 取证（A：IRAF_DEBUG_PICK=1 IRAF_DEBUG_PICK_STRIDE=5）
`build/acceptance/handoff_lab-joint-A.log`：

```
PICK_INPUTS invocation=1 positions_keys.grasp_positions = piper_joint1..piper_joint8      ← s03（piper 抓）
PICK_INPUTS invocation=1 positions_keys.grasp_positions = ur5e_shoulder_pan_joint, ur5e_elbow_joint,
              ur5e_wrist_1/2/3_joint, ur5e_rq2f85_fingers_actuator                          ← s06（ur5e 卸载）
```

⇒ 联合报告**已经有** per-robot 段（ur5e 的键是 ur5e_*，不是 piper 的），
   config/machines/ur5e_joint.yaml 里"尚未为 UR5e 解夹持事实"的注释已过期。
   所以问题不在"缺 per-robot manipulation 段"。

s06 失败原文（带调用链）：
```
位置 IK：臂关节回写失败：下列 id 在模型里没有名字 [{'id': 0, 'jnt_type': 0, 'type_zh': 'free'}]（模型关节数=36）
调用链=kinematics.py:203 ← mujoco_backend.py:3335 ← mujoco_backend.py:1195
```

## 3. 根因
`_correct_grasp_column` 把 `grasp_positions` 的键解析成关节 id 时，对
**`ur5e_rq2f85_fingers_actuator`（腱执行器，不是关节）**做"执行器→关节"反查，
`actuator_trnid[actuator, 0]` 得到 **0 = 模型的自由关节**（jnt_type=0）。
这个自由关节被塞进了 IK 的关节集 ⇒
  ① 求解器获得"通过移动载荷来满足末端目标"的自由度 ⇒ 画面上方块瞬移（从背到头上）；
  ② 回写阶段对无名关节调用 mj_id2name 得到 None ⇒ 抛"臂关节名称解析失败"（s06 失败）。

## 4. 修法（三处，都是"只允许铰接关节进 IK"）
1. `_correct_grasp_column`：解析出的 joint 必须是 `jnt_type == mjJNT_HINGE` 才进关节集；
   非铰接跳过并打一行证据 `PICK_CORRECTION_SKIPPED_NON_HINGE`。
2. `_arm_joint_names()`：同样只保留铰接关节（联合世界里 profile.joints 会混入 free 关节）。
3. `iraf_core/kinematics.solve_position_ik`：入参校验 `0 ≤ id < njnt`；回写阶段报错改为
   **点名**（id / jnt_type / 调用链），不再只给一句"名称解析失败"。

## 5. 复跑结果
```
IRAF_DEBUG_PICK=1 IRAF_DEBUG_PICK_STRIDE=5 ... scenario.py run --scene scenes/handoff_lab \
    --scenario nominal --world joint --display interactive_viewer
rc = 0
s01…s07 全部 SUCCEEDED（9/9；counts: steps=9 executed=9）
PICK_CORRECTION_SKIPPED_NON_HINGE [{"key": "ur5e_rq2f85_fingers_actuator", "joint_id": 0, "jnt_type": 0}]  ×7
```
证据：build/acceptance/handoff_lab-joint-A2.log、build/acceptance/handoff_lab/nominal/report.json

## 6. 观测者效应（必须记住）
`IRAF_DEBUG_PICK=1` 会改写 `phase_ms`（mujoco_backend.py:1101），逐拍通路 STRIDE>0 也会拖慢节拍 —— 
取证跑的数字不能当验收基线；验收一律用不带 DEBUG 的那一轮。

## 7. 未做
- 没有新增"载荷位移连续性"判据（|Δpos| 门禁）。本次靠"自由关节不得进 IK"从根上消除了瞬移来源；
  若以后还有别的瞬移通道，再加判据。
- B（按本体重建 manipulation 段）经取证判定**不需要**：联合报告已有 per-robot 段。

## 8. 干净轮验收（无 DEBUG，2026-10-26）
```
PYTHONPATH=src build/mcp-venv/bin/python scripts/scenario.py run --scene scenes/handoff_lab \
    --scenario nominal --world joint --display interactive_viewer --seconds 5
rc = 0；通过 9/9；executed=9；faults=0
s01 s02 s03 s04 s05 s02b s05b s06 s07 全部 SUCCEEDED
```
证据：build/acceptance/handoff_lab/nominal/report.json、build/acceptance/handoff_lab-joint-clean.log

## 9. 交付物一致性（0.3.21）
```
bash deploy/sdk/build_sdk.sh --target aarch64-manylinux_2_28-cp311
  → build/sdk/iraf_sdk-0.3.21-py3-none-any.whl / iraf-runtime-0.3.21-aarch64.tar.gz（219 文件）
uninstall-iraf.sh --profile sim-x86 → 六路残留全 0
install-iraf.sh  --profile sim-x86 （两个实例 piper@50052 / ur5@50053）
systemctl --user start iraf-provider@piper → active
装好的 mujoco_backend.py 含 PICK_CORRECTION_SKIPPED_NON_HINGE（1 处）
装好的 kinematics.py 含"臂关节回写失败"点名（1 处）
走 skill 链的实证：run_pick_remote --endpoint 127.0.0.1:50052
  status=SUCCEEDED wall=4.18s lifted=true bilateral_contact=true
  center_distance_m=4.178447590816052e-05（容差 0.005）
```
