# 2026-09-20 Viewer 跑通两台机型：三处"Piper 同名掩盖"的后端缺陷

背景：把 UR5e 抓取放进 MuJoCo Viewer（`DISPLAY=:0`）时连续暴露三个缺陷 ——
它们都能通过验收脚本，因为验收走的路径与显示路径不同（Piper 上两者恰好都正常）。

## 1. `move_joint` 这条路径没做"关节名 → 执行器名"翻译

```
Home 动作失败：status=FAILED error=IRAF-EXECUTION-FAILED
reason=actuator not found: shoulder_pan_joint
```

对外契约（`profile.joints`、move_joint 参数）用**关节名**
（`shoulder_pan_joint`），而 `ctrl` 数组按**执行器名**（`shoulder_pan`）索引。
Piper 两者同名（joint1..joint8）掩盖了差异；UR5e 不同名即失败。
此前只在场景生成器里做了翻译（`to_actuator_names`），`move_joint` 这条路径漏了。

修复：新增 `_actuator_channel(name)`（关节名 → 执行器通道；已是执行器名则原样返回；
解析不到即显式失败），并在 `move_joint`、`_move_trajectory`（含前馈键）、
`_set_controls` 三处统一使用。`move_joint` 的状态仍按**关节名**回写
（新增 `_joint_name_of`），与 `profile.joints` 及返回值口径一致。

## 2. 腱驱动关节没有可直接下发的 ctrl 通道

```
hold_current_pose → ValueError: 找不到关节或执行器: rq2f85_right_driver_joint
```

`profile.joints` 声明了 2F-85 的两个 driver 关节，但它们由**单个 tendon 执行器**
经 equality + 耦合杆驱动；MuJoCo 里 tendon 执行器的 `trnid` 指向 **tendon 而非关节**，
因此这些关节没有可直接写的 ctrl 通道。按关节角硬写会破坏欠驱动一致性。

修复：区分两种语义 —— `_direct_actuator_channel()`（能控制才控制，返回 None 表示
"无直接通道"）与 `_actuator_channel()`（必须能下发，找不到即失败）；
`hold_current_pose` 对腱驱动关节显式跳过。关节名根本不存在时仍然失败（那是配置错误）。

## 3. Viewer 手工白名单丢掉 `gravity_feedforward`（同一报告两条路径行为不一致）

Viewer 用 `_manipulation_from_scene_report()` 从场景报告拼后端配置，里面是一份
**手工维护的字段白名单**。它漏掉了 `gravity_feedforward`，于是 viewer 里的 UR5e
抓取丢掉了重力前馈补偿，退化为 16.6mm 失败；而**同一份场景报告**走统一验收却是
0.000119m —— 同一配置两条路径行为不同，是最难查的一类不一致。

修复：不再维护白名单，**直接透传场景报告的 `gripper` 段**（场景报告就是
manipulation 配置的唯一真源，除 targets 外无需裁剪）。这与部署配置的教训同源
（`deploy/iraf-runtime.env` 手写 JSON 也曾两次静默过期）。

顺带修掉 viewer 的视觉坐标读取：只读顶层 `vision_world_position_m` 会让
**已接入检测器的机型静默退回场景真值**；现在兼容检测器输出的
多目标格式（`targets[]` 按 id 选取）。

## 4. Viewer 入口去机型化

`scripts/view_piper_mujoco.py` → `scripts/view_mujoco.py`（通用入口）：
- `--baseline` 由可选改为**必填**，场景 / Profile / 视觉来源全部由配置声明解析
  （原先默认写死 Piper 的路径）；
- 旧名保留为薄包装（打印 deprecated 后转发，默认带 Piper 基线），
  既有操作手册命令与 Viewer 的 Mesa/C++ ABI 启动前置不受影响。

## 5. 验证

```
UR5e  Viewer 窗口内抓取：status=SUCCEEDED  抓取点偏差 0.00011876256957761631  抬升 0.041208m
      （与统一验收数值逐位一致；此前 viewer 内为 16.6mm 失败）
Piper Viewer 窗口内抓取：SUCCEEDED  偏差 0.002826042685491929  抬升 0.087224m
Home 动作：UR5e 与 Piper 均 SUCCEEDED（UR5e 此前报 actuator not found）
四条验收：UR5e pick_object / UR5e visual_pick / Piper pick_object / Piper visual_pick 全 SUCCEEDED
单元测试 263 项，失败项仍为既有 1 项 + 4 项导入错误

窗口确认（xwininfo）：
  "MuJoCo : ur5e"               1280x720+5+29  +5+56
  "MuJoCo : piper_description"  1280x720+5+29  +5+355
```

启动命令（需要 Ubuntu 桌面会话与 Mesa/C++ ABI 前置）：

```
DISPLAY=:0 XAUTHORITY=/run/user/1000/gdm/Xauthority MUJOCO_GL=glfw \
PYTHONPATH=src:scripts python3 scripts/view_mujoco.py \
    --baseline config/ur5_simulation_baseline.yaml --seconds 0 --pick-duration-ms 12000
```

## 6. 教训

"Piper 上能跑"不等于"契约正确"：本轮三个缺陷全部由**同名巧合**或
**手工字段清单**掩盖。凡是"名字相同所以看不出差异"的路径（关节名/执行器名、
字段白名单、默认路径）都必须用第二台机型或结构性校验去证伪。
