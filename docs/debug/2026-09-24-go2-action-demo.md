# Go2 动作演示（一次跑完七个动作，看得见；2026-09-24）

## 配方（可复跑）

```bash
# 每个动作 10 s；`--seconds 5` = **每条命令后保持窗口 5 s**（否则 stand/stop 会一闪而过）
DISPLAY=:0 XAUTHORITY=/run/user/$(id -u)/gdm/Xauthority MUJOCO_GL=glfw \
PYTHONPATH=src:scripts python3 scripts/scenario.py interact \
  --scene scenes/handoff_lab --robot unitree_go2 \
  --commands-from build/iraf-a6a14/demo-actions.txt \
  --display interactive_viewer --seconds 5 \
  --report build/iraf-a6a14/demo-all-report.json \
  --transcript build/iraf-a6a14/demo-all-transcript.jsonl
```

`build/iraf-a6a14/demo-actions.txt`（七条，命令语法与 S1 交互一致）：

```
stand duration_ms=10000
locomote velocity={"vx_mps":0.1,"vy_mps":0.0,"wz_rad_s":0.0} duration_ms=10000
locomote velocity={"vx_mps":-0.1,"vy_mps":0.0,"wz_rad_s":0.0} duration_ms=10000
locomote velocity={"vx_mps":0.0,"vy_mps":0.0,"wz_rad_s":0.3} duration_ms=10000
locomote velocity={"vx_mps":0.0,"vy_mps":0.0,"wz_rad_s":-0.3} duration_ms=10000
dock_for_handoff
stop
```

**必须显式 `--display interactive_viewer`**：`--display auto` 在本机**不开窗**（实测 frames=None、
报告 display=auto），你以为在演示其实什么都没显示。

## 两条实测教训（都在本机复现过）

1. **"10 秒"是仿真时长，墙钟按路径差 8 倍**：
   stand/stop 是纯 PD（无 MPC）——**快于实时**（10 s 仿真 → 2.71 s 墙钟）；
   行走/停靠走 MPC——**约 2.3× 慢于实时**（4 s 仿真 → 9.14 s 墙钟；停靠 13.5 s → 18.0 s）。
   本机瓶颈是 **4 核 CPU 争用**（llvmpipe 软件渲染 + MPC 子进程 + ToDesk + 基线负载 1.73）：
   实测硬件 GL **更慢**（4 s → 10.76 s）且画面几乎不亮；`render_hz` 30→10 **无改善**。
   要让行走接近实时：暂停/关闭 ToDesk 最有效。
2. **技能租约 TTL 必须覆盖墙钟耗时**（否则表现为"动作失败"，其实是等太久）：
   `skills/locomote/skill.yaml` 的 `timeoutSeconds` 原为 30 s，10 s 仿真的行走在 MPC 路径下
   ≈23 s 墙钟 + 冷启动 5~8 s ⇒ **租约过期** ⇒ 交互入口报
   `执行未返回结构化 status（raw_result）`。修法：TTL 30 → **60 s**（提交 ce25bc0）。
   对照自洽：`dock_for_handoff` 的 TTL 是 40 s ⇒ 同机 SUCCEEDED；1 s 时长的 locomote 墙钟 8.11 s < 30 ⇒ SUCCEEDED。
   注意：放宽的是**等待**，安全边界仍由安全策略的 `max_duration_ms: 30000`（仿真时长上限）与
   `max_speed_mps` 约束。

## 本次实测（七条）

| 命令 | 状态 | 墙钟 s | 本段位移 m | 到站位 m |
| --- | --- | --- | --- | --- |
| stand duration_ms=10000 | SUCCEEDED | 10.30 | - | 0.4569 |
| locomote velocity={'vx_mps': 0.1, 'vy_mps': 0. | SUCCEEDED | 12.89 | 0.0291 | 0.4860 |
| locomote velocity={'vx_mps': -0.1, 'vy_mps': 0 | SUCCEEDED | 10.39 | 0.0106 | 0.4966 |
| locomote velocity={'vx_mps': 0.0, 'vy_mps': 0. | FAILED | 72.87 | 0.0370 | 0.4644 |
| locomote velocity={'vx_mps': 0.0, 'vy_mps': 0. | SUCCEEDED | 59.58 | 0.0265 | 0.4388 |
| dock_for_handoff | FAILED | 93.77 | 0.4107 | 0.0290 |
| stop | SUCCEEDED | 6.96 | 0.0346 | 0.0625 |
（来源：build/iraf-a6a14/demo-all-report.json / demo-all-transcript.jsonl，display=interactive_viewer）
