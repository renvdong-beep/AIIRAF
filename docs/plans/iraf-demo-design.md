# IRAF 机器人演示方案（板卡 + x86 双轨，2026-11-17）

> 目标：给别人演示 IRAF 的机器人能力，**效果最好是有真实 MuJoCo 画面**；
> 同时准备"只有板卡"时的降级形态，保证任何现场都能讲完整。

---

## 0. 现状盘点（实测，2026-11-17）

| 端 | 事实 |
|---|---|
| **x86 `.145`**（原 `.121`，地址变了） | **有 NVIDIA RTX A4000**；仓库 0.4.6；`iraf-provider-supervisor` 在跑；`iraf-provider@piper` **running**（50052）；`ur5` **failed**；`iraf-mcp-local` 只绑 127.0.0.1:8765 |
| x86 MCP 本地配置 | `mcp-local.env` 缺 `IRAF_SAFETY_POLICY` / `IRAF_BACKEND_ENTRYPOINT` ⇒ 直接 submit 报"缺少 Runtime 配置" |
| x86 supervisor 鉴权 | `/status` 报 `IRAF-UNAUTHENTICATED`：token 与板卡侧不一致 |
| **板卡 `.147`** | AgentOS dashboard 在跑；但 `/etc/iraf` 里**没有** IRAF 的 scenes/providers/env；8765 被 `iraf-coding-*`（coding soak）占用，healthz 404；`/opt/iraf-sdk` 不在 |
| 结论 | 当前**两端都没接成一条能演示的链路**；需要先做 P0 复原 |

> 注意：`.147` 的 ssh 必须用 `ssh -F /dev/null`（否则本机 ssh 配置会串到 `.145`）。

---

## 1. 方案选型

| 方案 | 形态 | 画面 | 依赖 | 适用 |
|---|---|---|---|---|
| **A（推荐）** | `.145` 跑 MuJoCo + 板卡跑 AgentOS/Web | ✅ 真实物理画面 | 两端网络互通 + token | 现场有这台 x86 |
| B | 仅板卡：0.4.6 端侧计划 + 物理证据回放 | ⚠️ 截图/视频回放，无实时画面 | 无 | 现场只有板卡 |
| C | `.145` 单机全栈（AgentOS+MCP+Provider 同机） | ✅ | 需在 x86 装 dashboard | 允许在 x86 上演示 |

推荐 **A 为主、B 为兜底**；C 作为"连不上板卡"时的应急。

---

## 2. 方案 A：演示架构

```
            ┌──────────── 板卡 10.203.247.147 ────────────┐
            │  AgentOS chat (9119)  ←→  IRAF MCP (8765)    │
            │  演示 Web (8082, nginx)                      │
            └───────────────┬──────────────────────────────┘
                            │ HTTP + X-Iraf-Token
            ┌───────────────▼──────────────────────────────┐
            │  x86 .145：supervisor 8770                   │
            │    iraf-provider@piper 50052（MuJoCo）       │
            │    iraf-provider@ur5   50053                 │
            │    A4000 渲染 → ffmpeg 采集视频               │
            └──────────────────────────────────────────────┘
```

### 2.1 P0 复原（必须先做，约 2–3 小时）

1. **x86 MCP 配置补全**：`mcp-local.env` 增加（值取自 provider env）
   - `IRAF_SAFETY_POLICY=profiles/safety/simulation_lab.yaml`
   - `IRAF_BACKEND_ENTRYPOINT=iraf_adapters.mujoco.mujoco_backend:MujocoBackend`
2. **x86 ur5 单元修复**：查看 `journalctl --user -u iraf-provider@ur5`，重新 `ensure`；确认 50053 起来
3. **token 统一**：以板卡 `/etc/iraf/supervisor.token` 为准，覆盖 x86 `~/.config/iraf/supervisor.token`，重启 supervisor
4. **板卡装回 IRAF 0.4.6 edge-intel**：`/opt/iraf-sdk/0.4.6` 已在，但 `/etc/iraf` 被 coding soak 覆盖；
   需要 `deploy/iraf-runtime.env`、`scenes/`、`providers/`（指向 `.145` 的 endpoint/control_url）、`supervisor.token`
5. **MCP 暴露给板卡**：x86 MCP 改为 `bind 0.0.0.0` + token（不用 INSECURE），板卡 AgentOS 的 `iraf-task` 指过去
6. **连通性验收**：板卡 `iraf_scene_status(probe=true)` 能看到 `.145` 两个 provider

### 2.2 三幕演示脚本（约 5 分钟）

| 幕 | 操作 | 画面/看点 |
|---|---|---|
| 1. 架构 | Web 页打开：架构图 + 状态卡（板卡/ x86 / 3 provider） | "控制面在板卡、仿真面在 x86" |
| 2. 实时抓取 | chat：「piper 抓取 box_01」 | 60s 内 MuJoCo 窗口动，Web 实时进度条；完成后报 lift/接触 |
| 3. 联合链路 | chat：「跑 handoff_lab 的 nominal」 | 9 步进度表逐条亮；视频回放 9/9 |
| 4. 证据墙 | Web 切到证据页 | report.json、截图、视频、耗时 |

### 2.3 演示 Web（板卡 nginx :8082）

单页 + 一个只读聚合 API（60 行 Python）：

- `/` 架构图 + 三张状态卡（MCP / supervisor / chat）+ 当前 run 时间线 + 视频播放 + 证据墙 + chat iframe；
- `/api/status` 聚合 `iraf_scene_status`、supervisor `/status`、9119 健康、最近 run；
- `/media/` 放录制好的 mp4/png；
- basic auth（演示账号），只在内网。

### 2.4 素材录制（一次性，x86 有画面时）

```bash
# x86 上开窗跑一遍，ffmpeg 录屏（DISPLAY=:1）
ffmpeg -f x11grab -video_size 1280x720 -i :1 -t 90 -r 25 /opt/iraf-demo/piper_pick.mp4
```

同时保留：每步截图、`report.json`、耗时表。视频随 Web 页面分发（约 5–20 MB）。

---

## 3. 方案 B：只有板卡（兜底）

用 0.4.6 的端侧能力（已实现）：

- **实时计划**：`piper/ur5 抓取 box_01` → 0.4s 返回 IK/限位/报告（`mode=plan`）；
- **物理证据回放**：返回板卡事件库里最近一次真实 MuJoCo 抓取结果（lift/双侧接触），
  摘要只报结果，来源在字段 `mode=recorded/recorded_at`；
- Web 页同样可用：架构图 + "端侧运动学"说明 + 计划报告 + 历史证据墙 + 聊天录像。

演讲口径：「现场没有仿真主机，系统用**端侧运动学**即时计算计划，并展示同一目标此前的**真实物理执行证据**」——
既不空口，也不冒充实时。

---

## 4. 实施清单

| 阶段 | 内容 | 人日 | 验收 |
|---|---|---|---|
| P0 | 两端链路复原（token/环境/注册表/MCP 暴露/ur5） | 0.5 | 板卡 `scene_status(probe=true)` 见 2 provider；真实 piper 抓取成功 |
| P1 | 录制 piper/ur5/handoff 视频与截图 | 0.5 | 3 段 mp4 + 9 张步骤图 + report.json |
| P2 | 演示 Web（:8082） | 1.0 | 打开即见架构/状态/视频/进度 |
| P3 | 脚本 + 彩排（含拔网线切 B 方案） | 0.5 | 5 分钟全程无卡顿；兜底切换 <30s |
| P4（可选） | 板端轨迹可视化（matplotlib 3D 计划图/GIF） | 0.5 | 计划结果附一张轨迹图 |

---

## 5. 风险与兜底

| 风险 | 兜底 |
|---|---|
| x86 不在/网络不通 | 切方案 B（端侧计划 + 证据回放），Web 同样能讲 |
| 本地 9B 回答慢 | 预置 3 条"演示问答卡"，或把 chat 模型临时切到 `.89` 网关 |
| MCP 会话断（历史踩坑） | 一键脚本 `restart-demo.sh`：重启 MCP+dashboard 并预热 provider |
| 视频缺失 | 用历史截图 + report 数字讲，Web 证据墙仍成立 |
| 观众追问"是真跑的吗" | 方案 A 直接看窗口/视频；方案 B 明确说"端侧计算 + 历史物理证据"，字段可查 |

---

## 6. 一句话

**有 `.145` 就用方案 A（真实 MuJoCo + Web）；没有就切方案 B（端侧计划 + 证据回放）。**
两条路共用同一个 Web 页与同一套话术，现场只需切换一个开关。
