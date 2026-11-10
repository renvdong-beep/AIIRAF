# handoff_lab 演示（与 piper/ur5 同一入口）

> 2026-11-10 实证：`simulation=true`，x86 仿真（`.121`），GPU = RTX A4000。

## 1. 入口

单 MCP 端点 `http://127.0.0.1:8765/mcp`（`.121` 本地开发实例，`INSECURE=1` 仅 loopback），工具面 10 个。

| 用途 | 工具 | 关键参数 |
|---|---|---|
| 盘点 | `iraf_scene_status` | 无 |
| 发起 9 步链路 | `iraf_run_scenario` | `scenario=nominal`、`scene_id=joint-handoff`、`async_mode=true` |
| 查进度/结果 | `iraf_get_run` | `run_id`（按 `next_poll_after_s=20` 轮询） |
| 查执行事实 | `iraf_get_task` | `execution_id` |

场景名：`scene_id=joint-handoff`；supervisor 侧 `switch_name=handoff_lab_joint`（别名 `joint` 兼容旧板卡）。

## 2. 实测数据

| 模式 | 结果 | 墙钟 |
|---|---|---|
| MCP 异步 + `display=none` | **9/9、passed=true、rc=0** | **117.2s**（总 120s） |
| 窗口 `display=window`（GPU） | 9/9 | 130–155s（另有 1 次 `DOCK_DRIFTED` 偶发） |
| 本地脚本 headless | 9/9 | 108s |

GPU 期间利用率 37–53%；无窗口时 0%（链路由物理推进主导，渲染不在关键路径）。

## 3. 本机服务

| 单元 | 说明 |
|---|---|
| `iraf-provider-supervisor.service` | 8770，场景 → 单元映射（manifest `/etc/iraf-agent/manifest.yaml`） |
| `iraf-mcp-local.service` | 8765，跑**仓库源码**（`/home/coretek/AIIRAF/src`）+ 版本目录 `lib` |
| `iraf-provider@handoff_lab_joint.service` | 50054，joint 世界 Provider（窗口演示用） |

环境：`~/.config/iraf/mcp-local.env`（注册表根 `~/.local/etc/iraf`、事件库 `~/.local/state/iraf/iraf-runtime.db`）。
注册表改动后：`cp config/{scenes,providers}/*.yaml ~/.local/etc/iraf/...` → `registry_check.py --seal` → `systemctl --user restart iraf-mcp-local`。

## 4. 直接跑（不经 MCP）

```bash
bash /home/coretek/iraf-instance/scripts/run_full_scenario.sh nominal window 10   # 开窗
bash scripts/joint_demo.sh 10 10                                                # 等价入口
bash /home/coretek/iraf-instance/scripts/run_full_scenario.sh nominal none 60    # headless 验收
```

## 5. 坑（都已修）

1. `DISPLAY` 每次重启可能变（`:0`→`:1`）：`provider_launch.sh` / `joint_demo.sh` 启动时解析，别写死。
2. `mcp` 必须 `<2`：2.x 把 FastMCP 改名，`iraf_adapters.mcp.server` 起不来。
3. `submit` 的提前返回曾泄漏世界租约 → 后续提交 `IRAF-RESOURCE-BUSY` 到 TTL；已在三条路径上释放。
4. `software_gl: true` 会把窗口压回 llvmpipe；A4000 就位后应设 false。