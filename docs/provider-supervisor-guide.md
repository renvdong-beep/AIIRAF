# Provider Supervisor 使用与运维指南（阶段 C）

> 适用版本：0.4.2 + 控制面单点化改造（A/B/C 阶段）
> 口径：`simulation=true`；本文只描述 x86 侧的 Provider 控制面，不涉及技能与安全判定。

## 1. 它是什么

`iraf-provider-supervisor` 是跑在 x86 仿真主机上的**唯一 Provider 生命周期控制面**：

- 数据来源：`/etc/iraf-agent/manifest.yaml`（由 `scripts/emit_host_manifest.py` 从板卡注册表派生）；
- 同一时刻只让**一个 world** 在线（`ensure`：先起目标 → 等端口 LISTEN → 再停其余）；
- 守护：目标 unit 掉线 → 退避 `1s/3s/9s` 重启；连续 3 次失败标记 `DEGRADED`；
- 全量链路：`/scenario` 在本机跑 `run_full_scenario.sh`（多机器人同一世界），返回 `passed/steps/rc/wall_s`；
- 控制面鉴权：`X-Iraf-Token` + 客户端主机白名单。

板卡**不再** SSH 到 x86，也不再自己启停进程（私钥已从板卡移除）。

## 2. 部署形态

| 项 | 值 |
|---|---|
| unit | `~/.config/systemd/user/iraf-provider-supervisor.service` |
| 监听 | `0.0.0.0:8770` |
| 令牌 | `~/.config/iraf/supervisor.token`（600）；板卡侧 `/etc/iraf/supervisor.token`（600，内容一致） |
| 白名单 | `--allow-hosts 127.0.0.1,10.203.247.147` |
| 状态 | `GET /status`（需 token） |
| 健康 | `GET /healthz`（免 token） |

```bash
systemctl --user status iraf-provider-supervisor
journalctl --user -u iraf-provider-supervisor -n 50
```

## 3. 接口

```bash
TOK=$(cat ~/.config/iraf/supervisor.token)

# 状态（含每个 provider 的 unit/port/active/degraded/attempts 与最近 history）
curl -s -H "X-Iraf-Token: $TOK" http://127.0.0.1:8770/status | python3 -m json.tool

# 切到某场景（单 world：先起后停）
curl -s -X POST -H "X-Iraf-Token: $TOK" "http://127.0.0.1:8770/ensure?scene=ur5"

# 停某场景
curl -s -X POST -H "X-Iraf-Token: $TOK" "http://127.0.0.1:8770/stop?scene=ur5"

# 全量链路（x86 本机执行，最长 900s）
curl -s -X POST -H "X-Iraf-Token: $TOK" \
  "http://127.0.0.1:8770/scenario?scenario=nominal&display=none&hold=60"
```

`scene` 用 **switch_name**（`piper` / `piper-y05` / `ur5` / `joint`），不是 `scene_id`（`piper-pick`）。

## 4. 常见操作

**新增一台 host**：在该机跑 `emit_host_manifest.py` 得到 `/etc/iraf-agent`，装 supervisor + 同样的 token，
再在板卡 `providers/*.yaml` 的 `hosts[]` 增加条目（`control_url` 指向该机 8770），`--seal` 后热加载。

**换 token**：两端同时更新（x86 `~/.config/iraf/supervisor.token`、板卡 `/etc/iraf/supervisor.token`），
`systemctl --user restart iraf-provider-supervisor` + 板卡 `systemctl restart iraf-mcp-http`。

**supervisor 起不来**：板卡提交会**秒级**返回 `IRAF-PROVIDER-UNAVAILABLE`（`supervisor 不可达`），不会挂起；
按 `journalctl --user -u ...` 排查（常见：端口占用、manifest 校验失败、token 文件权限）。

## 5. 已知边界

- 控制面是**单点**：挂了以后所有提交显式失败，恢复需人工（板卡已无 SSH 兜底）。
- `/scenario` 是**同步阻塞**调用（最长 900s）；长链路期间客户端要有足够超时。
- 没有租约：同一 world 的并发提交目前会互相 restart（3–5× 变慢，长任务会被打断），由阶段 D 补。
- `ensure` 只在 `active_scene` 上守护；`active_scene=None` 时 monitor 不动作（演练/待命态）。
