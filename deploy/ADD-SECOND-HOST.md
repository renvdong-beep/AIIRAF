# 第二台 x86 主机接入清单（阶段 E3）

> 目标：让 `.121` 之外的第二台 x86 也能跑 IRAF Provider，并让板卡的**多主机调度/故障转移**真正生效。
> 现状（2026-11-09）：`192.168.100.128` 只开 ssh，**未装 supervisor（8770 关闭）、未跑 Provider（50054 关闭）**，
> 所以本清单是**待人工执行**的准备件，不是已验证结果。

## 0. 前置条件

| 项 | 要求 |
|---|---|
| 操作系统 | Linux x86_64，与 `.121` 同代的 glibc / python3.10 |
| Python 依赖 | `mujoco` / `numpy` / `PyYAML`（该机要能独立跑 provider） |
| 仓库 | `AIIRAF`（与板卡同一 `release/0.4.2` 分支） |
| 账号 | `coretek` + 免密 sudo（装 `/etc/iraf-agent`、`loginctl enable-linger`） |
| 网络 | 板卡可达（取注册表镜像）；板卡能访问本机 8770 与 provider 端口 |
| Provider env | `~/.config/iraf/iraf-provider@<scene>.env`（含 `IRAF_PROVIDER_PORT`、`model_path`、display） |

## 1. 安装（在**新主机**上执行）

```bash
scp <板卡>:/etc/iraf/supervisor.token ~/.config/iraf/supervisor.token   # 与板卡同一个 token
chmod 600 ~/.config/iraf/supervisor.token
bash AIIRAF/deploy/install_second_host.sh --host-id x86-2 --board 10.203.247.147 \
     --env-root ~/.config/iraf --repo ~/AIIRAF
```

脚本做 7 件事：前置检查 → 取板卡注册表镜像 → `emit_host_manifest` 生成本机 manifest →
装 supervisor 脚本与 user unit（`--bind 0.0.0.0:8770`、`--allow-hosts 127.0.0.1,<板卡>`）→
`enable-linger` + 启动 → 本机 `/status` 自检 → 打印板卡侧待办。

成功标志：`systemctl --user is-active iraf-provider-supervisor` 为 `active`，且
`curl -H "X-Iraf-Token: $TOK" http://127.0.0.1:8770/status` 返回 `providers` 数量与场景数一致。

## 2. 板卡侧登记（需人审，会改运行期路由）

```yaml
# /etc/iraf/providers/iraf-handoff-lab.yaml（示例：新增第二台）
hosts:
  - id: x86-1
    endpoint: 10.203.247.121:50054
    control_url: http://10.203.247.121:8770
    capacity: {cores: 4, gpu: none}
    priority: 10
  - id: x86-2
    endpoint: 192.168.100.128:50054
    control_url: http://192.168.100.128:8770
    capacity: {cores: 16, gpu: none}
    priority: 20
allowed_hosts: [10.203.247.121, 192.168.100.128]   # 必须与 hosts 的主机集合一致
```

然后：

```bash
# 原子写 + 重新封签 + 校验（不需要重启 MCP，热加载）
python3 /root/iraf-instance/scripts/registry_check.py --root /etc/iraf --seal
python3 /root/iraf-instance/scripts/registry_check.py --root /etc/iraf \
  --profiles-root /opt/iraf-sdk/0.4.1/profiles      # 期望 OK scenes=4 providers=4
```

## 3. 验证（按顺序）

| # | 步骤 | 期望 |
|---|---|---|
| 1 | `iraf_scene_status(probe=True)` | 两台 host 都在 `providers[].hosts[]`，且每台带 `runtime.load_1m` |
| 2 | 板卡 `POST <host2>/ensure?scene=joint` | `status=OK`，本机 50054 LISTEN |
| 3 | **跨主机并行**：占住 host1 的 `host:x86-1/__world__` 租约 | 提交 `scene_id=joint-handoff` **改路由**到 host2，响应 `resolved.endpoint` 指向 host2，审计有 `rerouted_to` |
| 4 | **容量耗尽**：两台都占住 | 立即返回 `IRAF-CAPACITY-EXHAUSTED`（不排队） |
| 5 | **故障转移**：停 host2 supervisor | 提交仍能落到 host1（priority 10）；两台都停 → `IRAF-PROVIDER-UNAVAILABLE`（秒级，不挂起） |

## 4. 回滚

1. 删掉 `providers/*.yaml` 里新增的 `hosts[]` 条目与 `allowed_hosts` 项；
2. `registry_check --seal` → `OK`；
3. 在新主机上 `systemctl --user disable --now iraf-provider-supervisor`；
4. 确认 `iraf_scene_status(probe=True)` 只剩一台 host。

## 5. 已知限制

- 第二台主机的 **Provider 版本必须与板卡 SDK 相同**（本仓 `release/0.4.2`），否则协议/技能契约可能不匹配；
- `control_url` 依赖板卡 `_supervisor_url(host)` 优先取注册表字段（已实现）；**改 `iraf_core` 任何模块都要与
  `server.py` 一起下发**（2026-11-09 踩过：只发 `server.py` 导致 `control_url` 恒空）；
- 跨主机**自动改路由**只在 `health.failover: next` 时生效，且当前单测覆盖、**未做真机验证**。
