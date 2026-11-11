# 控制面（Provider Supervisor）安装

> `sim-x86` 角色只装运行时与 Provider；控制面单独装。**一劳永逸用 `install_all.sh`**；
> 也可以按下面两个入口分开装。

## 入口 0（一键，推荐）：`deploy/sdk/install_all.sh`

```bash
# 板卡允许列表：不要写死在命令行里，用环境变量或配置文件（换环境不用改命令）
export IRAF_CONTROL_ALLOW_HOSTS="<板卡IP1>,<板卡IP2>"   # 板卡 `ip -br addr` 里的地址
bash deploy/sdk/install_all.sh \
  --sdk-wheel build/sdk/iraf_sdk-0.4.4-py3-none-any.whl \
  --runtime-bundle build/sdk/iraf-runtime-0.4.4-universal.tar.gz \
  --host-id x86-1 --run-user coretek

# 也可以写入 ~/.config/iraf/control-plane.allow（一行一个地址），或临时 --allow <IP>
```

先跑 `install-iraf.sh`（运行时 + 3 个 Provider），再跑 `install_control_plane.sh`（控制面）；
token 缺失时自动生成并提示同步到板卡。

## 入口 A（不依赖板卡 SSH）：`deploy/install_control_plane.sh`

```bash
# 注册表来源二选一：从板卡拉，或离线目录
# 允许列表来自环境变量/配置文件（见上）；--allow 仅作临时覆盖
export IRAF_CONTROL_ALLOW_HOSTS="<板卡IP1>,<板卡IP2>"
bash deploy/install_control_plane.sh --host-id x86-1 --from-board <板卡IP>
# 或：
bash deploy/install_control_plane.sh --host-id x86-1 --registry-dir /tmp/iraf-reg

# token（与板卡一致，脚本不自造）
ssh root@10.203.247.147 'cat /etc/iraf/supervisor.token' > ~/.config/iraf/supervisor.token
chmod 600 ~/.config/iraf/supervisor.token
```

脚本做 6 步：前置检查 → 准备注册表源 → 生成本机 manifest（按 host-id 过滤）→ 装 supervisor 脚本与 user 单元（`--allow` 写入白名单）→ enable-linger + 启动 → `/status` 自检。

## 入口 B（原有）：`deploy/install_second_host.sh`

与 A 等价，但**强制**从板卡 ssh 拉注册表，且白名单由脚本决定；已有脚本保持兼容。

## 板卡侧（两个入口都要做，需人审）

1. `providers/*.yaml` 的 `hosts[]` 增加本机条目（`endpoint`、`control_url: http://<本机IP>:8770`），`<本机IP>` 加进 `allowed_hosts`；
2. `registry_check.py --root /etc/iraf --seal` → 再跑一次校验（期望 `OK scenes=N providers=M`）；
3. **不重启 MCP**（按 mtime 热加载），用 `iraf_scene_status(probe=true)` 确认新 host 可见；
4. 故障转移验证：外部持住 host1 的 `__world__` 租约 → 提交应改路由到 host2（审计字段 `rerouted_to`）。

## 常见失败

| 现象 | 原因 |
|---|---|
| manifest 生成失败 | 本机缺 `~/.config/iraf/iraf-provider@*.env`（先装 `sim-x86` 运行时） |
| 启动即退出 | token 与板卡不一致（401）、或端口 8770 被占 |
| 板卡提交 `控制面不接受来自 <IP>` | `--allow-hosts` 没含板卡**出口** IP（多网卡时会变，见 `.147/.112`） |
