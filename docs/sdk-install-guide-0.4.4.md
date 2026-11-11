# IRAF 0.4.4 安装与使用说明

> 本文件与产物同目录：`iraf_sdk-0.4.4-py3-none-any.whl`、`iraf-runtime-0.4.4-universal.tar.gz`、
> `manifest.json`、`sbom.spdx.json`、`sha256sums.txt`。
> 产物里的 runtime bundle **架构无关**（纯源码 + 三维资产，无编译产物）；依赖（mujoco/grpcio 等）
> 由安装器按目标机 Python 版本安装。

## 0. 部署形态（先看懂再动手）

```
板卡（aarch64，例如 10.203.247.x）              x86 仿真机（Ubuntu 22.04）
┌──────────────────────────────┐              ┌───────────────────────────────────┐
│ AgentOS / chat（可选）        │   HTTP/JSON  │ iraf-provider-supervisor :8770     │
│ iraf-mcp-http :8765（10 工具）│ ───────────► │ iraf-provider@piper      :50052    │
│ /etc/iraf/scenes|providers    │              │ iraf-provider@ur5        :50053    │
│ （注册表，热加载）             │              │ iraf-provider@handoff_lab_joint :50054
└──────────────────────────────┘              └───────────────────────────────────┘
```

- **板卡只做控制面**：MCP 端点 + 场景/主机注册表，**不跑仿真**；
- **x86 跑仿真**：每个场景一个 Provider（MuJoCo），由 supervisor 统一启停；
- 同一时刻只允许一个“世界”在跑（租约保证），提交会显式返回 `IRAF-RESOURCE-BUSY`。

## 1. 前置检查（两端都要）

| 项 | 要求 |
|---|---|
| OS | x86：Ubuntu 22.04；板卡：厂商系统（aarch64） |
| Python | 3.10+（x86 侧 `/usr/bin/python3`） |
| 账号 | 具备 `sudo`；systemd **user** 实例可用 |
| 端口 | x86 空闲：`8770`、`50052`、`50053`、`50054`；板卡空闲：`8765` |
| 磁盘 | x86 预留 ≥ 1.5 GB（三维资产约 110 MB，安装后含 mesh 约 250 MB） |
| 图形（可选） | 要开窗演示：X 已登录（`DISPLAY` 与 `XAUTHORITY` 可用） |

```bash
# 校验产物（在放置产物的目录执行）
sha256sum -c sha256sums.txt
```

预期：两行 `OK`：
`iraf_sdk-0.4.4-py3-none-any.whl: OK`、`iraf-runtime-0.4.4-universal.tar.gz: OK`。

## 2. x86 仿真机：一条命令装完

```bash
# 0) 拿到仓库（安装器需要 deploy/ 与 scripts/）
git clone <仓库地址> ~/AIIRAF && cd ~/AIIRAF

# 1) 板卡允许列表：不要写死在命令里，放配置
export IRAF_CONTROL_ALLOW_HOSTS="<板卡IP1>,<板卡IP2>"   # 板卡上 `ip -br addr` 查，多网卡全写上

# 2) 一键安装：运行时 + 3 个 Provider + 控制面
bash deploy/sdk/install_all.sh \
  --sdk-wheel      ./iraf_sdk-0.4.4-py3-none-any.whl \
  --runtime-bundle ./iraf-runtime-0.4.4-universal.tar.gz \
  --host-id x86-1 \
  --run-user "$(id -un)"
```

安装器行为：
- 预检（架构/Python/端口/磁盘/声明一致性）不过 → **不写任何文件**；
- 中途失败 → 自动回滚本次创建的内容并还原 `current` 链接；
- 装完生成：`~/.local/opt/iraf-sdk/0.4.4/`、`current` 链接、3 份 Provider 单元与装配单、
  `/etc/iraf-agent/manifest.yaml`、supervisor user 单元。

**token**：脚本缺 `~/.config/iraf/supervisor.token` 时会自动生成并提示。该 token **必须与板卡一致**：

```bash
# 在 x86 上执行：把本机 token 推到板卡（首次或轮换时）
ssh root@<板卡IP> "cat > /etc/iraf/supervisor.token && chmod 600 /etc/iraf/supervisor.token" \
  < ~/.config/iraf/supervisor.token
```

**验证（x86）**：

```bash
systemctl --user is-active iraf-provider-supervisor          # active
TOK=$(cat ~/.config/iraf/supervisor.token)
curl -s -H "X-Iraf-Token: $TOK" http://127.0.0.1:8770/status | \
  python3 -c "import json,sys;d=json.load(sys.stdin);print(d['host_id'],d['active_scene'],len(d['providers']))"
# 期望：x86-1 None 3
```

## 3. 板卡：注册表 + MCP

```bash
# 1) 注册表随 bundle 的 config/ 分发（scenes/providers 各 3 份）
cp -a deploy_目录/config/scenes/*.yaml    /etc/iraf/scenes/
cp -a deploy_目录/config/providers/*.yaml /etc/iraf/providers/

# 2) 封签 + 校验（改过就必须重封签）
python3 /root/iraf-instance/scripts/registry_check.py --root /etc/iraf --seal
python3 /root/iraf-instance/scripts/registry_check.py --root /etc/iraf \
  --profiles-root /opt/iraf-sdk/current/profiles
# 期望：sealed N files ... / OK scenes=3 providers=3

# 3) 重启 MCP（改了 Python 代码才需要；只改注册表是热加载）
systemctl restart iraf-mcp-http && sleep 8
timeout 4 curl -s -o /dev/null -w '%{http_code}\n' http://127.0.0.1:8765/healthz   # 200
```

**板卡注册表里必须写对两件事**：
- `providers/*.yaml` 的 `hosts[]`：`endpoint: <x86IP>:50052`、`control_url: http://<x86IP>:8770`；
- `allowed_hosts` 含 `<x86IP>`；`scenes/*.yaml` 的 `switch_name` 与 supervisor 单元实例名一致
  （本版：`piper` / `ur5` / `handoff_lab_joint`；`joint-handoff` 场景另带别名 `joint`）。

## 4. 端到端验收（在板卡上执行）

```bash
python3 - <<'PY'
import json, time, urllib.request
URL = "http://127.0.0.1:8765/mcp"
TOK = open("/etc/iraf/mcp-client.token").read().strip()
H = {"Authorization": "Bearer " + TOK, "Content-Type": "application/json",
     "Accept": "application/json, text/event-stream"}
def rpc(p, sid=None, timeout=240):
    h = dict(H)
    if sid: h["Mcp-Session-Id"] = sid
    r = urllib.request.urlopen(urllib.request.Request(URL, data=json.dumps(p).encode(),
                                                      headers=h, method="POST"), timeout=timeout)
    raw = r.read().decode(); s2 = r.headers.get("Mcp-Session-Id")
    for line in reversed(raw.splitlines()):
        if line.startswith("data:"): return json.loads(line[5:].strip()), s2
    return json.loads(raw), s2
def call(t, a, sid, timeout=240):
    r, _ = rpc({"jsonrpc":"2.0","id":9,"method":"tools/call",
                "params":{"name":t,"arguments":a}}, sid, timeout)
    txt = ((r.get("result") or {}).get("content") or [{}])[0].get("text","")
    try: return json.loads(txt)
    except Exception: return {"_raw": txt[:300]}

_, sid = rpc({"jsonrpc":"2.0","id":1,"method":"initialize",
              "params":{"protocolVersion":"2025-03-26","capabilities":{},
                        "clientInfo":{"name":"verify","version":"1"}}})
print("scenes:", call("iraf_scene_status", {}, sid).get("mode"))

t0 = time.time()
d = call("iraf_submit_task", {"text": "抓取 box_01", "scene_id": "ur5-pick"}, sid)
print("ur5: %.1fs %s" % (time.time()-t0, d.get("status")))          # 期望 SUCCEEDED，约 60s

t0 = time.time()
d = call("iraf_run_scenario", {"scenario":"nominal","display":"none",
                               "scene_id":"joint-handoff","async_mode":True}, sid)
rid = d.get("run_id")
print("handoff run:", d.get("status"), rid)
while rid:
    time.sleep(15)
    run = (call("iraf_get_run", {"run_id": rid}, sid).get("run") or {})
    st = (run.get("status") or "").lower()
    if st in ("succeeded","failed"):
        steps = run.get("steps") or []
        ok = sum(1 for s in steps if s.get("status")=="SUCCEEDED")
        print("handoff: %s %d/%d wall=%ss" % (st, ok, len(steps), run.get("wall_s")))
        break
PY
```

验收口径：ur5 `SUCCEEDED`；handoff `succeeded` 且 **9/9**、`passed=true`。
典型耗时：ur5 60s 左右；handoff 90–110s（冷启动或高负载会更久，**不要放宽判据**）。

## 5. 常见问题

| 现象 | 原因 / 处理 |
|---|---|
| 板卡提交报 `控制面不接受来自 <IP>` | x86 supervisor 的允许列表没含板卡**出口** IP：`export IRAF_CONTROL_ALLOW_HOSTS=...` 后重装控制面或改单元后 `daemon-reload` |
| `IRAF-SCENE-NOT-FOUND` | 板卡注册表 `switch_name` 与 supervisor 实例名不一致，或 manifest 未重生成 |
| `IRAF-PROVIDER-UNAVAILABLE` | x86 单元没起/端口没监听：`journalctl --user -u iraf-provider@ur5 -n 50` |
| `IRAF-RESOURCE-BUSY` | 上一个执行未释放世界租约；等 TTL 或用 `iraf_lease(action="list")` 查后释放 |
| 开窗报 `Failed to open display` | `DISPLAY` 过期：本版启动器会自动解析；手工调试时用 `DISPLAY=:N`（`ls /tmp/.X11-unix` 查 N） |
| `ModuleNotFoundError: mcp` | 缺 MCP 宿主依赖：`pip install 'mcp<2'`（**2.x 无 FastMCP**） |
| 装完 `handoff` 起不来 | 确认 `sim-x86.services` 含 `iraf-provider@handoff_lab_joint`（0.4.4 已含） |

## 6. 回滚与卸载

```bash
# 回滚到旧版本：只动软链
ln -sfn ~/.local/opt/iraf-sdk/<旧版本> ~/.local/opt/iraf-sdk/current
systemctl --user restart iraf-provider-supervisor

# 规范卸载（按安装记录删除；不会误删手改文件）
bash deploy/sdk/uninstall-iraf.sh --profile sim-x86 --dry-run
bash deploy/sdk/uninstall-iraf.sh --profile sim-x86
```

## 7. 给自动化 agent 的最小流程（照做，不要即兴发挥）

1. `sha256sum -c sha256sums.txt` —— 不过就停，报错给上游；
2. `export IRAF_CONTROL_ALLOW_HOSTS=<板卡IP...>`；
3. 跑 `install_all.sh`（参数见 §2），把完整输出留档；
4. `systemctl --user is-active iraf-provider-supervisor` 与 `curl /status` —— 不是 `active`/`providers=3` 就停；
5. 板卡：复制注册表 → `registry_check --seal` → 校验 `OK scenes=3 providers=3`；
6. 跑 §4 的验收脚本，记录 ur5 状态与 handoff 的 `9/9 + passed=true`；
7. 失败时**只做诊断**：收集 `journalctl --user -u iraf-provider@*`、supervisor `/status`、
   `runs`/`leases` 表；**不要**改判据、伪造成功、手工改 mesh 或 `digest`。

禁止事项：不要用 `execute_code` 直接驱动 MuJoCo 绕开 TaskFlow；不要把失败包装成成功；
不要把仿真结论表述为真机能力（`simulation=true`）。