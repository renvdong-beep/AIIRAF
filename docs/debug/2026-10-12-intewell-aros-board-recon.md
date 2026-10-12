# 2026-10-12 边缘板卡实机侦察（Intewell AROS 实验室板卡，10.203.247.74）

- 目的：为 IRAF 在**真机**上的双域部署（智能域 + 运控域）建立事实基线
- 手段：**只读采集**（`scripts/board_collect.sh`，22 个探针全部 rc=0）
- 证据：`build/board/e300-lab/{collect.json, probe-status.tsv, raw/*.txt}`（`build/` 已被 gitignore）
- 声明回填：`profiles/boards/intewell_aros_lab.yaml`（status 仍为 `unverified`）
- 本记录只写**实测到的事实**与**由事实直接推出的结论**；推断项一律标注为推断

---

## 1. 结论摘要（先看这五条）

```
① 板卡可达且是真机：Intewell AROS-R1.2.1（Hyper V3.1.2 / Linux 域 / RTOS），aarch64，4 核，16 GB RAM，468 GB NVMe
② AgentOS **已在运行**：宿主 host_worker（8083）+ CLI 常驻 dashboard（9119）+ 7 个 agentos-* 容器（含 Postgres）
③ 板卡上**尚无任何 IRAF** ⇒ 全新安装，不需要清理历史残留
④ **端口无冲突**：IRAF 声明的 8765 / 50051 都空闲（AgentOS 占 22/5432/8080-8083/8090/8888/9119）
⑤ 平台当前**不健康**：load 6.3（4 核）但 CPU 97.6% 空闲 ⇒ 负载来自 6 个 D 状态 VPU 内核线程；
   另有 31 个僵尸进程（父进程 searxng）。安装/验收前必须记录该背景，否则会误判成 IRAF 缺陷
```

---

## 2. 板卡身份与资源（raw/identity.txt、os_release.txt、iaros_version.txt、resources.txt）

```
uname        Linux localhost.localdomain 6.6.10 #1 SMP PREEMPT … aarch64
systemd-detect-virt = none（不是来宾；是 Hyper 的**宿主 Linux 域**）
cmdline      rw fw_devlink=off root=/dev/nvme0n1p2 rootwait smmu=off iommu.passthrough=1 maxcpus=4
/etc/os-release      ID=Intewell  VERSION=V1.2.1（品牌层）
/etc/back.os-release PRETTY_NAME="openEuler 24.03 (LTS-SP1)"（内核发行版层）
/etc/iaros-version   套件 Intewell AROS-R1.2.1
                     Hyper  Intewell AROS-Hypervisor-V3.1.2-R1.2.1
                     Linux  Intewell AROS-Linux-V6.12.43-R1.2.1
                     RTOS   Intewell AROS-RTOS-V3.1.1-R1.2.1
资源                  4 核 / MemTotal 15979 MB（可用 13443）/ swap 0 / / 468G（用 66G，余 378G）
运行时                Python 3.11.6 + pip 23.3.1（清华源 /etc/pip.conf）/ glibc 2.38 / systemd 255
```

**⚠ 声明 vs 实测不一致（必须留档，不得抹平）**：`/etc/iaros-version` 写 `AROS-Linux-V6.12.43`，
而实测 `uname -r` 是 `6.6.10`。两者相差一个大版本，属"清单声明 ≠ 运行内核"，需要研发确认是
清单未随内核更新，还是跑的不是该发布包（这正是 `profile_check --board` 要暴露的那类差异）。

## 3. 域拓扑（raw/domains.txt、net_topology.txt）

```
dmesg 实测两个来宾（本机是宿主）：
  vm-0：rootfs_vm1.img，2×virtio-net（vm-0-net0 / vm-0-net1）
  vm-1：rootfs_vm2.img，1×virtio-net（vm-1-net0）
本机接口：
  br0  10.203.247.74/23   ← 外网（默认网关 10.203.246.1）
  br1  10.192.1.3/24      ← **域间内网**；br1 邻居只有 10.192.1.2，lladdr fe:fe:00:00:00:01
  br-58970757e05a 172.18.0.1/16 ← AgentOS 容器网；docker0 down
```
**推断（未证实，待签名清单或人工确认）**：`10.192.1.2`（MAC 形如 `fe:fe:00:00:00:01` 的 Hyper 虚拟网卡）
很可能是**运控域**（来宾 VM1），即 `br1` 是智能域↔运控域的域间通道。
**不做的事**：不据此把构型写成 `MQ50-E300-3VM-*` —— 铁律 6.10 要求构型名与角色映射只能来自
签名 HyperProfile/Release manifest，而本机**没有**该清单（见 §6）。

## 4. AgentOS 的真实形态（raw/agentos_*.txt、docker_state.txt）

```
宿主进程
  pid 77868   /opt/IntewellAgent/agentbase/bin/python3.10 -m agentos.host_worker   （已 2 天，健康口 127.0.0.1:8083）
  pid 165246  …/intewell-agent-cli dashboard --host 0.0.0.0 --port 9119            （CLI 常驻 + node TUI）
              子进程 tui_gateway.slash_worker --session-key 20261012_173601_… --model deepseek-flash
容器栈（镜像 tag 1781d6d3-arm64，均 Up 5h）
  agentos-db(postgres, 127.0.0.1:5432) / agentos-relay(healthy) / agentos-worker-1 / agentos-worker-2
  agentos-task-api / agentos-searxng(127.0.0.1:8888→8080) / agentos-searxng-network-monitor
配置
  /etc/agentos/host-worker.env  AGENTOS_WORKER_ID=intewell-host-worker-01、
                                AGENTOS_STORE_BACKEND=postgres、AGENTOS_REQUIRE_ISOLATION=true
  /etc/agentos/host-bindings.json  [{tenant_id: default, principal_id: *, session_id: *, host_id: host-worker-01}]
  安装物 /opt/IntewellAgent/{agentbase, node-v22.23.2-linux-arm64,
                             wheel/intewell_agent-1.0.0-py310-none-linux_aarch64.whl}
```

**关键更正（我自己的前一条结论是错的）**：直接执行 `bin/intewell-agent-cli` 会报
`python3.10: error while loading shared libraries: libpython3.10.so.1.0` —— 这**不是** CLI 损坏。
该 `so` 就在 `/opt/IntewellAgent/agentbase/lib/libpython3.10.so.1.0`，常驻实例的 env 里带着
`LD_LIBRARY_PATH=…:/opt/IntewellAgent/agentbase/lib:`。补上该变量后 CLI 正常：
```
LD_LIBRARY_PATH=/opt/IntewellAgent/agentbase/lib /opt/IntewellAgent/agentbase/bin/intewell-agent-cli --help
→ 正常打印子命令表
```
⇒ `agentos_bridge` 在 BoardProfile 里因此记为 `available`（有实测可用证据），而不是 `unverified`。

### 4.1 AgentOS CLI 的能力面（决定了 IRAF 该怎么接进去）

`intewell-agent-cli` 的子命令表实测包含：`mcp / tools / skills / plugins / bundles / host-worker /
gateway / webhook / cron / sessions / memory / curator / dashboard / serve / acp / voice / config / doctor …`
（`--help` 全文见 raw/agentos_cli.txt 的尾段）。

**由此得到 IRAF 接入的两条合法路径（按代价排序）**
```
① MCP（首选，官方通道、配置驱动、不碰 AgentOS 代码）
   CLI 自带：`intewell mcp add <name> --command <cmd> --args <args…>`（或 --url <endpoint>）
   现状：`mcp list` = "No MCP servers configured." ⇒ 我们注册即可
   内置 MCP 目录：/opt/IntewellAgent/agentbase/optional-mcps/{linear,n8n}/manifest.yaml
   ⇒ 做法：把 IRAF 暴露成 MCP server（工具 = 列技能 / 提交意图 / 查执行），
           MCP server 内部只调 IRAF 的公开入口（iraf-agentos --json 或 HTTP/gRPC），
           **不**让 AgentOS 触到 ROS topic/设备凭据（铁律 1.1 / 6.9）
② skills（CLI 支持 --skills / skills 子命令）—— 适合把"操作 IRAF 的说明书"固化给 AgentOS 侧
```

## 5. 与部署直接相关的事实

```
IRAF 现状     无任何 IRAF（raw/iraf_presence.txt：no-iraf-console-scripts / no-iraf-units）⇒ 全新安装
端口          已占：22 5432 8080 8081 8082 8083 8090 8888 9119
              **空闲**：8765（IRAF http）、50051（IRAF grpc）⇒ 形态 B 的 edge-intel 无端口冲突
python 口径   系统 3.11.6（cp311）；AgentOS 自带 py310 仅是它自己的运行时
pip 源        /etc/pip.conf → https://pypi.tuna.tsinghua.edu.cn/simple/（说明本机曾配置过在线源；
              实际出网连通性本轮未取到证据 —— network 探针只拿到 curl 存在，输出为空，需补测）
```
**网络结论**：本轮的 `network` 探针**没有拿到**外网连通性证据（输出只有 `curl`/`python3` 两行）。
因此形态 B 的依赖来源仍按声明走 **wheelhouse（离线）**，不得凭"配了清华源"就当在线可达。

## 6. 签名清单缺失（铁律 6.10 的硬缺口）

```
manifests 探针（find /etc /opt /usr/share /var/lib -maxdepth 4，-iname packaging_config*/HyperProfile/
release*manifest*/ariadne*/iaros*）⇒ 只命中 /etc/iaros-version
```
⇒ 正式构型名、VM 数量/编号/IP/OS/**角色映射**在本机取不到。**需要**：
① 研发/交付提供签名 HyperProfile 或 Release manifest（最好在 Hyper 宿主侧或发布包内）；
② 在此之前 `hyper_models` 保持 `unverified`，安装只能以 `--allow-unverified` 探索，**不得**当作验收证据。

## 7. 平台健康（raw/health.txt）—— 部署前必须记录

```
load average 6.32 / 6.18 / 6.19（4 核）   而 top 的 %Cpu(s) = 97.6 id ⇒ 负载不是 CPU 造成
D 状态（不可中断 I/O，父进程 kthreadd）：vpu-read-msg0 / vpu-dispatch0 / vpu-codec0-1..4（已 2 天 6:40:58）
僵尸 31 个：全部是 wget，父进程 = `searxng`（pid 84949）⇒ AgentOS 侧搜索组件不回收子进程
```
影响：ssh 认证与 `docker ps` 曾各被拖超时一次（采集脚本因此加了 per-probe 超时 + 认证重试）。
**判读纪律**：后续在板卡上出现的"安装超时/命令挂住"，先查这份背景，再怀疑 IRAF。

## 8. 下一步（按依赖排序）

```
① 拿到签名 HyperProfile/Release manifest → 回填 hyper_models 与角色映射（铁律 6.10）
② 确认 10.192.1.2 是不是运控域；若是，取到该域的访问方式（形态 B 的 edge-motion 要装在那里）
③ 补测板卡出网连通性（决定 edge-* 用在线索引还是离线 wheelhouse）
④ 实现按声明的安装/卸载/残留核验（config/install/install_profiles.yaml 已就位，脚本待写）
⑤ 装完立即验证：APP 侧 `iraf-agentos --list-skills`；跨域 `verify_domain_split.sh` 五例
⑥ 把 IRAF 以 MCP server 形态注册进 AgentOS CLI（`intewell mcp add …`），再由 CLI 侧唤起
```
