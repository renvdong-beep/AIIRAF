# Skill：实时性能测试（engineering.rt_latency_test）

> 用途：测 **RTOS/运控域** 的实时延迟（cyclictest），输出每核 min/avg/max 与报告。
> 2026-11-11 新增；诊断类能力，**不属于运动链路**。

## 1. 架构约束（为什么不在 RTOS 里装 IRAF）

- RTOS 内**不安装、不运行任何 IRAF 进程**——任何常驻进程都会影响实时性；
- 由**边缘侧** Provider 通过 SSH 在 RTOS 上执行**固定命令**，只回收文本、解析、出报告；
- 命令模板固定、参数有界，不接受任意 shell；目标地址/账号来自配置，代码不写死 IP。

```
边缘侧（IRAF Runtime / MCP） ── ssh ──► RTOS/运控域
   解析 + 报告                          cyclictest（系统自带能力，非 IRAF）
```

## 2. 执行命令

```
cyclictest --mlockall --smp --priority=<P> --interval=<I> --distance=0 -n -h <H> -q -D <D>s
```

兼容性（实测 rt-tests 2.20）：不支持 `-n`；`--distance=0` 与 `-h` 同时出现会落到 usage。
引擎按 `( -n, --distance )` 四种组合**逐级降级**，报告里记录实际使用的组合（`fallback` 字段）。

## 3. 配置（装配单 `/etc/iraf/edge-intel.env` 或环境变量）

| 变量 | 含义 | 默认 |
|---|---|---|
| `IRAF_RT_TARGET_MODE` | `ssh` / `local`（local 仅调试） | `ssh` |
| `IRAF_RT_TARGET_HOST` | RTOS IP/主机名 | 空（未配置则显式失败） |
| `IRAF_RT_TARGET_USER` | SSH 用户 | `root` |
| `IRAF_RT_TARGET_PORT` | SSH 端口 | `22` |
| `IRAF_RT_TARGET_IDENTITY` | 私钥路径（可选） | 空 |
| `IRAF_RT_TARGET_KNOWN_HOSTS` | known_hosts 路径（可选） | 空 |
| `IRAF_RT_TARGET_SUDO` | 目标端用 `sudo -n` 执行（非 root 时） | `false` |
| `IRAF_RT_MAX_US` | 判定阈值（µs）；不设=只测量不判定 | 不设 |
| `IRAF_RT_REPORT_DIR` | 报告目录 | `build/acceptance/rt` |

## 4. 用法

### 4.1 chat / MCP

```
iraf_rt_latency(duration_s=10, interval_us=200, priority=240, histogram_us=100)
```

返回：`per_core[]`（cpu/min/avg/max）、`summary`（核数/最大延迟/最差核）、`report`（json+md 路径）；
失败显式返回 `error_code`：`TARGET-UNCONFIGURED` / `SSH-FAILED` / `CYCLICTEST-MISSING` /
`PERMISSION` / `TIMEOUT` / `PARSE-FAILED`。

### 4.2 走 Skill 链路（runtime + engineering_tooling profile）

```bash
python3 scripts/run_skill.py --profile profiles/engineering_tooling.yaml \
  --skill engineering.rt_latency_test \
  --input '{"duration_s":10,"interval_us":200,"priority":240,"histogram_us":100}'
```

能力名 `engineering.rt_latency_test` 与运动能力隔离；不申请运动租约、不产生物理动作。

## 5. 报告

`IRAF_RT_REPORT_DIR` 下生成同名 JSON + Markdown：

```markdown
# 实时性能测试报告（cyclictest）
- 目标：`<RTOS IP>`   - 命令：`cyclictest ...`
| CPU | min (µs) | avg (µs) | max (µs) |
| 0 | 2 | 2 | 66 |
...
汇总：核数 4，最大延迟 113 µs（core 1）
```

## 6. 本地引擎自检（x86，非 RTOS，仅验证解析/报告）

`IRAF_RT_TARGET_MODE=local IRAF_RT_TARGET_SUDO=1` 在 `.121` 上跑 5s：

```
cores=4  max_us: 66 / 113 / 84 / 34   min=2  avg=2
fallback: 去掉了 -n（目标 rt-tests 不支持）；去掉了 --distance=0（与直方图冲突）
```

> 这组数字是 **x86 通用内核**，不是 RTOS 实时性能；RTOS 的结果必须在其本机（经 SSH）测得。

## 7. 失败与安全

| 失败 | 含义 | 处置 |
|---|---|---|
| `TARGET-UNCONFIGURED` | 没配 RTOS 地址 | 设 `IRAF_RT_TARGET_HOST` |
| `SSH-FAILED` | 不可达/凭据错 | 检查 IP、用户、密钥、known_hosts |
| `CYCLICTEST-MISSING` | RTOS 没装 rt-tests | RTOS 上 `dnf install -y rt-tests` |
| `PERMISSION` | 不能设实时优先级 | root 或 `IRAF_RT_TARGET_SUDO=1` |
| `TIMEOUT` | 命令超时 | 检查 RTOS 负载/时长 |
| `PARSE-FAILED` | 输出格式不识别 | 附原始输出（报告 `raw`）排查 |

铁律遵循：不做任何运动、不触碰安全策略、失败不包装成功、结果带 `kind=diagnostic`。