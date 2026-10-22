# 板卡侧 provider token 的受控注入（0.3.12）

日期：2026-10-22　触发：0.3.11 装完 board_diag 三项红同因 —— `ValueError: token <redacted>`

## 现象与真因

```
board_diag: ✗ CLI 可运行（--list-skills）
  iraf_adapters/remote/provider_backend.py:56 → ValueError: token <redacted>
  iraf_adapters/bootstrap.py:46 → load_backend(...)
```

真因不是代码 bug，是**凭据没有受控来源**：
- 装配单按设计不写凭据（`deploy/sdk/env/edge-intel.env.tmpl` 注释明写"凭据绝不写入本文件"）；
- systemd 单元的 `EnvironmentFile=-/etc/iraf/edge-intel.env` 因此也拿不到 token；
- x86 侧同样是手输环境变量（本机没有 `/etc/iraf/provider.token`，`deploy/.env` 也不存在）
  ⇒ 重启 Provider 就得重新手输，两端没有共同事实来源。

结果：板上 runtime 一定 fail-closed（设计正确），但**永远起不来**，board_diag 永远红。

## 口径

- 凭据**不进 git、不进命令行参数值、不进装配单**；只以 600 文件 + systemd `EnvironmentFile`
  注入（systemd 自身以 0600 读取，进程环境不落盘）。
- 缺 token 仍必须 fail-closed（现有语义不变）。
- 两端（x86 服务端 / 板卡客户端）用**同一份** token 文件内容，部署时由同一来源给出。

## 改动

1. `src/iraf_adapters/remote/provider_backend.py`
   - `from_config`：token 优先级 = `os.environ[token_env]` → `cfg["token_file"]`（默认
     `IRAF_PROVIDER_TOKEN_FILE`，再默认 `<CONFIG_DIR>/provider.token` 不猜）里的 `KEY=VALUE`；
     两处都空 ⇒ 仍抛 `ValueError`（fail-closed 不变）。
   - 新增 `read_token_file(path)` 纯函数（解析 `IRAF_PROVIDER_TOKEN=...` 或裸值），可单测。
2. `deploy/sdk/units/iraf-runtime-grpc.service.tmpl`
   - 增 `EnvironmentFile=-@CONFIG_DIR@/provider.token`（`-` 前缀：文件不存在不致命，
     但 runtime 会因缺 token 显式失败）。
3. `deploy/sdk/install-iraf.sh`（远端）
   - 新增 `--provider-token-file <远端路径>`：拷到 `@CONFIG_DIR@/provider.token`，600，
     并登记进安装记录（卸载时随之删除，避免卸载后残留凭据）。
4. `deploy/sdk/board_deploy.sh`（开发机）
   - 新增 `--provider-token-file <本地文件>`：校验权限为 600 且非空 → scp 到远端
     `<remote-dir>/artifacts/provider.token`（暂存，600）→ 安装时由安装器落地到 CONFIG_DIR。
5. `deploy/sdk/board_diag.sh`
   - CLI 自检前：若存在 `@CONFIG_DIR@/provider.token` 则 `set -a; . 它; set +a`（只导出，不打印）。
6. 回归
   - `tests/unit/test_provider_token_source.py`：环境优先 / 文件兜底 / 都没有→ValueError /
     文件格式两种（KEY=VALUE 与裸值）/ 600 以外的文件也读但记 warning（不 fail）。
7. 版本 0.3.12，构建 → 上板 → `board_diag` 复验（目标：`CLI 可运行` 转 ✓）。

## 验收

```bash
PYTHONPATH=src python3 -m pytest tests/unit/test_provider_token_source.py -q
# 板上
bash deploy/sdk/board_diag.sh --profile edge-intel     # 期望 CLI 可运行 ✓、单元 active ✓
```
判红条件不变：缺 token 时服务端拒起、客户端拒装配。

## 实施中的修正（2026-10-22 同日，0.3.13）

第一版只落地 `IRAF_PROVIDER_TOKEN`（单键文件 provider.token），上板后单元立刻 exit 2：

```
iraf-runtime-grpc: error: IRAF_GRPC_TOKEN must be provided by deployment
```

即板上 runtime 需要**两个**凭据键：
- `IRAF_PROVIDER_TOKEN`：出站调 x86 Capability Provider（RemoteProviderBackend）
- `IRAF_GRPC_TOKEN`：本域 gRPC 服务端入站鉴权（grpc.server）

所以改成**多键受控文件** `<config_dir>/runtime.secrets`（600，每行 KEY=VALUE，白名单
`IRAF_PROVIDER_TOKEN / IRAF_GRPC_TOKEN / IRAF_AGENTOS_TOKEN`），unit 用
`EnvironmentFile=-@CONFIG_DIR@/runtime.secrets`，CLI/探针在 diag 里 source 同一份。
参数名 `--secrets-file`。单键写法仍兼容（一行 KEY=VALUE 即可）。

口径：**simulation=true**，不构成真机能力证据。
