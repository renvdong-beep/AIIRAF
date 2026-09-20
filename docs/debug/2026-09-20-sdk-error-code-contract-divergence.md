# 2026-09-20 步骤 05：错误码「IDL 声明 ↔ 实现抛出点」不一致（SDK 映射口径）

计划：`plans/iraf-24h/05-iraf_sdk 最小层与中文诊断.md`
关联：AGENTS.md 铁律 1.3（安全上限只来自已签名声明）、2.3（不允许吞掉错误/默认成功）、
2.4（状态与异步可追溯）、5.4（先读调试记录再改）；`docs/iraf-idl-runtime-detailed-design.md` §5。

## 1. 症状

步骤 05 要求"把既有错误码映射为中文诊断"。但"既有错误码"有两个互不一致的来源：

1. `docs/iraf-idl-runtime-detailed-design.md` §5「标准错误码与测试」表格（**契约**）；
2. `src/iraf_core/*`、`src/iraf_adapters/*`、`scripts/*` 的实际抛出点（**实现**）。

若直接按 §5 表格写映射，SDK 会对真实高频失败（如 `IRAF-UNAUTHENTICATED`、
`IRAF-EXECUTION-FAILED`）返回「未知错误码」；若直接按实现写映射，又会漏掉契约已声明但
尚未落地的码（`IRAF-RESOURCE-BUSY`、`IRAF-SAFETY-STOP`），让调用方误以为现在就能触发它们。

## 2. 证据链（数字与命令）

三方比对脚本：`build/iraf-24h/05/error_code_divergence.py`（证据区，不提交），
原始输出 `build/iraf-24h/05/error-code-divergence.txt`：

```
IDL §5 错误码表条目数: 9
源码抛出点覆盖的错误码数（src/ 不含 iraf_sdk）: 15
含 scripts/ 的抛出点覆盖错误码数: 16
SDK 映射条目数: 19
```

| 集合 | 数量 | 错误码 |
|---|---|---|
| 两侧都有（IDL 声明 + 实现抛出） | 7 | INPUT-INVALID、POLICY-DENIED、PRECONDITION-FAILED、SKILL-PROVIDER-UNAVAILABLE、IDEMPOTENCY-CONFLICT、DEADLINE-EXCEEDED、CANCELLED |
| 仅 IDL 声明（无抛出点） | 2 | RESOURCE-BUSY、SAFETY-STOP |
| 仅实现抛出（§5 表格未收录） | 9 | UNAUTHENTICATED、EXECUTION-CONFLICT、EXECUTION-NOT-ACTIVE、EXECUTION-FAILED、SAFETY-QUARANTINED、CANCEL-STOP-FAILED、INTENT-PARSE-FAILED、INTERNAL、TRANSPORT-FAILED |
| 仅 API 契约目录声明 | 1 | STREAM-LAGGED（`docs/iraf-api-contract-catalog.md`） |

补充实测（同一轮，`build/iraf-24h/05/status.txt`、`grpc-message-probe.txt`）：

- `/usr/bin/python3` = 3.10.12；protobuf **4.25.7**，而 `build/generated/python` 的生成 stub
  需要 protobuf 5.x 的 `google.protobuf.runtime_version` ⇒ 本机**无法**导入生成的 IDL stub
  （`ImportError: cannot import name 'runtime_version'`），gRPC 消息层用例因此 SKIP 4 例；
- `/usr/bin/python3` **无 grpcio** ⇒ `SkillRuntimeServiceStub` 等 RPC 通道无法联调。

## 3. 根因

1. **§5 表格不是生成的**：错误码是散落在 `policy.py` / `runtime.py` / `runtime_http.py` /
   `bridge.py` 里的字符串字面量，表格靠人工维护，因此实现新增码时表格不会自动更新
   （9 个码已实现但未入表），表格里声明的 ResourceCoordinator 语义（`IRAF-RESOURCE-BUSY`）
   与安全中断语义（`IRAF-SAFETY-STOP`）实现也尚未落地。
2. **同一语义两个名字**：安全事件在实现里表达为 `IRAF-SAFETY-QUARANTINED` + 终态
   `SAFETY_STOP`，而 §5 表格写的是 `IRAF-SAFETY-STOP`（错误码）。调用方若只按表格匹配，
   会漏判安全中断。
3. **retryable 三处不一致**：§5 表格声明 2 个码可重试（RESOURCE-BUSY、
   SKILL-PROVIDER-UNAVAILABLE），但 `src/iraf_adapters/grpc/runtime_grpc.py:38` 对**所有**
   反馈统一写 `error.retryable = False`。

## 4. 处置（本步骤实际做的）

不改实现、不放宽任何门禁，把"不一致"显式化，并做成可回归的门禁：

1. SDK 映射**同时收录三方来源**，逐码用 `provenance` 标注
   （`idl+code` / `idl-only` / `api-catalog-only` / `code-only`），共 19 个码；
2. `idl-only` 的码在中文诊断里写明"实现尚未落地，不得据此假设当前可触发"；
3. `code-only` 的码在诊断里标注"IDL §5 表格未收录（契约缺口）"，供后续补表；
4. 重试判读以 §5 表格为准（`retry_policy_zh` 逐字以表格"是否重试"列开头），
   服务端若显式给出 `retryable` 则以服务端为准，并在诊断里打印「契约漂移提示」；
5. 两条回归门禁（`tests/unit/test_sdk_errors.py`）：
   - `test_every_idl_code_has_a_known_chinese_diagnosis` + `test_idl_retry_column_matches_declared_retry_policy`：
     §5 表格解析结果逐行与映射比对（表格增删码而映射未跟 ⇒ 失败）；
   - `test_every_code_emitted_in_source_is_registered`：扫描 `src/**` 的 `IRAF-` 字面量，
     出现未登记码 ⇒ 失败；`test_declared_emitters_exist_and_really_emit_the_code` 反向校验
     每条 `路径:行号` 真实存在且确实出现该码（防止声明凭印象）。

**未做的事（有意留白，避免扩大本步骤范围）**：不修改 `docs/iraf-idl-runtime-detailed-design.md`
§5 表格去补那 9 个 `code-only` 码，也不改 `runtime_grpc.py` 的 `retryable=False`——
两者都属框架线变更（本战役不触碰非本窗口路径）。**关闭条件**：把 9 个码补入 §5 表格，
并把 `IRAF-SAFETY-QUARANTINED` 与 `IRAF-SAFETY-STOP` 的命名统一（或明确二者分层），
届时 `provenance` 相应改为 `idl+code`，两条门禁仍应通过。

## 5. 复现命令

```bash
# 三方比对（需要 PYTHONPATH=src 才能 import iraf_sdk）
/usr/bin/python3 build/iraf-24h/05/error_code_divergence.py

# 门禁（任一不成立即失败）
PYTHONPATH=src /usr/bin/python3 -m unittest tests.unit.test_sdk_errors -v

# 环境差异（为何 gRPC 消息层用例 SKIP）
/usr/bin/python3 build/iraf-24h/05/env_probe.py
PYTHONPATH=src:build/generated/python /usr/bin/python3 build/iraf-24h/05/grpc_message_probe.py
```

## 6. 适用范围与限制

- 本文结论只覆盖**本机 x86_64 开发端**（解释器 `/usr/bin/python3` 3.10.12）。
- gRPC RPC 通道（Execute/GetExecution/Cancel/ListEvents/GetReplayManifest）在本轮
  **未验证**：缺 grpcio，且生成的 stub 需要 protobuf ≥5.29。SDK 侧只完成了
  惰性导入契约、参数校验与消息层用例（后者因环境 SKIP），不得表述为"gRPC 已联调通过"。
- HTTP 路径的协议与解析逻辑用**本机 loopback stub** 验证，属 stub 级证据，
  不能作为运行时健康或目标端/真机证据。
