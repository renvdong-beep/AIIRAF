# Provider 成功返回后不释放执行域租约 ⇒「复位 → 立刻抓」被判 IRAF-RESOURCE-BUSY（2026-10-26）

## 1. 现象与复现（本机 x86，进程内走完整 `servicer.Invoke`，不需要端口/MuJoCo）

```
PYTHONPATH=src:build/generated/python build/mcp-venv/bin/python scripts/probe_provider_lease_release.py
```

修复前：

```
[1] reset_scene   accepted=True code=- token=1
    authority 残留租约: [('robot-arm', 'remote-runtime:exec-e4eaf054ad4d41a89ba2d8ee98605188', 1)]
    registry._leases: 1 条
[2] pick_object   accepted=False code=IRAF-RESOURCE-BUSY token=0 msg=resource busy: robot-arm
    authority 残留租约(owner, token, 剩余秒): [('robot-arm', ..., 1, 120.0)]
    backend 实际执行次数: 1 [('reset_scene', 'all')]
```

修复后（同一探针）：

```
[1] reset_scene   accepted=True code=- token=1
    authority 残留租约: []
    registry._leases: 0 条
[2] pick_object   accepted=True code=- token=2
    authority 残留租约(owner, token, 剩余秒): []
    backend 实际执行次数: 2 [('reset_scene', 'all'), ('pick_object', 'cube')]
```

结论：`IRAF-RESOURCE-BUSY` 来自 **x86 Provider 服务端自己的** `ControlAuthorityManager`，
不是板卡侧。板卡侧 `iraf_core/runtime.py:160-172` 在 finally 里按执行释放，实测正常。

## 2. 根因（`src/iraf_adapters/remote/provider_service.py`）

1. `ExecutionRegistry.resolve()` 为每个 `execution_id` 在本地 authority 上取独占租约：
   `_leases[execution_id] = lease`，TTL 由 `ttl_seconds` 决定（默认 120 s）。
2. `CapabilityProviderServicer.Invoke()` **只在 `except Exception` 里调 `release`**，
   成功路径直接 `return result` ⇒ 租约留在 `_leases` 与 authority 里，直到 TTL 到期。
3. 客户端 `RemoteProviderBackend._invoke()` 每次调用都换 `correlation_id`/`idempotency_key`
   且**不传 `execution_id`**（服务端生成）⇒ 每次动作都是一个新 execution、都要重新 acquire
   ⇒ 上一个动作没释放就是硬冲突：`LeaseConflict: resource busy: robot-arm`。
4. 同类泄漏还有一处：`CapabilityRejected` 若在**取到租约之后**抛出（能力未实现、
   `_POSITIONAL` 未登记参数形状），原代码只 `mark(REJECTED)` 不释放 ⇒ 同样卡 120 s。

## 3. 为什么"成功即释放"不放松安全边界

- 板卡侧租约就是**执行域**（每次 skill 执行 acquire、finally release，TTL = skill timeout），
  `docs/iraf-idl-runtime-detailed-design.md` §「两级租约」也把 `piper.arm` 这类列为
  execution-scope；Provider 侧按同粒度释放，两侧语义一致。
- 跨调用保留租约换不到"同一控制源"保证：一个 execution 就是一次动作，客户端从不复用
  `execution_id`。
- 仍保留的护栏：fencing token 单调水位（`_last_token`）、终态执行的再发控制请求拒绝、
  安全隔离锁存、每次 acquire 生成新 token（实测第二次调用 token=2 > 第一次 1）。

## 4. 改动

| 文件 | 改动 |
|---|---|
| `src/iraf_adapters/remote/provider_service.py` | `ExecutionRegistry.release()` 改为返回 bool、无租约时不写终态；`Invoke()` 三条路径（SUCCEEDED / REJECTED / FAILED）都释放执行域租约 |
| `tests/unit/test_provider_lease_release.py` | 新增 5 例：复位→立刻抓取 accepted、token 单调、拒绝后不残留、执行失败不残留、无租约 release 不写终态 |
| `scripts/probe_provider_lease_release.py` | 本缺陷的可复跑探针（退出码 0 = 第二次调用被接受） |
| `pyproject.toml` | 0.3.17 → 0.3.18 |

验收（本机）：

```
PYTHONPATH=src:build/generated/python build/mcp-venv/bin/python -m pytest \
  tests/unit/test_provider_lease_release.py tests/unit/test_remote_provider_service.py \
  tests/unit/test_remote_safety_propagation.py -q
27 passed
```

## 5. 观察项（本次不改，另行拍板）

1. `reset_scene` 不在 `factory.MOTION_CAPABILITIES` 内 ⇒ "终态执行再发控制请求"的拒绝对它
   不生效。它确实改物理状态，是否纳入属能力分类口径变更，会影响装配期反向核对报告。
2. 板卡侧 fencing token 在**板卡进程重启**后从 1 重来，而 Provider 的 `_last_token` 水位仍在
   ⇒ 可能出现 `IRAF-FENCING-TOKEN-REGRESSION`。与本次缺陷无关，先记录。
