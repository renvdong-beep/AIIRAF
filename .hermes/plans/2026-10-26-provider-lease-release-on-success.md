# Provider 侧租约在成功返回后未释放（复位→立刻抓 卡 IRAF-RESOURCE-BUSY）

日期：2026-10-26
范围：x86 仿真侧 `src/iraf_adapters/remote/provider_service.py`（+ 回归测试 + 调试记录）
不动：板卡侧 `iraf_core/runtime.py`（每步 finally 释放，实测正常）

## 1. 现象（已复现，进程内走完整 servicer.Invoke）

`scripts/probe_provider_lease_release.py`（build/mcp-venv/bin/python）：

```
[1] reset_scene   accepted=True code=- token=1
    authority 残留租约: [('robot-arm', 'remote-runtime:exec-e4eaf054ad4d41a89ba2d8ee98605188', 1)]
    registry._leases: 1 条
[2] pick_object   accepted=False code=IRAF-RESOURCE-BUSY token=0 msg=resource busy: robot-arm
    authority 残留租约(owner, token, 剩余秒): [('robot-arm', ..., 1, 120.0)]
    backend 实际执行次数: 1 [('reset_scene', 'all')]
```

## 2. 根因（代码位置，一句话一条）

1. `provider_service.ExecutionRegistry.resolve()` 为每个 `execution_id` 在**本地**
   `ControlAuthorityManager` 上取独占租约（`_leases[execution_id] = lease`）。
2. `Invoke()` 成功路径**从不调用** `self._registry.release(...)`；只有异常路径
   （`except Exception`）才 `release(execution_id, "FAILED")`。
3. 于是 `reset_scene` 成功后，`robot-arm` 仍被 `remote-runtime:exec-…` 占着
   `ttl_seconds=120` 秒 ⇒ 紧随其后的 `pick_object` 在 `authority.acquire` 抛
   `LeaseConflict` ⇒ `CapabilityRejected(CODE_RESOURCE_BUSY)`。
4. 次要泄漏：`CapabilityRejected`（在 `_capability_call` 里抛，如参数形状未登记）
   发生时租约已取到，却只 `mark()` 不释放 ⇒ 同样卡 120 秒。

## 3. 为什么"成功即释放"是对的（不是放松安全）

- 板卡侧 `iraf_core/runtime.py:116/160-172` 的租约就是**执行域**（每次 skill 执行
  acquire、finally release，TTL = skill timeout），id 详情设计文档 §「两级租约」也把
  `piper.arm` 这类列为 execution-scope。Provider 侧按同一粒度释放，语义一致。
- 客户端 `RemoteProviderBackend._invoke` **每次调用都换 correlation/idempotency key，
  且不传 execution_id**（服务端生成）⇒ 一个 execution 就是一次动作；跨调用保留租约
  换不来任何"同一控制源"保证，只会挡住下一步。
- 仍保留的护栏：fencing token 单调水位（`_last_token`）、终态执行的再发控制请求拒绝、
  安全隔离锁存、每次 acquire 生成新 token —— 释放后下一次动作仍拿**更新**的 token。

## 4. 改动清单

1. `ExecutionRegistry.release(execution_id, state) -> bool`：没有租约时**不写终态**、
   返回 False（避免把"从未取到租约"的调用记成终态，也避免 `execution_id=""` 污染表）。
2. `Invoke()`：成功 / `CapabilityRejected` / `Exception` 三条路径都释放执行域租约，
   终态分别为 `SUCCEEDED` / `REJECTED` / `FAILED`。
3. 回归测试：
   - 注册表级：无租约时 release=False 且不产生终态；
   - servicer 级：`reset_scene` 成功后**同一 resource** 的 `pick_object` 必须 accepted
     且 `authority._leases` 为空；拒绝路径后同样不残留租约。
4. 调试记录 `docs/debug/2026-10-26-provider-lease-not-released-on-success.md`。
5. 版本 0.3.17 → 0.3.18（`provider_service.py` 在 SDK 清单内，板上也装这份）。

## 5. 验收

- `scripts/probe_provider_lease_release.py` 退出码 0、第二次调用 accepted=True；
- `pytest tests/unit/test_remote_provider_service.py tests/unit/test_remote_safety_propagation.py`
  + `tests/unit/test_reset_scene_skill.py` 全绿；
- 真实链路（provider serve + 板上 CLI「reset_scene → pick_object」）由用户侧跑，
  本机只证明服务端租约表在成功返回后为空。

## 6. 不做的（观察项，另行拍板）

- `reset_scene` 不在 `MOTION_CAPABILITIES` 内 ⇒ 终态再发请求的拒绝对它不生效。
  是否纳入属能力分类口径变更，牵扯装配期反向核对报告，单独评估。
- 板卡侧 fencing token 在板卡进程重启后会从 1 重来，而 Provider 的 `_last_token`
  水位仍在 ⇒ 可能出现 `IRAF-FENCING-TOKEN-REGRESSION`（本次不涉及，先记录）。
