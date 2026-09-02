# ADR-0005：AgentOS 北向契约与三 Hyper 构型适配

**状态**：已接受（接口/硬件事实待验证）  
**日期**：2026-08-28  
**决策人**：AgentOS、IRAF、Hypervisor、Release、运控负责人（待签名）

## 决策

IRAF 通过独立 `AgentOSBridge` 北向兼容 AgentOS，通过 `HyperProfile` 南向适配现有三个 AICICD 正式发布构型：

1. `MQ50-E300-3VM-LPR-Hyper-config.yaml`；
2. `MQ50-E300-3VM-LPP-Hyper-config.yaml`；
3. `FIREFLY-RK3588-2VM-LR-Hyper-config.yaml`。

IRAF 不假设固定 VM 数量、编号、IP、OS 组合或跨域 transport。运行时从签名 Profile 和发布制品版本清单解析部署角色；Task/Skill 公共语义在三个构型间一致，OS/VM/transport 差异停留在 Adapter 和部署层。

## 后果

- E300 LPR、LPP 分别验证 RTOS 主站和第二 Linux RT 主站；两者不能共用一份“已通过”报告。
- RK3588 2VM-LR 将 Motion 与 Master 合并到 Linux RT，但必须保持进程、权限、资源和设备隔离。
- Ubuntu Dev Runtime 只能模拟分域语义，不能代替 Hypervisor、实时和设备安全验收。
- S600 未进入当前三个正式 Release profile，继续保持 `unverified`。
- AgentOS 使用 current/previous 契约测试和显式版本协商；无兼容 major 时 fail closed。

## 验证

详细接口、部署矩阵、Profile 和故障测试见 `../iraf-agentos-hyper-compatibility.md`。M0 完成 AgentOSBridge/HyperProfile schema 与三套模拟 profile；M5 在对应板卡完成逐构型 HIL/硬件证据。
