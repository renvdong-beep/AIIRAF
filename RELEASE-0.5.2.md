# IRAF 0.5.2 — x86 Provider + Board SDK

Date: 2026-11-24

## 版本

0.5.2

## 主要变更

- piper provider 增加 `pick_random`，在 provider 进程内建立随机场景并执行物理抓取；
- joint provider 修复 `reset_scene` 的 `_arm` 代码缺口，复位改回共享 plant；
- Go2 单机 provider 增加 `stand/locomote` 的运行时清单；
- Go2 场景支持前后左右按距离行走（`walk_distance`）；
- 三场景注册表：`piper-pick` / `go2-walk` / `joint-handoff`；
- provider supervisor：单窗口切换、全屏、关闭窗口不自动重启、scenario 期间不重启静态 Provider；
- 联合世界 9 步演示（`iraf_run_scenario` / `iraf_get_run`）；
- AgentOS 机器人模式技能清单对齐当前三场景。

## 产物

```text
releases/iraf-x86-provider-0.5.2-go2-joint-20261124.tgz
sha256 0a0eb4d639fc5c2fa73075ba66fbd043d680f047cb558c929828ddf85351ab2f

releases/iraf-sdk-board-arm64-0.5.2.tgz
sha256 f48ed4159527e049cd9d1feea75eabd26dfe796def7139c6a6fb2ac8b2e2c4e0
```

## 验证

- piper `pick_random`：`bilateral_contact=true`，`lifted=true`；
- joint `reset_scene`：`baseline=keyframe:0`，`status=SUCCEEDED`；
- Go2 `walk_distance`：1m → `duration_ms=5000`，`simulation=true`；
- 联合 9 步：`passed=true`，`steps` 全部 `SUCCEEDED`。

## 说明

全部为 MuJoCo 仿真，`simulation=true`，不是真机。
