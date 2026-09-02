# IRAF 易用性与泛化实施方案

## 目标群体与成功定义

IRAF 面向四类用户：机器人应用开发者、算法/Skill 开发者、机器人集成商、部署运维人员。成功不是提供更多配置项，而是让各角色只处理自己负责的抽象，并在 30 分钟内完成一个可回放的仿真场景：创建工程、启动 Piper/机器狗场景、调用 Skill、看到明确状态和失败原因。

## 产品原则

1. **能力优先**：业务调用 `navigate`、`pick_object`、`place_object`，不调用厂商 topic 或设备节点。
2. **约定优于配置**：模板提供安全默认值；超时、版本、能力、日志位置和退出码有统一约定。
3. **渐进暴露复杂度**：默认只展示 TaskFlow/Skill/Profile；需要时才进入 Provider、ROS 2、模型和板级层。
4. **失败可理解**：每个错误同时提供机器可读 error code、中文诊断、相关 Profile/Provider 版本和下一步建议。
5. **可替换且可验证**：仿真器、本体、模型、GPU/NPU、板卡都通过 adapter/profile 替换；替换后用同一契约与场景测试验证。

## 最小用户旅程

```text
iraf init -> 选择 handoff-lab 模板 -> iraf dev up
         -> 修改 delivery_handoff.yaml -> iraf test scenario
         -> 观察 trace/replay -> iraf package -> profile check/deploy
```

`handoff-lab` 模板预置 Piper 眼在手、机器狗相机/雷达、托盘、物品和交接状态机。默认值低速、短工作空间、无真实硬件权限；切换到 `edge-prod` 必须显式指定经签名且已验收的 Robot/Board/Safety Profile。

## 交付物

| 能力 | 面向用户的产物 | 验收 |
|---|---|---|
| 工程初始化 | `iraf init` 与 `handoff-lab` 模板 | 新目录可直接通过 lint/contract test |
| 本地仿真 | `iraf dev up/down/logs` | 一条命令启动并健康检查所有组件 |
| Skill 开发 | `iraf skill scaffold`、manifest、Provider 模板 | 生成代码可编译并含失败路径测试 |
| 配置验证 | schema、中文帮助、`iraf profile check` | 缺能力/版本/策略问题在启动前报出 |
| 验收回放 | `iraf test scenario`、trace/replay 索引 | 成功和故障用例均可重放与定位 |
| 部署 | `iraf package/deploy`、SBOM、签名摘要 | 只接受已验收目标 Profile |

## 防止泛化失控

只有满足至少两个独立 Provider/本体需求时才上升到核心抽象；单个平台特殊项保留在 adapter。所有新抽象必须同时给出：公开合约、最小示例、替换理由、兼容策略和至少两个验证场景。不能证明复用价值的抽象不进入 Runtime。

## 度量与门禁

- 新应用从模板到首次仿真成功不超过 30 分钟，且不编辑 Runtime 源码。
- 新 Skill 从脚手架到契约测试通过不超过半天，失败路径覆盖率与成功路径同等要求。
- 新 Robot/Board 接入不修改 TaskFlow/Skill 公共合约；其 Profile/adapter 和 HIL 证据独立可审查。
- 所有 CLI 必须支持 `--help`、非零退出码、结构化输出和中文可操作错误；不得依赖未文档化环境变量。
