# IRAF 项目治理、RAID 与评审记录

**状态**：待项目组签署  
**维护者**：项目经理  
**更新频率**：每个里程碑评审前及每周至少一次

## RACI

| 工作域 | R 负责 | A 最终负责 | C 会签 | I 知会 |
|---|---|---|---|---|
| 产品范围与验收 | 产品经理 | 产品负责人 | 架构/测试/用户代表 | 项目组 |
| IDL/Runtime/TaskFlow | Runtime 工程师 | 架构负责人 | AgentOS/安全/运控 | 项目经理 |
| Policy 与安全论证 | 安全工程师 | 安全负责人 | 架构/运控/硬件 | 产品负责人 |
| Piper/机器狗仿真 | 仿真/运控工程师 | 运控负责人 | 算法/测试 | 架构负责人 |
| E300/S600/FIREFLY | BSP 工程师 | BSP 负责人 | 运控/Release/安全 | 项目经理 |
| CI、制品与发布 | 平台工程师 | Release 负责人 | 安全/测试 | 项目组 |

真实姓名在项目启动会填写；未指定 A 的工作域不得进入实施。

## RAID 清单

| ID | 类型 | 项目 | 影响 | Owner | 截止/触发点 | 退出条件 | 状态 |
|---|---|---|---|---|---|---|---|
| R-01 | 风险 | Piper Humble/MuJoCo 及 ROS 桥兼容性 | 阻塞 M3 | 运控负责人 | M0 结束 | 锁定 commit 的模型/MoveIt/夹爪/桥 smoke 通过 | OPEN |
| D-01 | 依赖 | Go2 EDU 精确 SKU、固件与全量许可证 BOM | 阻塞真机 M3 | 产品+运控 | M0 结束 | ADR-0004 已选型；采购确认、锁定 commit、传感器/载荷证据 | DECIDED/PENDING-EVIDENCE |
| D-02 | 依赖 | AgentOS 身份与生命周期接口 | 阻塞 M2 | AgentOS 负责人 | M1 评审 | interceptor 与四服务契约通过 | OPEN |
| D-03 | 依赖 | 三个 AICICD Hyper 发布构型 | 阻塞 M5 | Release+Hypervisor | M0/M5 | ADR-0005 已建模；LPR/LPP/LR 逐项生成并回读证据 | DECIDED/PENDING-EVIDENCE |
| D-04 | 依赖 | S600 正式 board/Hyper release profile | 阻塞 S600 S2 | BSP+Release | M5 前 | board baseline、构型、assembler、组件来源与独立验证 | OPEN |
| D-03 | 依赖 | 主站受控目标/SafetyEvent | 阻塞 M5 | 通信主站负责人 | M4 结束 | IDL、看门狗、stop/ack HIL 通过 | OPEN |
| D-04 | 依赖 | 三块板精确 SKU/BSP | 阻塞各 S2 发布 | BSP 负责人 | M4 结束 | BoardProfile 与设备访问就绪 | OPEN |
| R-02 | 风险 | 仿真与真机偏差 | 降低交付可信度 | 测试负责人 | M3 起持续 | 故障注入、HIL、偏差报告 | OPEN |
| A-01 | 假设 | CentOS/openEuler 可运行锁定 OCI 无头仿真 | 影响 S1 | 平台负责人 | M0 | 三宿主 smoke 证据 | OPEN |
| R-03 | 风险 | ROS 2 Humble 于 2027-05 结束支持 | 中期维护与安全更新 | 架构负责人 | 2026-12 | 后继 LTS 迁移 ADR 与兼容 spike | OPEN |
| R-04 | 风险 | Go2 官方 MuJoCo 主要覆盖低层控制 | 导航/停靠成熟度被高估 | 四足负责人 | M3 开始前 | 低层、导航、停靠、传感器分层验收 | OPEN |

## 里程碑签署记录

| 里程碑 | 产品 | 架构 | 安全 | 运控/测试 | 项目经理 | 结论/日期 |
|---|---|---|---|---|---|---|
| M0 | 待签 | 待签 | 待签 | 待签 | 待签 | NOT REVIEWED |
| M1 | 待签 | 待签 | 待签 | 待签 | 待签 | NOT REVIEWED |
| M2 | 待签 | 待签 | 待签 | 待签 | 待签 | NOT REVIEWED |
| M3 | 待签 | 待签 | 待签 | 待签 | 待签 | NOT REVIEWED |
| M4 | 待签 | 待签 | 待签 | 待签 | 待签 | NOT REVIEWED |
| M5 | 待签 | 待签 | 待签 | 待签 | 待签 | NOT REVIEWED |

文档中的“有条件通过”仅表示方案内部评审结论；只有本表对应里程碑完成实名签署并关闭阻塞 RAID 项后，才构成正式项目放行。
