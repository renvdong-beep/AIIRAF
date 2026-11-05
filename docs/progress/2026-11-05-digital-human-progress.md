# 数字人进度同步（2026-11-05）

> 本文件由数字人（Robot Engineering Digital Human）写入，记录当前进度供 AIIRAF 团队对齐。simulation=true。

## 本会话完成

1. **三份使用文档**（`docs/iraf-sdk-install-guide.md` / `iraf-usage-guide.md` / `iraf-technical-design.md`）：覆盖两种安装场景（x86 单机 + x86 仿真/边缘智控运控分离）+ AgentOS/AI Studio 使用。
2. **实例工程模板**（`docs/iraf-instance-project.md`）：SDK 与实例工程分层 + AgentOS 进入实例的 workflow。
3. **runtime bundle 去 arch**：`iraf-runtime-{version}.tar.gz`（内容平台无关，x86/板卡同包，避免误导）。
4. **147 板抽离**：`/root/iraf-deploy` → `/root/iraf-instance`（实例工程：scenes/scripts/config/tasks/artifacts + README）。
5. **WeKnora 知识库**：「机器人应用框架开发知识库」（id=723855c3-afb7-45a4-a641-535a1fb7b6d7）已写入 10 篇文档（架构/安全铁律/能力登记/VLA推理/机器人接入/调试记录/SDK/MCP/Skill契约/部署拓扑）。

## 数字人累计进度

- 阶段 0 基础设施 + G1 接入 ✅（RL 步态 25.78m/0.85m/s + 整链 SkillRuntime）
- 阶段 1 MVP ✅（ssh_run / inspect-robot / add-robot preset xarm+so100 / Franka 整链）
- 阶段 2：piper 优化 ✅（随机抓取 80%→100%）、go2 ✅（确认 MPC 已取代 legacy CPG）、ur5 ⏳
- 阶段 3/4：新机器人铺开 + 硬化 ⏳

## 待办 / 已知缺口

- ur5 抓取/放置优化（阶段 2.4）
- 智能域侧远端 Provider 后端（deploy-topology 明列缺口）
- VLA/学习策略训练 + 南向 Infer 三接口（设计完整、代码空白）
- test_install_profiles.py 2 个失败（主线 0.4.1 遗留：测试临时最小副本缺 edge-linux-standalone 文件；全仓 install_profile_check.py 114 项全过）
