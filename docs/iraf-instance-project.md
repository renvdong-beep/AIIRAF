# IRAF 实例工程（AgentOS 机器人开发工作目录）

> 适用版本：v0.4.1。本文定义「IRAF 实例工程」的标准结构与 SDK 的分界，回答「板卡装好 IRAF 后，AgentOS 进入哪个目录做机器人开发」。
> 全程 `simulation=true`。

---

## 1. 概念：SDK 与实例工程分层

IRAF 交付物分成**不可混**的两层：

| 层 | 位置 | 内容 | 谁改 |
|---|---|---|---|
| **SDK（运行时）** | `/opt/iraf-sdk/current` | 运行时（iraf_core/iraf_adapters/iraf_skills/iraf_tools）、`iraf-mcp-http.service`、5 个 CLI、skills、profiles、config、deploy | 装机固化，版本化，可卸载；**实例不改** |
| **实例工程（开发目录）** | 如 `/root/iraf-instance` | 场景构建、标定、装配单切换、任务编排、本实例产物 | 开发者 / AgentOS 进入此目录开发 |

**原则**：实例工程**只依赖** SDK 的运行时与 MCP，不复制 SDK 源码。板卡装好 IRAF 后，AgentOS 进入实例工程做机器人开发，SDK 保持只读。

## 2. 实例工程标准结构（模板）

```
iraf-instance/                    # 实例工程根
├── README.md                     # 本实例说明：什么场景、怎么构建/标定/跑任务
├── scenes/                       # 场景构建（MJCF 生成 + scene_check 校验）
│   ├── build_2box.py
│   └── build_2box_034.py
├── config/                       # 场景/装配单/部署取值（instance 级，覆盖 SDK 默认）
│   └── ...
├── scripts/                      # 标定/切换/运维脚本
│   ├── calibrate_box02_column.py
│   └── deploy_2box.sh
├── tasks/                        # 任务编排（AgentOS 进入后下发的任务清单/场景入口）
└── artifacts/                    # 本实例产物（场景 XML、标定 JSON、日志、报告）
```

命名约定：`README.md` + `scenes/` 必留；`config/`、`scripts/`、`tasks/`、`artifacts/` 按需。

## 3. 与 SDK 的分界（哪些放 SDK、哪些放实例）

| 内容 | 放哪 | 说明 |
|---|---|---|
| 运行时、服务、CLI、skills、profiles、safety | SDK | 装机固化，版本化 |
| 构建/部署脚本（build_sdk、board_deploy、install_shape、install_profile_check） | SDK | 属交付/运维，不在实例目录 |
| 构建产物（.whl、.tar.gz）、离线依赖（pylib、wheelhouse） | SDK 构建侧 | 不留在实例目录 |
| 场景构建脚本（build_*.py）、标定脚本（calibrate_*.py）、装配单切换（deploy_*.sh） | **实例** | 场景/实验，属开发 |
| 场景 XML、标定 JSON、日志、报告 | **实例** artifacts | 本实例产物 |

## 4. AgentOS 进入实例工程的 workflow

1. **板卡装好 IRAF**（`install_shape.sh --shape intel/motion-* --apply`，见 SDK 安装说明）。
2. **AgentOS 绑定 MCP**（`board_agentos_mcp_bind.sh --apply --verify`，见使用说明 §4）。
3. **进入实例工程开发**：开发者在 `/root/iraf-instance` 里写场景构建/标定脚本，跑 `scene_check`/`scenario` 验收。
4. **AgentOS 下发任务**：AgentOS chat 新开会话 → 调用 `iraf_enter_robot_mode` → `iraf_submit_task`（自然语言）→ `iraf_get_task` 回查。实例工程的场景由开发者构建，AgentOS 只做任务级语义下发（不直接改场景/技能）。
5. **实例归档**：本实例的场景 + 标定 + 报告打包归档，可复跑（ReplayManifest 冻结场景+模型+Profile+输入摘要）。

## 5. 从旧目录抽离（147 板 `/root/iraf-deploy` 为例）

`/root/iraf-deploy` 是「SDK 构建/部署 + 实例开发」混在一起的历史目录，抽离规则：

| 归 SDK 侧（不留在实例） | 归实例工程侧 |
|---|---|
| `artifacts/`、`pylib/`、`wheelhouse/`、`pyproject.toml`、`src/`、`tests/`、`scripts/install_profile_check.py`、`config/install/`、`deploy/sdk/` | `build_2box*.py` → `scenes/`；`calibrate_*.py`、`deploy_2box.sh` → `scripts/`；场景配置 → `config/`；场景 XML/标定 JSON → `artifacts/` |

抽离后，板卡上只留 `/opt/iraf-sdk`（SDK）+ `/root/iraf-instance`（实例工程）两层。
