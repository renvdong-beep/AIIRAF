# A+B 实施计划：S1 命令式交互入口 + 场景实时观看（Go2 / 机械臂通用）

- 制定时间：2026-09-21
- 使用者选择：先做 A（S1 命令式交互）+ B（机器狗实时演示入口）
- 前置事实（已实测）：
  - Go2 声明齐备：`config/go2_loopback.yaml`（model.file=`build/scenes/handoff_lab/handoff_lab.xml`、
    backend=`unitree_go2_mujoco`、control.frequency_hz=100、kp=150/kd=4、重力前馈、力矩上限读模型）；
    产能声明 `profiles/unitree_go2_mujoco.yaml` 的 `capabilities: [stand, stop]`（`locomote` 不声明）。
  - `src/iraf_adapters/unitree/unitree_go2.py` 暴露 `self.model/self.data`，`stand()/stop()` 内部自带
    `mj_step` 控制循环（阻塞式）⇒ 实时观看需要一个**控制周期钩子**，而不是另起线程读同一份 `mjData`。
  - `src/iraf_adapters/mujoco/viewer_runner.py` 已有 `resolve_display_mode`（interactive_viewer /
    offscreen_frames / unavailable）、`ViewerSession`、`run_interactive_live`（臂侧实时播放的既有模式）。
  - `scripts/scenario.py` 已有 `assemble()`（Profile+Safety+Registry+Authority+Backend+SkillRuntime）
    与 `_request()`；S2 场景执行已验收（27 项）。

## 0. 目标与判据

- A：`scripts/scenario.py interact` —— 终端敲命令 → 只允许**已声明能力** → 走
  `TaskFlow → SkillRuntime → PolicyGateway → ControlAuthority → Provider → 适配器 → MuJoCo`；
  未声明能力/越界参数/未知命令一律显式拒绝并留痕；全程 `simulation=true`。
- B：`--window` 让"动作执行期间"实时看到 MuJoCo 画面（有 DISPLAY 走窗口，无 DISPLAY 自动降级为
  离屏帧并**如实标注**）。机械臂与机器狗共用同一条 viewer 桥。
- 不做：S3 遥操作/手柄；不新增任何未验收能力（`locomote` 仍只走拒绝路径）。

## 1. 步骤

| 步 | 内容 | 文件 | 验收 |
|---|---|---|---|
| A+B.1 | 四足契约加声明的 `on_step` 钩子（可选、默认 None，机器人无关） | `src/iraf_adapters/unitree/quadruped.py`、`src/iraf_adapters/unitree/unitree_go2.py` | 单测：不传钩子行为不变（逐位一致）；传钩子时每个控制周期被调用且收到步数/model/data |
| A+B.2 | 场景 viewer 桥（机器人无关）：窗口/离屏 + 每帧镜像 + 帧统计 | 新增 `src/iraf_adapters/mujoco/scene_viewer.py` | 离屏模式产出 ≥N 帧、非全黑判定、报告含 display_mode/frames；无 DISPLAY 时不报"窗口已开" |
| A+B.3 | `scenario.py interact`：REPL + `--commands-from`（含 stdin）；命令词表来自声明；transcript JSONL + 报告 | `scripts/scenario.py`（新增子命令，不动 list/run 语义） | 正向：Go2 上 `stand`/`stop`/`state` 成功；负向：`locomote`（未声明）、越界参数、未知命令、错误本体各自退出码非 0 且原因可定位 |
| A+B.4 | `scenario.py run/interact` 支持 `--window`（复用 A+B.2） | `scripts/scenario.py` | 离屏降级路径可复跑并有报告；窗口路径在有 DISPLAY 的机器上人工确认 |
| A+B.5 | 文档与图：把新入口登记进已实现入口清单（未实现项保持"尚未实现"） | `docs/iraf-engineering-design.md` §8.1.1、`docs/iraf-developer-experience.md` | 文档一致性门禁通过；图文一致 |
| A+B.6 | 集中验收脚本 + 冻结工作区运行 | 新增 `scripts/verify_interactive_demo.py`（或 `build/iraf-24h/22/run_acceptance.sh`） | 正/负路径全绿；全量单测失败集合不变 |

## 2. 纪律

1. 声明先于实现：命令词表、时长上限、速度上限、判据阈值一律来自 `config/scene.schema.json`、
   场景包与机型声明；脚本内不得出现阈值/路径默认值。
2. fail-closed：未声明能力、越界参数、缺声明、无显示设备（降级但显式标注）都要有确定行为。
3. 逐位一致：A+B.1 钩子改造后，Go2 的 stand/stop 数字必须与步骤 15/16 的既有证据逐位一致
   （高度均值 0.2801007638069918 m、停止末速 0.003964950131491022 m/s 等）。
4. 提交只 add 本步声明的路径；工作区冻结后再跑最终验收（验收脚本已含脏工作区门禁）。
5. 中文文档/注释/提交体；Conventional Commits。

## 3. 证据落盘

- `build/acceptance/interactive/<run>/`：transcript.jsonl、report.json、frames/（离屏时）
- `build/iraf-24h/22/`：本批次的逐步证据与验收输出
