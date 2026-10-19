# 计划：把"三档安装"固化进 IRAF 安装包（2026-10-19）

## 背景（这次手工装机暴露的缺口）

在 10.203.247.130（E300 / openEuler 24.03 / aarch64 / py3.11.6）上手工走通了两角色同机部署。
过程中有 6 处必须靠人肉记忆或临时改动才能完成，属于"下次还要重新踩"的东西：

| # | 缺口 | 本次实测证据 |
|---|---|---|
| G1 | env 模板里 JSON 值未加外层单引号 ⇒ shell source / systemd 解析后 JSON 非法 | `JSONDecodeError: Expecting property name enclosed in double quotes`；修后 `--list-skills` 列出 13 技能 |
| G2 | 两角色共用 `/opt/iraf-sdk` 会互相覆盖安装记录（file_record 不含角色 id） | `lib_install.py:181/207`；本次已给 edge-motion 做 `/opt/iraf-sdk-motion` |
| G3 | 单元模板**没有** `CPUQuota/MemoryMax` ⇒ AGENTS 6.11「独立资源限额」未落地 | `grep -l CPUQuota deploy/sdk/units/*.tmpl` = 无命中 |
| G4 | x86 仿真角色**没有**声明仿真依赖（mujoco/numpy/pillow/glfw），defaults 里也没有 | `install_profiles.yaml:46` 注释明确"仿真专用不在边缘角色里"，但单机/仿真角色也没补 |
| G5 | 矩阵有 2 个构建目标后，`package_board_bundle.sh` 必须手工 `--target`，否则直接拒绝 | 实测 `矩阵声明了 2 个构建目标，必须用 --target 显式指定` |
| G6 | 部署期 4 个取值靠 4 次 `--env-set` 手敲，容易漏/写错 | 本次命令：`--env-set PROFILE=… SAFETY_POLICY=… PROVIDER_ENDPOINT=… RUNTIME_SUBJECT=…` |

目标：三档安装各自一条命令可复现，且"选错档位"要 fail-closed。

```
S1  x86 全装      single-ubuntu2204（仿真 + 运行同机，仅 x86_64）
S2  aarch64 智控域 edge-intel（板上只跑 TaskFlow/Skill/Policy，能力在 x86 仿真 Provider）
S3  aarch64 两域   edge-intel + edge-motion（分机 → 各自默认路径；同机 → 路径隔离）
```

## 批次拆分与判据

### 批次 1（本轮做）：声明层补全 + target 推断
- G4：给 `single-ubuntu2204` 与 `sim-x86` 加**角色级** `runtime_dependencies`
  （mujoco / numpy / pillow / glfw，按 `src/iraf_adapters/mujoco/*` 的实际 import 定），
  并加一条负向用例：仿真角色缺 mujoco 必须被校验器拦下。
- G5：`package_board_bundle.sh` / `install.sh` 的 `--target` 改为**可省略**：
  从 BoardProfile 的 `python_tag` + `platform_tag` 反查矩阵唯一目标；
  命中 0 个或多个时仍 fail-closed 并打印候选。
- 判据：`install_profile_check` 90+ 项 0 未通过；`pytest tests/unit/test_install_profiles.py` 全绿；
  新增用例覆盖"推断唯一/推断歧义/推断无命中"三条路径。

### 批次 2：AGENTS 6.11 资源限额落地（G3）
- schema `definitions.service` 增 `cpu_quota` / `memory_max`（pattern：`150%` / `512M`）。
- 两个 edge 单元模板加 `CPUQuota=@CPU_QUOTA@`、`MemoryMax=@MEMORY_MAX@`；渲染器对**空值必须报错**
  （不得留空行给 systemd 静默忽略）。
- 声明里给 edge-intel（150% / 1G）与 edge-motion（100% / 512M）赋值；x86 单机角色允许缺省。
- 判据：渲染出的单元 `systemd-analyze verify` 通过；负向用例：less 于最小限额必须被拒。

### 批次 3：部署期取值模板化（G6）
- 新增 `config/install/deploy_values/<board>-<role>.yaml`（每档一个），字段与 env 模板占位符一一对应。
- `board_deploy.sh` 增 `--values <file>`，与 `--env-set` 二选一；缺项仍显式失败。
- 判据：三档各自"一条命令"完成 dry-run；缺字段时报出**字段名 + 模板行号**。

### 批次 4：三档一键入口 + 文档
- `scripts/install_shape.sh --shape {x86-all,intel,motion-split,motion-cohost}`：
  只做参数组装（build/package/deploy 三步串起来），不复制任何路径/端口事实（全部读声明）。
- 文档：`docs/install-shapes.md`（三档的适用场景、命令、验收判据、失败退码）。
- 判据：三档 dry-run 全绿；文档里的每条命令都保证真跑过（AGENTS 6.4：未实现的不得写成可用）。

## 进度（2026-10-19）

| 批次 | 内容 | 提交 | 状态 |
|---|---|---|---|
| 1 | 仿真依赖声明 + `--target` 按板卡声明推断 | 227e69d、7e4d563 | 完成 |
| 2 | 单元资源限额 CPUQuota/MemoryMax（6.11） | a7d9700 | 完成 |
| 3 | `--values` 取值文件 + 缺值定位到模板行号 | 86b1dfe | 完成 |
| 4 | `install_shape.sh` 四档入口 + `docs/install-shapes.md` | （见 git log） | 完成 |

仍未验证（见 docs/install-shapes.md 的"已验证/未验证"表）：
- `install_shape.sh --apply` 未跑过（真装走的是 board_deploy.sh 手工路径）；
- `x86-all` 档未在本机跑（会与现有开发 venv/systemd --user 冲突，须单独安排）；
- `motion-split` 未跑（没有第二块可达的运控域机器）。

## 硬约束（不改）
1. 仿真只装 x86（AGENTS §0.1）：任何 aarch64 角色 `<forbidden_components: [adapters_sim]>` 保持。
2. 两域同机不得合并安全边界（6.11）：独立单元 + 独立路径（G2 已做）+ 独立限额（批次 2）。
3. 未实测即 `unverified`；`--allow-unverified` 只能用于探索，不得当验收证据。
4. 所有路径/端口/单元只在声明里出现一次；脚本不得再写一份（5.3）。
