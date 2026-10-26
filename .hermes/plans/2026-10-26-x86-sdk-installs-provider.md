# x86 端「装完 SDK 即有 Provider」：把仿真 Provider 固化进 sim-x86 安装角色

日期：2026-10-26　状态：待执行（本文件是施工单，未动代码）
起因：使用者口径「应该基于 IRAF 在 x86 端安装的 SDK 进行固化 Provider —— 当 x86 当 SDK 安装后，
已经具备 Provider 的条件」。现状是 Provider 只能由仓库脚本 + 会话前台拉起（pid 1354301 就是这么来的），
SDK 装完并不带服务。

## 0. 已核实的现状（证据，不是推测）

| 事实 | 证据 |
|---|---|
| `sim-x86` 角色 `services: []`，注释写"不起常驻服务" | config/install/install_profiles.yaml:155-157 |
| `sim-x86` 没有 `env_template` / `robot_profile` / `safety_policy` / `sim_scene` | 同上 132-167 |
| `sim-x86` 的 `runtime_dependencies` 缺 grpcio、protobuf | 同上 149-154（Provider 服务端要 grpc + proto 桩） |
| `sim-x86` 的 residue 只声明进程 scenario.py/view_mujoco.py，无单元无端口 | 同上 163-167 ⇒ 服务一旦装上也**卸载不干净** |
| runtime 交付包**不含** scripts/，因此 `scripts/iraf_provider_serve.py` 装不上 | `tar -tzf build/sdk/iraf-runtime-0.3.17-aarch64.tar.gz` 里只有 src/ + deploy/sdk/{units,env}，无 scripts/ |
| 已有可复用机制：`sim_scene` ⇒ 目标端拷场景并用报告生成 `IRAF_BACKEND_CONFIG` | deploy/sdk/lib_install.py:996-1029 |
| 已有可复用机制：`env_template` ⇒ 占位符渲染装配单（缺值显式失败） | lib_install.py:632-671 |
| 已有可复用机制：`--secrets-file` allowlist 含 `IRAF_PROVIDER_TOKEN`，落 600 `runtime.secrets` | lib_install.py:1046-1075 |
| 单元模板占位符白名单 `UNIT_TOKENS`（新增 token 必须登记） | lib_install.py:591-598 |

## 1. 目标（可验收的说法）

在 x86（Ubuntu 22.04）上执行一次 `install-iraf.sh --profile sim-x86 …` 后：
1. `<prefix>/<version>/models/` 有场景 XML + 旁挂报告，`<config_dir>/sim-x86.env` 里
   `IRAF_BACKEND_CONFIG` 由报告生成（不手写）；
2. systemd（user）里有 `iraf-provider.service`，`ExecStart` 指向**安装目录**里的入口；
3. `systemctl --user start iraf-provider` 后 `scripts/probe_provider_health.py` 返回 ready=true、
   capabilities 6 项含 visual_pick；
4. 「reset_scene → 立刻 move_joint」0.1 s 内两次都 accepted（本日已修的执行域租约释放）；
5. 安装记录里 `units=[iraf-provider.service]`、`ports=[50052]`；`uninstall-iraf.sh` +
   `verify_residue.sh` 后零残留（单元、端口、进程、配置、凭据全清）。

## 2. 设计决策（我建议的取值，理由随附）

D1 **入口改为模块入口**：把 `scripts/iraf_provider_serve.py` 的逻辑搬进
`src/iraf_adapters/remote/provider_cli.py`（`main()` 可 `python -m iraf_adapters.remote.provider_cli`），
`scripts/iraf_provider_serve.py` 保留为薄壳（dev 用法不变，含 `--self-check`）。
理由：交付包不含 scripts/（实测），把产品入口放在包里才装得上；也顺带让 CLI 可被单测覆盖。

D2 **场景来源**：`sim-x86` 声明 `sim_scene: build/models/piper-pick-scene.xml`。
安装器已有路径会拷 XML+JSON 到 `<version>/models/` 并在目标端跑 `emit_backend_config.py` 生成
`IRAF_BACKEND_CONFIG`（lib_install.py:996-1029）。x86 安装本来就是在仓库机上做（install_method=venv），
`build/models/` 一定存在；缺文件时安装器已显式失败（不猜）。

D3 **显示**：声明 `provider_display: window`（使用者口径"先看得见"），`DISPLAY` / `XAUTHORITY`
由**部署期值**给（`--env-set DISPLAY=:0 --env-set XAUTHORITY=/run/user/1000/gdm/Xauthority`），
模板里不写死；headless 场景 `--env-set PROVIDER_DISPLAY_MODE=headless`。
理由：铁律 5.3（机器相关值只来自声明/部署期），且 XAUTHORITY 路径每台机器不同。

D4 **端口**：`services[].ports=[50052]`，同时进 `residue.units/ports`。
`install_profile_check.py` 的"residue.units / ports 与服务清单一致"会自动把它钉住；
卸载依据安装记录，因此端口/单元必须声明在同一处。

D5 **依赖**：`sim-x86` 的 `runtime_dependencies` 补 `grpcio`、`protobuf`。
依据：Provider 服务端 `import grpc` + `from iraf.v1 import …`（桩已由 `python_stubs_dir` 注入）。

D6 **凭据**：`IRAF_PROVIDER_TOKEN` 走 `--secrets-file`（allowlist 已含），落到 `<config_dir>/runtime.secrets`（600）；
单元 `EnvironmentFile=-@CONFIG_DIR@/runtime.secrets`。缺 token ⇒ 服务 fail-closed 退出（现有 provider 逻辑），不静默。

D7 **保留 dev 入口**：`scripts/dev_provider_up.sh` 不删，降级为"从仓库源码跑（不走系统服务）"的开发入口；
文档里两者边界写清：**演示/产线用安装后的 systemd 单元，改代码验证用 dev 脚本**。

## 3. 改动清单（按提交粒度）

提交 1｜入口下沉（不改行为）
- 新增 `src/iraf_adapters/remote/provider_cli.py`：搬 `scripts/iraf_provider_serve.py` 全部逻辑（含 `--self-check`、
  显示模式解析、主线程 `run_live_mirror`、信号处理），并支持 `IRAF_PROVIDER_HOST/PORT` 环境变量作默认值。
- `scripts/iraf_provider_serve.py` 改为 3 行转发（`from iraf_adapters.remote.provider_cli import main`）。
- 回归：`tests/unit/test_provider_cli_contract.py`（`--help` 可跑、缺 token 退出码 2、`--self-check` 不监听端口）。

提交 2｜声明与模板
- `config/install/install_profiles.yaml` sim-x86：
  `robot_profile: profiles/piper_mujoco.yaml`、`safety_policy: profiles/safety/simulation_lab.yaml`、
  `sim_scene: build/models/piper-pick-scene.xml`、`env_template: deploy/sdk/env/sim-x86.env.tmpl`、
  `runtime_dependencies` += grpcio/protobuf、`services:[{name: iraf-provider, template: deploy/sdk/units/iraf-provider.service.tmpl, scope: user, ports: [50052], optional: false}]`、
  `residue.units: ["iraf-provider.service"]`、`residue.ports: [50052]`、`residue.processes` += `iraf_provider_cli`。
- 新增 `deploy/sdk/units/iraf-provider.service.tmpl`（照 grpc 单元的写法：User/@RUN_USER@、PYTHONPATH=@PREFIX@/@CURRENT_LINK@/src:lib、
  EnvironmentFile=@CONFIG_DIR@/@PROFILE@.env + runtime.secrets、`ExecStart=@PYTHON_BIN@ -m iraf_adapters.remote.provider_cli`、
  Restart=on-failure、独立限额占位、`ReadWritePaths=@LOG_DIR@ @DATA_DIR@`）。
- 新增 `deploy/sdk/env/sim-x86.env.tmpl`（PROFILE/SAFETY_POLICY/SKILL_ROOT/ENTRYPOINT/BACKEND_CONFIG/EVENT_STORE +
  `IRAF_PROVIDER_DISPLAY_MODE=@PROVIDER_DISPLAY_MODE@`、`DISPLAY=@PROVIDER_DISPLAY@`、`XAUTHORITY=@PROVIDER_XAUTHORITY@`）。
- `UNIT_TOKENS` 增补 `PROVIDER_DISPLAY_MODE / PROVIDER_DISPLAY / PROVIDER_XAUTHORITY`（未登记即渲染失败）。

提交 3｜校验与测试
- `scripts/install_profile_check.py`：新增两条机器可查规则
  （a）`simulation_only: true` 且组件含 `adapters_sim` 的角色**必须**声明 `iraf-provider` 服务的 50052 端口；
  （b）`sim-x86` / `single-ubuntu2204` 必须声明 `sim_scene`（否则装完没有场景，服务起不来）。
- `tests/unit/test_install_profiles.py`：负向用例（删掉 provider 服务 ⇒ 校验器报红）。
- `tests/unit/test_install_record_and_residue.py`：安装后记录含 `iraf-provider.service` / 50052；
  卸载后 unit、env、models、secrets 全清。

提交 4｜演练与文档
- `deploy/sdk/rehearse_install_layout.sh` 增加 sim-x86 场景（沙箱里装一次，断言单元文件与 env 生成）。
- 真机演练：`--profile sim-x86` 装到 `~/.local/opt/iraf-sdk`（与本仓库 build/mcp-venv 隔离），
  `systemctl --user start iraf-provider` → `probe_provider_health` → `reset→move` 探针 0.1 s；
  再 `uninstall-iraf.sh` + `verify_residue.sh` 出零残留证据（落 build/acceptance/）。
- 文档：`docs/iraf-sdk-install-shapes.md`（或对应安装文档）+ `docs/debug/2026-10-26-x86-sdk-provider-service.md`
  写清"服务 vs dev 脚本"的边界，并注明 `window` 模式下 provider 重启 = 场景复位（诚实语义）。

## 4. 风险与红线

- 本机安装会与当前会话后台的 provider（pid 1354301，端口 50052）冲突 ⇒ 演练前先 kill；
  演练用沙箱 root 时端口不冲突，但 systemd 单元不 enable（只渲染、只校验文本）。
- 装了 `adapters_sim` 的角色只能 x86_64（既有规则，install_profile_check:258-262 已钉死）。
- Provider 服务是**仿真域**：单元 Description 与 env 里都必须带 `simulation=true` 口径，不得表述为真机能力（铁律 1.7）。
- 版本 0.3.18 → 0.3.19（安装物变化）。

## 5. 估时

提交 1：40 min｜提交 2：40 min｜提交 3：40 min｜提交 4（含真机演练与卸载验证）：60 min。合计约 3 小时。
