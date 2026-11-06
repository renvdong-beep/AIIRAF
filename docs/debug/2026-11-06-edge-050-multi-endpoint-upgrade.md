# IRAF 0.5.0 多场景 MCP 端点：双机升级实录（2026-11-06）

范围：`.121` x86 仿真机（sim-x86）与 `.147` 边缘板（edge-intel）从 0.4.1 升到 0.5.0；
口径：一切结论 `simulation=true`，不构成真机/实时证据。

## 1. 交付物

```
build/sdk/iraf_sdk-0.5.0-py3-none-any.whl   sha256=4aedf401e9718465e92a0e8db1aeb498867b774f9f9b9cbfd5208be710b0f0b4
build/sdk/iraf-runtime-0.5.0.tar.gz         sha256=6eb72a5a91a74fd2653c27456e189e142edd86f9ee484f4ff7712a5208915313
```

0.4.1 保留在 git tag `v0.4.1`（指向 a667c5a），0.5.0 提交为 33a71ab。

## 2. 0.5.0 改了什么

- 同一角色两个 MCP 实例：`iraf-mcp-http`(8765, ur5) / `iraf-mcp-piper`(8766, piper)；
- `services[].env` 实例级装配单覆盖（`${DEPLOY_KEY}` 引用 `--env-set`，缺值即失败）；
- 单元模板端口参数化 `@MCP_PORT@`、装配单参数化 `@ENV_FILE@`；
- 基础装配单固定 `env_files[0]`，板侧脚本仍读 `/etc/iraf/<profile>.env`；
- AgentOS 绑定一次一个 server，验证记录按 server 分文件；
- `credential_keys` 声明必需凭据键，`--secrets-file` 缺键即拒绝安装；
- 新增 `deploy/sdk/verify_record.sh`（只读检查安装记录与 env/凭据的 sha256 漂移）。

## 3. 实测证据

### 3.1 .121

```
受控卸载 9125 项；六路残留：files 0 / dirs 0 / env_files 0 / ports 0 /
processes 0 / unlisted 0 / missing 0 → 干净
安装 9120 文件 + 3 配置 + 11 proto 桩；current -> 0.5.0；自检 iraf_sdk-ok+stubs
iraf-provider@ur5 active/enabled 50053 LISTEN
Health ready=True simulation=True capabilities=6
MuJoCo 窗口 0x400000b "MuJoCo : ur5e" 1280x720
```

### 3.2 .147

```
/opt/iraf-sdk/current -> 0.5.0
iraf-mcp-http  active 0.0.0.0:8765
iraf-mcp-piper active 0.0.0.0:8766
两个端点 healthz=200、POST 无 token=401
工具各 5 个
8765 MODE robot_dev profile=profiles/ur5_mujoco.yaml skills=15 simulation=True
8766 MODE robot_dev profile=profiles/piper_mujoco.yaml skills=15 simulation=True
```

### 3.3 端到端抓取

```
ur5  : execution_id=480ecb67-a982-5ff2-8476-2b79139de0aa status=SUCCEEDED
       lift_delta_m=0.07633 force_ok=true bilateral_contact=true
       center_distance_m=0.00011855145570256166
piper: execution_id=4b1ed285-a655-5114-9fd0-4d07853c61cd status=SUCCEEDED
       lift_delta_m=0.083308 force_ok=true bilateral_contact=true
       center_distance_m=0.0000514227433 regrasp.applied=true
```

两个场景跑完都复位（ur5 keyframe:0 / piper initial-snapshot），窗口保留。

## 4. 本次抓到的两个真缺陷

### 4.1 凭据取回漏键（已修）

现象：0.5.0 装完后 8765 提交任务返回
`token 必填（来源：环境变量或 600 凭据文件；缺 token 即拒绝装配，fail-closed）`。

真因：`board_upgrade.sh` 只取回 `/etc/iraf/runtime.secrets`；这块板历史上
`IRAF_PROVIDER_TOKEN` 存在独立的 `/etc/iraf/provider.token`（不在 runtime.secrets）
⇒ 新 `runtime.secrets` 只有 `IRAF_GRPC_TOKEN`。

修复（两层）：
1. `install_profiles.yaml` 声明 `credential_keys`（edge-intel: PROVIDER+GRPC；sim-x86: PROVIDER），
   `--secrets-file` 缺任一键即安装失败（写盘前拦）；
2. `board_upgrade.sh` 取回凭据时，如果 runtime.secrets 缺 PROVIDER 键且板上有
   `/etc/iraf/provider.token`，自动把该键并入临时 secrets 文件。

现场处置：把 provider.token 补进 runtime.secrets（600），并重算安装记录里该文件的
sha256（否则下一次受控卸载会以"哈希不符"拒绝）。

### 4.2 记录与 env 漂移（已加检测）

现象：受控卸载拒绝
`配置文件哈希不符（拒绝删除）：/etc/iraf/edge-intel.env`。

真因：前几轮切 ur5/piper 时手工改过 `/etc/iraf/edge-intel.env`，记录里的 sha256 过期。

处置：把旧 env 与记录备份到 `/root/iraf-0.4.1-backup-20261106/`，走
`board_deploy.sh --uninstall --force-clean`（只删声明路径，删 7 项，data/log 保留）。

新工具：
```
bash deploy/sdk/verify_record.sh --profile edge-intel
# 一致 → 0；漂移 → 4 并逐项列出 path / kind
```

### 4.3 装好的 SDK 不能独立跑安装/卸载入口（已修）

现象：在 `.147` 直接跑
`bash /opt/iraf-sdk/current/deploy/sdk/verify_record.sh --profile edge-intel`
报 `缺少声明解析实现 scripts/install_profile_check.py`；补进该文件后又报
`版本引用指向的文件不存在：pyproject.toml`。

真因：runtime bundle 的成员目录只有 `src/skills/profiles/config/deploy/scenes`，
`scripts/install_profile_check.py` 与 `pyproject.toml` 都不在包里；此前只有
`board_deploy.sh` 的临时 staging 目录能跑通。

修法：
- `lib_manifest.BUNDLE_EXTRA_FILES = ("scripts/install_profile_check.py", "pyproject.toml")`，
  缺任一文件时构建直接失败；
- `install-iraf.sh` / `uninstall-iraf.sh` / `verify_record.sh` 自行追加
  `PYTHONPATH=<repo>/lib:<repo>/src`，目标端不再依赖临时 pylib。

### 4.4 0.5.0 最终上板状态

- `.147` board_diag：IRAF 就绪=True，单元状态 ✓、CLI --list-skills ✓
- AgentOS 两个 server 的 verify 全 OK（service / host_api tools=5 / 两个 worker mcp+tools）
- 宿主日志同时注册 `iraf-task` 与 `iraf-piper`，各 9 个工具
- worker 的 `mcp==1.26.0` 是容器内手工安装；容器重建会丢（厂商镜像职责）

### 4.5 联合世界 Provider（2026-11-06 新增）

- 新增 `iraf-provider@joint`（50054）：scene=`build/scenes/handoff_lab/handoff_lab_joint.xml`，
  `robot_profile=profiles/piper_mujoco.yaml`；联合报告的 `manipulation.name_map` 注入
  `IRAF_BACKEND_CONFIG`，把 joint1… 映射到 piper_joint1…。
- 新增 `scenes/handoff_lab/scene-profile.yaml`：scene_id=joint-handoff、primary_robot=piper、
  resources=joint-piper/joint-ur5/joint-go2；ur5e/go2 的能力**留空**（未在本 Provider 验收）。
- 新增 `iraf-mcp-joint`（8767），AgentOS server 名 `iraf-joint`。
- 安装器修了两处联合模型断链：
  ① `file=` 绝对路径（构建产物写死 `/home/.../scenes/handoff_lab/assets/...`）→ 场景相对路径；
  ② `meshdir` 指向仓库 vendor → 拷进 `models/mesh_assets/` 并把场景 `assets/` 也并入其下
     （meshdir 会前缀所有 mesh 引用，实装报 `mesh_assets/assets/piper/base_link.STL`）。
- 实测：
  ```
  .121   iraf-provider@joint active :50054；Health ready=True capabilities=6
         窗口 0x400000b "MuJoCo : go2" 1280x720
  直连 pick：SUCCEEDED wall=123.68s center_distance_m=0.0013979652743957177
  .147   iraf-mcp-joint :8767 healthz=200 / 无 token 401
         edge 全链路 pick：execution_id=00779111-a9b3-5f8a-81c7-cf8d96079405
         lift_delta_m=0.093907 center_distance_m=0.0021519895262300053
  ```
- 口径：联合世界的 `realtime=true` 在 2 核机上把 12 s 动作拉到 **123.68 s** 墙钟；
  这是性能事实，不是失败。演示若不能接受，改 `realtime=false`（同一任务约 25 s）或换机器。

## 5. 未完成

- worker 的 `mcp==1.26.0` 是容器内手工安装；容器重建后会丢，需要厂商写进镜像。
- `board_diag` 的 "CLI 可运行（--list-skills）" 在本轮最终上板时已为 true；此前 0.4.1 为 false 的原因不再复现，未单独归因。
- 0.5.0 对外交付目录已重打包（build/upper-sdk-delivery-0.5.0 + tar.gz）。
- 联合世界 `realtime=true` 墙钟 123.68 s；是否改为 `realtime=false` 待演示口径确认。

## 6. 可复跑命令

```bash
# 构建
bash deploy/sdk/build_sdk.sh --target aarch64-manylinux_2_28-cp311

# .147 升级（示例，地址按现场）
SSHPASS=... IRAF_BOARD_HOST=10.203.247.147 \
  bash deploy/sdk/board_upgrade.sh --profile edge-intel --apply --version 0.5.0 \
  --provider-endpoint 10.203.247.121:50053 \
  --piper-provider-endpoint 10.203.247.121:50052 \
  --intent-model-url <url> --intent-model-name <model>

# .121 安装
bash deploy/sdk/install-iraf.sh --profile sim-x86 \
  --sdk-wheel build/sdk/iraf_sdk-0.5.0-py3-none-any.whl \
  --runtime-bundle build/sdk/iraf-runtime-0.5.0.tar.gz \
  --secrets-file <600-secrets> --allow-unverified \
  --env-set PROVIDER_BIND=0.0.0.0 --env-set PROVIDER_DISPLAY=:0 \
  --env-set PROVIDER_XAUTHORITY=<xauthority>

# 记录漂移
bash deploy/sdk/verify_record.sh --profile sim-x86
```
