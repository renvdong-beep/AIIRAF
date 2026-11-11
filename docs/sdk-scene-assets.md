# SDK 场景资产与 ur5 / handoff_lab 可用性（2026-11-11）

## 现象

同事装了 SDK 后，ur5 与 handoff_lab 不可用。

## 根因（实测）

1. **runtime bundle 不含 `build/`**（构建产物不进交付包），而 `sim-x86` 声明里的
   `sim_scene` 指向 `build/models/ur5-pick-scene.xml`、`build/scenes/handoff_lab/handoff_lab_joint.xml`
   ⇒ 干净机器安装时直接报"声明的仿真场景缺文件"。
2. 已发布的 0.4.3 bundle 中 `sim-x86.services` **只有 piper/ur5**，没有 `handoff_lab_joint`。
3. `sim-x86` 角色**不安装 Provider Supervisor**（8770），需另跑 `deploy/install_second_host.sh`；
   没有控制面时边缘端 ensure 必然失败。
4. 注册表与网络耦合：`switch_name`（joint → handoff_lab_joint）、板卡出口 IP 白名单等需要对齐。

## 修法（0.4.4 起）

- **预置场景资产随包**：`deploy/sdk/prebuilt/scenes/<stem>/`，每个目录含
  `<stem>.xml`、`<stem>.json`、`<stem>_assets/`；资产按内容去重、引用改为相对路径、删除
  `meshdir/texturedir`。安装器在仓库缺少 `build/` 时自动回退到预置资产。
- **`sim-x86.services` 增加 `iraf-provider@handoff_lab_joint`（50054）**，残留清单同步。
- 解析不出的场景报错带两条修法（现场构建 / 用预置资产），不再只报缺文件。
- 重新生成预置资产：`python3 deploy/sdk/gen_prebuilt_scenes.py`（场景变更后必须重跑）。

## 安装前提（同事版）

```bash
# x86 仿真机：装运行时（含 3 个 Provider）
bash deploy/sdk/install-iraf.sh --profile sim-x86 \
  --sdk-wheel build/sdk/iraf_sdk-0.4.4-py3-none-any.whl \
  --runtime-bundle build/sdk/iraf-runtime-0.4.4-aarch64.tar.gz \
  --run-user <user> \
  --env-set PROVIDER_BIND=0.0.0.0 --env-set PROVIDER_DISPLAY=:0 \
  --env-set PROVIDER_XAUTHORITY=/run/user/1000/gdm/Xauthority

# 控制面（sim-x86 档不含）：另跑
bash deploy/install_second_host.sh --host-id x86-1 --board <板卡IP>

# 板卡：注册表 + MCP（config/scenes|providers 随 bundle 分发；改完要 --seal）
```

## 验证口径（0.4.4 沙箱实证）

在没有 `build/` 的沙箱根上安装，得到 3 个 provider 单元与 3 份装配单；
`mujoco.MjModel.from_xml_path()` 对三条场景全部加载成功：
piper `bodies=11 meshes=10`、ur5 `bodies=24 meshes=28`、handoff `bodies=52 meshes=54`。
