# x86 端「装完 SDK 就有 Provider」：sim-x86 角色固化 iraf-provider 服务（2026-10-26）

## 1. 起因（使用者口径）

「应该基于 IRAF 在 x86 端安装的 SDK 进行固化 Provider —— 当 x86 当 SDK 安装后，已经具备 Provider 的条件。」

改之前的事实：Provider 只能由仓库脚本 + 会话前台拉起（本日演示用的 pid 1354301 就是这么来的），
装完 SDK 的 x86 机器**没有任何服务**，所以"会话断了/机器重启了 Provider 就没了"是必然结果。

## 2. 改前差距（逐条有出处）

| 差距 | 出处 |
|---|---|
| sim-x86 角色 `services: []`，注释"不起常驻服务" | config/install/install_profiles.yaml（改前 155-157 行） |
| sim-x86 无 `env_template` / `robot_profile` / `safety_policy` / `sim_scene` | 同上（改前 132-167 行） |
| sim-x86 `runtime_dependencies` 缺 grpcio、protobuf（服务端要 grpc + iraf.v1 桩） | 同上（改前 149-154 行） |
| sim-x86 residue 无单元无端口 ⇒ 装了服务也卸载不干净 | 同上（改前 163-167 行） |
| runtime 交付包**不含 scripts/** ⇒ `scripts/iraf_provider_serve.py` 装不上 | `tar -tzf build/sdk/iraf-runtime-0.3.17-aarch64.tar.gz`（只有 src/ 与 deploy/sdk/{units,env}） |
| 装配单端口与 `deploy/iraf-runtime.env` 交叉校验不含 50052 | scripts/install_profile_check.py（改前只认 IRAF_RUNTIME_PORT / IRAF_GRPC_PORT） |

## 3. 改动（四个提交）

1. `7d696c7` **入口下沉**：`src/iraf_adapters/remote/provider_cli.py`（`python -m iraf_adapters.remote.provider_cli`），
   `scripts/iraf_provider_serve.py` 改 3 行转发。理由：交付包不含 scripts/，服务单元不能指向仓库脚本。
   同一模块兼容两种布局（仓库 `<repo>/build/generated/python`、安装 `<version>/lib` 桩路径）。
2. `53a6df3` **声明与模板**：sim-x86 补 `robot_profile` / `safety_policy` / `sim_scene` / `provider_display` /
   `env_template` / `services[iraf-provider:50052]` / `residue.units+ports`；新增
   `deploy/sdk/units/iraf-provider.service.tmpl`、`deploy/sdk/env/sim-x86.env.tmpl`；
   `lib_install.py` 新增 `unit_dir()` 统一单元落点（user 作用域非 root ⇒ `~/.config/systemd/user`，
   否则 `/etc/systemd/user`）、注入 `BACKEND_CONFIG/PROVIDER_PORT/PROVIDER_DISPLAY_MODE`、
   `deploy/iraf-runtime.env` 补 `IRAF_PROVIDER_PORT=50052`。
3. `7e1f9f9` **校验与测试**：新增三条结构性规则（仿真 Provider 角色必须有服务 / 装配单模板 / 场景），
   IP 判据区分"协议常量（127.0.0.1、0.0.0.0）"与"具体机器地址"；`tests/unit/test_install_profiles.py`
   加 4 条负向用例。校验器：**105 项检查 0 项未通过**。
4. 本文件 + 版本 0.3.19。

## 4. 验收证据（本机 10.203.247.121，2026-10-26 实装实跑）

安装（真实安装，非演练）：

```
bash deploy/sdk/install-iraf.sh --profile sim-x86 \
  --sdk-wheel build/sdk/iraf_sdk-0.3.19-py3-none-any.whl \
  --runtime-bundle build/sdk/iraf-runtime-0.3.19-aarch64.tar.gz \
  --secrets-file build/provider-up/runtime.secrets \
  --env-set PROVIDER_BIND=0.0.0.0 --env-set PROVIDER_DISPLAY=:0 \
  --env-set PROVIDER_XAUTHORITY=/run/user/1000/gdm/Xauthority
→ 退出码 0；lib 依赖 + 桩 11 个；IRAF_BACKEND_CONFIG 由场景报告生成 3759 字节（场景=piper-pick-scene.xml）
  env  /home/coretek/.config/iraf/sim-x86.env
  单元 /home/coretek/.config/systemd/user/iraf-provider.service
  current -> 0.3.19；记录 9043 个文件（版本目录 9041 + 配置 2：env + runtime.secrets）
```

服务（安装后直接起，不再需要仓库脚本）：

```
systemctl --user enable --now iraf-provider → active，LISTEN *:50052
X 窗口 0x620000b "MuJoCo : piper_description" 1280x720（窗口真的在桌面上）
```

跨机能力（scripts/probe_provider_health.py，端点 10.203.247.121:50052）：

```
reachable=true ready=True simulation=True
capabilities=6 move_joint,pick_object,place_object,reset_scene,stop,visual_pick
inventory: mode=simulation safety={'estop': False}
```

复位→立刻动（scripts/probe_e2e_reset_then_move.py）：

```
[1] reset_scene accepted 0.08s baseline=initial-snapshot exec=exec-56ac02c3…
[2] move_joint  accepted 0.01s exec=exec-301eb24a…
[3] move_joint  accepted 0.01s exec=exec-bdcccbd0…
[5] reset_scene 收尾复位 baseline=initial-snapshot
结论：3 次执行互不阻塞、execution 各不相同、总墙钟 0.10s —— 无 IRAF-RESOURCE-BUSY
```

干净卸载（受控卸载 + 六路残留核验）：

```
uninstall-iraf.sh --profile sim-x86 → 删除 9050 项（保留 data/log 两项数据目录）
verify_residue.sh  --profile sim-x86 → files 0 / dirs 0 / env_files 0 / ports 0 /
                                       processes 0 / unlisted 0 / missing 0 / unit exists=False
结果：干净（六路全 0）
```

## 4.1 实装时抓到并修掉的四个真缺陷（都不在计划里）

| 现象 | 根因 | 修法 |
|---|---|---|
| `Unit network.target not found`，服务起不来 | user 作用域 systemd 里没有 `network.target` | 单元模板去掉 After/Requires=network.target |
| `status=216/GROUP Failed to determine supplementary groups` | user 作用域写 `User=@RUN_USER@`，用户级 systemd 无权切身份 | 模板去掉 User=（安装器对非 system 作用域强制删 User=/Group=） |
| `Error opening file '../../../piper_ros/.../link1.STL'`，装配失败 | 场景 XML 用**相对路径**引用 mesh，拷进版本目录后断链 | 安装器把被引用的 mesh 目录一起装进 `<version>/models/meshes` 并把引用改写为 `meshes/`（安装物自包含） |
| 卸载后残留 `<config_dir>/runtime.secrets` | 凭据文件只进了"回滚清单"，没进**安装记录**的 config_files | `write_record(config_paths=[env, secrets])` ⇒ 卸载随记录删除（凭据不留残留） |


## 5. 边界与诚实性

- 本角色仍属**仿真域**：单元 Description 与装配单都写明 `simulation=true`，任何结论都不构成真机能力证据（铁律 1.7）。
- `provider_display: window` 需要部署期给 `PROVIDER_DISPLAY` / `PROVIDER_XAUTHORITY`；缺失时安装**显式失败**
  （不猜路径），`--env-set PROVIDER_DISPLAY_MODE=headless` 可走无窗模式。
- 服务重启 = MuJoCo 世界重载（场景回到 initial-snapshot）。这是"演示前场景已复位"的免费好处，但必须在操作手册里写明，不能变成悄悄重置。
- `scripts/dev_provider_up.sh` 保留为"从仓库源码跑"的开发入口；**演示/交付走安装后的 systemd 单元**。
