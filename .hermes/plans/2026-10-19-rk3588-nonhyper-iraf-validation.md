# 非 Hyper 板卡（RK3588 / topeet）上验证 IRAF v1 板卡侧

日期：2026-10-19　状态：进行中

## 1. 目标

在**没有 Hypervisor 的边缘 aarch64 板卡**上，验证 IRAF v1 板卡侧的三条完成定义（AGENTS §0.1）：
1. 按声明装得上（install-iraf.sh 全链路 + 后置自检）
2. 能干净卸载（uninstall 只按记录 + verify_residue 六路全 0）
3. 能经版本化通道调 x86 的仿真 Capability Provider（板上 TaskFlow/Skill/Policy，x86 只执行与显示）

明确**不在**本工作范围：HyperProfile/Release manifest 构型声明、VM 数量/编号/IP/角色映射（铁律 6.10）、
2VM 合并部署的安全边界（铁律 6.11）、VM 级权限/资源限额/设备所有权隔离 —— 这块板没有 VM。

## 2. 实测事实（2026-10-19，只读采集）

证据目录：`build/board/rk3588-topeet/`（collect.json、probe-status.tsv 22 探针全 rc=0、raw/）

```
IP            10.203.247.239（10.203.247.221 / .74 当前均不可达）
内核          Linux 5.10.198 aarch64
发行版        Ubuntu 20.04.6 LTS
glibc         2.31-0ubuntu9.18
systemd       245
资源          8 核 / RAM 15949 MB / 根 58G 已用 46G，余 9.3G
解释器        /usr/bin/python3 = 3.8.10（pip 20.0.2）
              /usr/local/bin/python3.10 = 3.10.12（pip 23.0.1）  ← AgentOS 用这个
Hyper 判定    无 /etc/iaros-version；domains 探针只有 shm；manifests 为空 ⇒ 单系统，无 Hypervisor
AgentOS       在跑：agentos.api_server / 2×container_worker / mem_manager / agentos.host_worker
              dashboard --host 0.0.0.0 --port 9119
ROS2          /opt/ros/foxy（edge-intel 不需要 ROS2）
端口          50051 / 8765 均未监听（IRAF 声明的两个端口都空）
IRAF          未安装（no-iraf-console-scripts / no-iraf-units）
出网          mirrors.aliyun.com HTTP 200，connect 0.060 s
```

## 3. 关键结论：为什么"天然装得上"

- 我们自己的 SDK wheel 是 `iraf_sdk-0.2.0-py3-none-any.whl`（纯 Python，与 ABI 无关）。
- 板上 3.10.12 的 ABI 标签就是 **cp310**。
- 真正绑 ABI 的只有第三方二进制轮：numpy / grpcio / protobuf / rpds-py（仿真轮 mujoco/glfw 不在板卡上装）。
- 现有交付 wheelhouse 是 `aarch64-manylinux_2_28-cp311` 抓的 ⇒ 与 cp310 不匹配，pip 会拒。**这是唯一的卡点**，
  不是板卡能力问题。

## 4. 契约层缺口（先改契约再改实现，铁律 2.1）

`config/sdk/package_matrix.schema.json` 里的 BoardProfile 段要求 `metadata.hyper_models` 为
`minItems: 1` 的数组 ⇒ **现有契约无法合法表达"这块板没有 Hyper 构型"**。
不能为了填进去而编造构型名（铁律 6.10）。

处置：契约新增"单系统（无 Hyper）"形态（例如 `metadata.hyper_models: []` + 显式
`spec.target.hypervisor: none`，由 schema 的 allOf 约束两者一致），再新增板级声明。
这是**契约变更**，必须先落在 schema 上并配套负向用例。

## 5. 执行步骤（每步给数字证据）

- [ ] S1 构建目标新增 `aarch64-manylinux_2_28-cp310`（config/sdk/package_matrix.yaml）
      判据：`fetch_wheelhouse.sh --target ... --dry-run --no-resolve` 退 0 且打印解析结果
- [ ] S2 抓 cp310 wheelhouse 到 `build/wheelhouse/aarch64-manylinux_2_28-cp310/`
      判据：`--verify` 退 0；逐个 wheel 标签含 cp310/none-any
- [ ] S3 修"单目标"隐含假设的测试（预期连锁：test_profile_check_board / test_sdk_manifest /
      test_wheel_tags 三族），保留"多目标必须显式 --target"作为拒绝路径用例
      判据：全量单测绿
- [ ] S4 契约新增"无 Hyper"形态 + BoardProfile `profiles/boards/topeet_rk3588.yaml`（实测值回填，
      不可读到的写 unverified）+ 安装角色（边缘 aarch64 单系统，实验档）
      判据：`install_profile_check.py --profile <新角色>` 退 0 并打印生效值
- [ ] S5 沙箱全链路演练（rehearse_install_chain）先绿，再上板
- [ ] S6 上板 `--apply`（需用户授权）：装 → board_diag → 起 gRPC → x86 侧跨机调用 → 卸载 → 残留核验 0
      判据：三条证据各自 exit 0 + 数字（文件数、卸载项数、六路 = 0）
- [ ] S7 留档：证据入 build/acceptance/、调试记录入 docs/debug/、经验回写技能

## 6. 诚实性口径（不得省略）

- 这块板 = 边缘 aarch64 单系统、Ubuntu 20.04（不在 AGENTS 6.7 的 22.04/Humble 认证基线）⇒ 一律标
  **实验档 / 未认证**，不得写"受支持"。
- 能力结论仍是 `simulation=true`（Provider 在 x86 仿真侧），不构成真机能力证据。
- `--allow-unverified` 只用于探索；装完 ≠ 验收。
