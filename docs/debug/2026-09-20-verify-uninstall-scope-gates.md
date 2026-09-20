# 2026-09-20 目标端自检与卸载的范围门禁（步骤 09）

- 范围：`deploy/sdk/verify.sh`、`deploy/sdk/uninstall.sh`、`deploy/sdk/lib_target_verify.py`
- 战役：`iraf-24h` 步骤 09（x86-first：边缘板卡不在场，目标端验收一律 DEFERRED）
- 结论：4 个实现缺陷 + 1 个**安装流水线顺序缺陷**已在同一 tick 内定位并修复；
  修复后验收 26 项全过（`build/iraf-24h/09/acceptance-rerun.txt`），未放宽任何既有门禁。
  （首轮 24 项在 `acceptance.txt`，失败 2 项即下面第 2 条的**第二次**表现；修复后未再复现。）

## 症状 → 证据 → 根因 → 修法

### 1. 卸载在干净沙箱上也报"找不到已装版本"（退出 1）

- 症状：`bash uninstall.sh --root <沙箱>` 在刚装好的沙箱上退出 1；
  8 个卸载用例中 7 个失败（`build/iraf-24h/09/unittest-uninstall.txt`）。
- 证据：`[target_verify][错误] 在 /tmp/iraf-uninstall-* 下找不到激活链接指向的已装版本`。
- 根因：定位已装矩阵用的是 `root.glob("*/"*/<MATRIX_REL>")`（只匹配前缀下一级的固定深度），
  而实际布局是 `<root>/opt/iraf-sdk/0.2.0/config/sdk/package_matrix.yaml`——前缀层级由声明决定，
  不是固定的两级。**深度假设即隐性硬编码**。
- 修法：改用 `root.rglob(MATRIX_REL)`，以 `path.parents[2]` 反推版本目录，再用声明自校验
  （`version_dir.parent == <root>/<delivery.install.prefix>`），不一致即拒绝。

### 2. 安装记录自身被当成"清单外文件"（全部卸载用例退出 1）

- 症状：`[target_verify][错误] 发现清单外文件 1 个：… - iraf-sdk-files.json`。
- 根因：`compare_record` 比对"记录 `files` ↔ 版本目录实际文件"时，记录文件本身在版本目录内，
  却不可能登记自己的哈希 → 永远多出一个"清单外文件"。这是**自指**问题，不是数据问题。
- 修法：比对时显式排除记录文件，并在结果里写 `record_file_excluded`，记录 notes 里同步说明
  （"记录自身不登记自身哈希"），**不做静默忽略**。另有 `--version` 分支同样收敛到 `parents[2]`。
- **同一缺陷的第二次表现（只在比对侧排除是不够的）**：只改 `compare_record` 后，干净沙箱上
  `uninstall-dry-run` 与 `uninstall-apply` 两项仍退出 1，报
  `[target_verify][错误] 记录内文件缺失 1 个：… - iraf-sdk-files.json`
  （证据 `build/iraf-24h/09/cases/uninstall-dry-run.out`，首轮验收 24 项中失败的那 2 项）。
  根因：写入侧 `write_install_record` 仍用 `collect_tree_files(version_dir)` 全量清点，把**上一份**
  记录登记进 `files`；比对侧又把它排除，于是"记录里声明、实际树里查不到"→ 整条卸载在**全量树上**
  拒绝（**规模无关、次数相关**：只有记录被**生成两次**才暴露——install 演练写一次、随后的
  `verify.sh` 后置校验再写一次，第二次才把上一份记录清点进去；单测的小沙箱只生成一次，测不出来）。
  修法：写入侧与比对侧用**同一个** `record_name` 过滤（`lib_target_verify.py` 写入侧
  `item["path"] != install["file_record"]`，比对侧 `actual.pop(record_name, None)`），保持双向对称。
  修复后实测：`uninstall-apply.json` 的 `file_count_record=161` 与 `file_count_actual=161` 相等
  （修复前为 162/161），`record_file_excluded="iraf-sdk-files.json"`，`extras=[]`，`removed=4`。
  教训：自指文件必须**双向**排除；只修一侧会把"多一个"变成"少一个"。

### 3. 非演练失败路径缺 `service_state`（证据契约破损）

- 症状：单测 `test_service_not_running_is_reported_and_fails` → `KeyError: 'service_state'`。
- 根因：`service_state` 只在演练分支里赋值；非演练判定失败时直接抛错，证据里缺这个键。
  下游（人/脚本）因此无法区分"服务未启动"与"探针不通过"。
- 修法：把 `service_state` 的计算提到 dry-run 分支之前，三种取值
  `SERVICE_NOT_RUNNING` / `unhealthy_in_probe` / `healthy_in_probe`；演练分支仅在
  "未启动或不健康"时覆盖为 `SERVICE_NOT_RUNNING`。断言器 `check_evidence.py not-running`
  现在钉住 `service_state`、`health.state`、`exit_code=5` 与 failure 文案含「服务未启动」。

### 4. 事件库写入检查越出 `--root` 沙箱（写真实 `/var/lib`）

- 症状：stub 正向对照用例退出 5：`事件库目录不可创建（/var/lib/iraf）：[Errno 13] Permission denied`。
- 根因：非演练分支直接采用**声明的目标端绝对路径**，忽略了 `--root` 是沙箱。沙箱执行的副作用
  溢出到真实系统路径——这正是"演练不得触碰真实系统"要禁止的。
- 修法：`--root ≠ /` 时把声明路径重挂到 root 下（`scope=declared_path_under_root`），
  只有 `--root=/` 才用原样路径（`installed_env_file` / `target_store`）；演练永远走
  `sandbox_scratch` / `ephemeral_scratch`，**不触碰真实事件库**。

### 5. 【流水线顺序】systemd 单元晚于后置 verify.sh 渲染 → V2 假失败（退出 2）

- 症状：`install.sh --dry-run` 整体退出 0，但报告里
  `V2 后置校验 verify.sh` = `REHEARSAL_BLOCKED`（`后置校验 deploy/sdk/verify.sh 失败（退出码 2）`），
  且 `service_not_running: 0`（说明失败发生在探针之前）。
- 证据：手工重跑同一沙箱里的 `verify.sh --root … --dry-run` **退出 0**——差别只在"install 未跑完"，
  指向顺序而非逻辑。
- 根因：verify.sh 安装后要生成**安装记录**，记录必须同时登记目录外文件（env 文件 + systemd 单元）。
  install 在 `postcheck` 里先跑 V1/V2 后渲染单元文件，于是 V2 时单元还不存在 →
  `安装记录无法生成：systemd_unit 缺失` → 退出 2。**"预检要用到的声明/文件必须先可见"**这条
  在步骤 08 已踩过一次（runtime 布局声明可见性），这里是同一类错误的第二次出现：这次是
  "后置校验要用到的产物必须先落盘"。
- 修法：把单元渲染抽成 `Installer.render_unit()` 并在 `postcheck` 开头调用；阶段顺序与报告内容
  不变（V1 → V2 → V3），V3 仍只负责 systemctl 与阶段记录。

## 探针的诚实边界（不是缺陷，是设计约束）

- gRPC 探针三级降级：`grpc_health_v1` → `grpc_channel_ready` → `tcp_connect`。本机无 grpcio，
  结果里写 `probe_kind=tcp_connect` 与 `limitation`（"只证明 TCP 端口可达，不得表述为 gRPC
  健康检查通过"），断言器强制该字段存在。
- 事件库检查只写**独立探针表** `iraf_verify_probe`（建表 + 写行 + 回读 + `PRAGMA integrity_check`），
  不碰运行时业务表；探针行数计入证据。
- 本机 8765 端口确有开发仿真服务在跑（实测 `HTTP 200`、`body_status=ok`），因此 S5 通过属
  **本机开发服务**证据；"服务未启动"分支由关闭端口（`127.0.0.1:1`）稳定复现，退出 5。
  目标端 `/health`、systemd、真实事件库验收一律 DEFERRED（板卡不在场）。
- runtime 解包内容不在安装记录逐文件范围内（bundle 侧只保证 runtime tar 的 SHA-256）：
  这是已知缺口，写在安装记录 notes 里，不当作"已覆盖"。

## 复现命令

```bash
cd /home/coretek/AIIRAF
export PYTHON=/usr/bin/python3
bash build/iraf-24h/09/run_acceptance.sh                      # 26 项验收（含 5 种篡改、范围拒绝、退出 5）
PYTHONPATH=src $PYTHON -m unittest tests.unit.test_uninstall_scope -v
PYTHONPATH=src $PYTHON -m unittest tests.unit.test_target_verify_evidence -v
```
