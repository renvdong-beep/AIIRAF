# vendor 资产锁定（unitree_mujoco / Go2）——抓取路径与完整性门禁

- 日期：2026-09-20（步骤 12，战役 `iraf-24h`）
- 范围：`x86_64` / `/usr/bin/python3` 3.10.12 / MuJoCo 3.3.3；**不含真机**（边缘板卡不在场）
- 结论一句话：把上游 `unitree_robots/go2` 子树按 commit **`1eb6642e3f3fdfb7fb13a9794fd6a2dd93ea0e7d`**
  锁定为 **22 件 / 29 091 323 字节**，逐件 SHA-256 + git blob SHA-1 双向可复算；资产本体按仓库既有
  `.gitignore` 策略**不入库**，改由锁文件 + `scripts/fetch_vendor_assets.py` 按锁重取。

## 1. 现象与证据链

**现象 A（取文件挂死）**：在 `git clone --depth 1 --filter=blob:none --no-checkout` 的 partial clone 上
`git archive HEAD unitree_robots/go2` 挂死 —— **400 秒零输出、输出的 tar 为 0 字节**。

**现象 B（列路径却拉全仓）**：`git ls-tree -r -l HEAD -- unitree_robots/go2`（为拿文件大小）触发全仓 blob
拉取 → `Failed to connect ... after 133977 ms`。**只用 `--name-only` 不会下载**。

**现象 C（逐件取在个别对象停滞）**：`git cat-file blob HEAD:<path>` 逐件可用（小文件约 5 s），但
`hip_0.obj` 上撞过 **120 s 与 300 s 两次超时**。

**现象 D（批量任务被未捕获异常打死）**：抓取批处理在**第 13 件**上以未捕获回溯中止，前 12 件全部白抓；
根因是重试的 `except` 只写了 `(urllib.error.URLError, OSError)`，而 `http.client.IncompleteRead`
派生自 `HTTPException`（**不是** `OSError`），因此 `base_4.obj`（7.7 MB）的 `IncompleteRead` 直接逃逸。

**现象 E（host 可达性不能一次判定）**：`github.com` 探测超时、`raw.githubusercontent.com` **不可达**，
而 `api.github.com` 与 `codeload.github.com` 可达。

## 2. 根因与修法（均为实测，不是推断）

| 现象 | 根因 | 修法 |
|---|---|---|
| A | partial clone 里 `git archive <subtree>` 需要按树补拉全部 blob，退化为大请求 | **不用** `git archive`；改逐件取 |
| B | `-l` 需要对象大小 ⇒ 触发全仓 blob 拉取 | 列路径用 `git ls-tree -r --name-only`（不触发下载） |
| C | 单对象停滞 | per-file `timeout` + 重试；**断点续抓**：盘上已存在且 `sha1("blob <len>\0"+content) == 锁内 blob_sha1` ⇒ 直接复用（记 `source=reused_verified_local`），不走网络 |
| D | `except` 漏 `http.client.HTTPException` | 重试的 `except` 补齐 `HTTPException`（现有实现见 `scripts/fetch_vendor_assets.py::download`） |
| E | 单 host 判定 | 抓取走 **GitHub Contents API + `Accept: application/vnd.github.raw`**：`hip_0.obj`（2 863 864 B）秒级返回；`base_4.obj` 偶发 `IncompleteRead` 由重试覆盖 |

**provenance 交叉校验（字节级，且不需要网络）**：`sha1(b"blob %d\0" % len(data) + data)` 必须等于
`git ls-tree HEAD -- <path>` 的第三列。只取 oid、不取内容，所以不触发 blob 下载，却能把任何来源
（API / 镜像 / 人工拷贝）的字节证明为与锁定 commit **逐位相同**。实测 `LICENSE`（1559 B）
→ `42d2e648c8881ea075a3bc386c91669290b2e386`，与上游一致；22 件资产全部落 `blob_sha1` 进锁。

## 3. 门禁设计（`scripts/verify_vendor_lock.py`，双向 + 可选只读）

退出码：`0` 通过 / `1` 用法错误 / `2` 锁缺失·非法·声明不一致（含 `--expect-commit` 不符）/
`3` 内容不符（哈希·字节·缺失）/ `4` 存在未登记文件 / `5` 只读门禁失败。

- **正方向**：逐件 SHA-256 + 字节复算；`file_count == len(files)`、`total_bytes == Σbytes`、
  sha256 为 64 位小写十六进制、路径必须 root 相对且不得越界 —— 任一不满足即退 2，**不用默认值兜底**。
- **反方向**：root 下出现未登记文件即退 4，防止资产被悄悄增删。
- **自指文件**：锁自己就在被清点的目录里 ⇒ `non_asset_files` 显式声明白名单
  （`source-lock.json`、`LICENSE-BOM.md`），写入侧与比对侧共用同一份声明（与步骤 09 的自指安装记录同一纪律）。
- **只读位不是可移植门禁**：git 只保存可执行位、**不保存写位**，新克隆里资产必然可写 ⇒ 默认只如实报告
  `writable_files`，`--require-readonly` 才硬失败（退 5），报告里带固定说明字段 `readonly_portability_note`。
  把「本地 chmod 过」当 CI 门禁就是制造假绿。
- **报告是纯 JSON**（stdout，可 `--json-out`），且不得写本机绝对路径；单测里调用 CLI 必须用
  `contextlib.redirect_stdout`，否则证据文件本身会被 JSON 噪声污染。

## 4. 「涉及文件」与 `.gitignore` 的冲突：先量，再决

- `.gitignore` 第 11 行注释明示策略：**「厂商机器人模型：第三方代码，不入库。通过
  `vendor/*/source-lock.json` 锁定来源与哈希，需要时按锁文件重新拉取。」** 且 `git ls-files vendor/` = **0 件**。
- 步骤 12 的「涉及文件」却要求入库 `vendor/unitree_go2/unitree_robots/go2/**`。两者冲突，量出的事实：
  本步资产 **28.4 MB 网格 + 0.65 MB 贴图**，而仓库**当前最大已跟踪文件 530 740 B**；
  29 MB 二进制入库会与既有策略直接对撞，且战役硬规则禁止 `git add -f`。
- 本轮取**中间路线**（可逆、可审查，不替用户拍板体积决策）：
  1. 入库**声明与工具**：锁文件 + 许可证 BOM + 校验器 + 按锁重取入口 + 夹具单测；
  2. 资产本体**不入库**（保持既有策略），用**精准反忽略** `!vendor/unitree_go2/source-lock.json`
     放行锁文件，并新增 `vendor/unitree_go2/unitree_robots` 忽略行把「厂商模型不入库」在同一步落到 unitree；
  3. 「按锁重取」变成**可执行契约**（`scripts/fetch_vendor_assets.py`，锁驱动、`--dry-run`/`--apply`/
     `--require-complete`），而不是注释里的一句话（AGENTS.md 5.2）。
- **资产本体是否入库 = 体积/策略决策，留给人工**（编号选项见步骤台账 note 与 tick 汇报）。
- 反忽略的安全性由单测闭环：`test_real_lock_report_has_no_local_path` /
  `test_lock_shape_and_pinned_commit` 断言锁内与报告内**无本机绝对路径** —— 这正是原忽略行给出的理由，
  必须被证明不成立才允许放行。

## 5. 许可证 BOM（`vendor/unitree_go2/LICENSE-BOM.md`）

- 上游主许可证 `BSD-3-Clause`，版权行 `Copyright (c) 2016-2024 HangZhou YuShu TECHNOLOGY CO.,LTD.
  ("Unitree Robotics")`；`LICENSE` 文本 sha256 `a5d73fc4…30bf25`（1559 B），blob sha1 `42d2e648…b2e386`。
- **未覆盖范围必须显式登记**（仓库主许可证不能替代逐项审查）：上游 `simulate/` 下有第三方
  `LICENSE-2.0.txt`、`lodepng/LICENSE`；其它本体目录（g1/h1/h1_2/h2/r1/b2/b2w/go2w/a2/as2）未锁定；
  `unitree_sdk2` / `unitree_ros2` / 固件与 Go2 EDU SKU 授权**未收集**；逐项资产审查**未完成**。
- `license.review_state = pending_per_asset_review` 是 **fail-closed 分发门禁**：未改为已审查即禁止对外分发。

## 6. 模型实测旁证（证明最小集自洽，且**不含传感器**）

| 模型 | 加载 | nq | nv | nu | nbody | ngeom | ncam | nsensor | timestep |
|---|---|---|---|---|---|---|---|---|---|
| `go2.xml` | ok | 19 | 18 | 12 | 18 | 56 | **0** | 41 | 0.002 |
| `scene.xml` | ok | 19 | 18 | 12 | 18 | 65 | **0** | 41 | 0.002 |
| `scene_terrain.xml` | ok | 19 | 18 | 12 | 18 | 162 | **0** | 41 | 0.002 |

`go2.xml` 是 **12 个 `<motor>`（力矩型，非位置型）**，**`ncam = 0`**：厂商模型不含相机/雷达 ⇒
传感器必须由 IRAF 场景构建器按声明注入（步骤 13），**厂商文件保持只读、不得改写**。

## 7. 复现命令

```bash
PYTHONPATH=src /usr/bin/python3 scripts/verify_vendor_lock.py --lock vendor/unitree_go2/source-lock.json            # exit=0
PYTHONPATH=src /usr/bin/python3 scripts/verify_vendor_lock.py --lock vendor/unitree_go2/source-lock.json --require-readonly
PYTHONPATH=src /usr/bin/python3 scripts/verify_vendor_lock.py --lock vendor/unitree_go2/source-lock.json --expect-commit 1eb6642e3f3fdfb7fb13a9794fd6a2dd93ea0e7d
PYTHONPATH=src /usr/bin/python3 scripts/fetch_vendor_assets.py --lock vendor/unitree_go2/source-lock.json --dry-run   # 不联网、不写盘
PYTHONPATH=src /usr/bin/python3 scripts/fetch_vendor_assets.py --lock vendor/unitree_go2/source-lock.json --apply     # 新克隆后按锁重取
PYTHONPATH=src /usr/bin/python3 -m unittest tests.unit.test_vendor_lock -v                                          # Ran 32 / OK
bash build/iraf-24h/12/run_acceptance.sh                                                                            # A1~A7 全绿
```

## 8. 未完成 / 缺口（不伪造）

1. **资产本体入库与否**属体积/策略决策，本轮按既有 `.gitignore` 策略不入库；若决定入库需另行放行
   （白名单）并重新评估仓库体积（当前最大已跟踪文件 530 740 B）。
2. 逐项资产许可审查、SDK/固件授权收集**未完成**（`review_state=pending_per_asset_review`）。
3. 人形（g1/h1/…）资产**未锁定**，按 ADR-0007 只作后续静态模型候选。
4. 新克隆无资产时依赖资产的 2 条单测**显式 skip**（不是通过），由
   `build/iraf-24h/12/check_fresh_clone.py` 证明两向行为；`--require-complete` 才是 CI 的硬门禁。
5. 目标端/真机结论不在本步范围（板卡不在场）：本步只产出 `simulation=true` 的仿真资产证据。
