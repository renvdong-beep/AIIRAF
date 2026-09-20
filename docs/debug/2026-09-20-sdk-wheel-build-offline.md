# SDK 打包（步骤 06）：离线环境下的 wheel 组装、stub 与篡改检测实测记录

- 日期：2026-09-20　范围：`plans/iraf-24h/06-build_sdk.sh 打包与 manifest.md`（x86-first 战役）
- 产物：`deploy/sdk/build_sdk.sh`、`deploy/sdk/lib_manifest.py`、`tests/unit/test_sdk_manifest.py`
- 证据目录：`build/iraf-24h/06/`（`build/` 已 gitignore）；主证据 `build/iraf-24h/06/build-sdk.txt`
- 解释器口径：`/usr/bin/python3`（3.10.12，mujoco 3.3.3、protobuf 4.25.7、PyYAML 5.4.1、setuptools 59.6.0、wheel 0.37.1）

本记录只写**实测**结论；未验证的部分在第 7 节单列，不得当作通过。

## 1. 现象：标准打包路径要么不可用，要么"绿灯 + 假产物"

```bash
$ /usr/bin/python3 -m build --version
/usr/bin/python3: No module named build.__main__; 'build' is a package and cannot be directly executed
$ cd /tmp && /usr/bin/python3 -m pip show build
WARNING: Package(s) not found: build          # 从 /tmp（无 build/ 目录遮蔽）才看得出真相
```

- 上面那条 `-m build` 的报错其实是**仓库根有 `build/` 目录造成的遮蔽**（cwd 在 `sys.path` 上），
  并不证明 `build` 模块存在；真正的结论是 **pypa/build 未安装**，且官方源不可达、无法离线安装。
- 仓库没有 `setup.py` / `setup.cfg`，`[project]` 表是 PEP 621，而本机 setuptools 是 **59.6.0（< 61）**：

```bash
$ /usr/bin/python3 -m pip wheel . --no-build-isolation --no-deps -w build/iraf-24h/06/wheel-try
  Preparing metadata (pyproject.toml): finished with status 'done'
  Created wheel for UNKNOWN: filename=UNKNOWN-0.0.0-py3-none-any.whl size=961
Successfully built UNKNOWN
exit=0
```

**根因**：setuptools 59.6.0 不解析 `[project]`，于是"成功"产出一个 **961 字节的空包**
（`UNKNOWN-0.0.0-py3-none-any.whl`，无包内容、无正确元数据）。退出码 0 但产物是假的——
这正是"不得只看退出码，必须校验产物内容"的实例。

**修法**：用标准库手工组装 wheel（`zip` + `<dist>-<ver>.dist-info/{METADATA,WHEEL,RECORD}`，
固定 `ZipInfo.date_time=(1980,1,1,0,0,0)`），并在 `validate_wheel` 做三重校验：
文件名/版本/标签必须等于声明、METADATA 名称版本正确、RECORD 覆盖全部成员、
**无 `Requires-Dist`**（SDK 零第三方依赖）、无二进制扩展。
上述实测形状（`UNKNOWN-0.0.0` 空包）作为**负向对照**钉进单测
（`test_rejects_the_measured_unknown_placeholder_wheel`），避免以后有人把 `pip wheel` 接回来还以为通过。

## 2. 现象：生成的 proto stub 在本机"两层都导入不了"

| 来源 | 生成方式 | 在 protobuf 4.25.7 下的实测结果 |
|---|---|---|
| `build/generated/python`（既有） | protoc 5.29.0 gencode | `ImportError: cannot import name 'runtime_version' from 'google.protobuf'` |
| `build/sdk/pystub-local`（本机重生成） | protoc 3.12.4（`libprotoc 3.12.4`） | `TypeError: Descriptors cannot be created directly … must be regenerated with protoc >= 3.19.0` |

探针：`build/iraf-24h/06/import_probe.py` → `build/iraf-24h/06/pystub-import-probe.txt`。

**根因**：仓库声明的依赖是 `protobuf>=5.29,<6`（`pyproject.toml`），而本机只有 4.25.7；
5.29 gencode 需要 5.x 运行时，3.12.4 gencode 反而更老（<3.19 的旧式 descriptor），两者在本机都不可导入。

**修法（不做假动作）**：
1. 用本机 protoc 生成**副本**（`build/sdk/pystub-local/iraf/v1/*_pb2.py`，4 个文件）证明"生成流程可复跑"；
2. **不覆盖**既有 stub——覆盖只是把一套本机不可导入的 gencode 换成另一套同样不可导入的，
   却是对声明依赖（protobuf>=5.29）的降级；
3. 在 manifest 里如实登记 `stubs.importable_locally=false` + `import_probe_reason` + `declared_protobuf`；
4. grpc 层（`*_pb2_grpc.py`）本机**无法生成**（`grpc_python_plugin` 缺失），按 `prebuilt` 登记路径与 sha256，
   不写成"已生成"。

**顺带修掉一个自身假设缺陷**：最初把"四个 proto 都应有 `_pb2_grpc.py`"写死，导致
`grpc_present=False` 的假报警。改为**从 IDL 源码实测 `service` 声明**：
`runtime.proto`(SkillRuntimeService)、`events.proto`(EventService) 才有 grpc stub，
`common.proto`/`skill.proto` 没有（`stub_status()["grpc_expected"] == ["events","runtime"]`）。

## 3. 现象：自己写的"禁止绝对路径"门禁第一次运行就把自己拦下

```
[build_sdk][错误] 预检摘要出现绝对路径（只允许仓库相对路径）
[build_sdk][错误]   - $.protoc.path 出现绝对路径：/usr/bin/protoc
→ exit=2
```

**修法**：把"写盘路径"与"日志路径"分开——写盘内容只记录
`protoc {"version": "libprotoc 3.12.4", "source": "PATH|explicit --protoc"}`，
`display_path()` 只用于终端输出；`relpath_posix()`（强制相对）只用于写产物。
该门禁保留并加了单测 `test_absolute_path_gate_rejects_home_path`。

## 4. 现象：重打包"篡改成功"但读侧内容没变

用 `tarfile` 重打包并给成员加字节时，若沿用原 `TarInfo`，`TarInfo.size` 仍是旧值：
tar 读侧按声明长度取数据，多出的字节被当成下一个头，**成员内容与原来完全一致**，
于是"篡改"在逐文件比对里不可见（第一次跑 `test_verify_detects_tampered_bundle_member_*` 时
`violations == []`）。

**修法**：重打包必须同步 `info.size`（单测与 `build/iraf-24h/06/tamper.py` 都已修正）。
这也是"负向用例必须先证明它真的能失败"的例子：一个恒不触发的负向用例等于没有用例。

## 5. 现象：跨目录/相对路径导致负向验收"报错但证明的不是那件事"

1. manifest 里存的是**仓库相对路径**。把清单与产物拷到 `build/iraf-24h/06/tamper-*/` 后，
   `--verify` 仍会去校验**原始未篡改**的 `build/sdk/...`，于是报出的"不符"来自错误的对象。
   修法：负向脚本重写 `artifacts[].path` 指向副本（`tamper.py: retarget()`）；
   `--verify` 语义保持"清单里写的路径 = 被校验的对象"，不做隐式回退。
2. `pip install --target <相对路径>` 之后在 `cwd=/tmp` 的子进程里导入 → `ModuleNotFoundError`。
   修法：安装检查脚本把 wheel/target 都 `resolve()` 成绝对路径。

## 6. 验收数字（`build/iraf-24h/06/build-sdk.txt`，可复跑）

| 项 | 实测 |
|---|---|
| `--dry-run` | `exit=0`，且**未创建** `build/sdk`（核对行："dry-run 未创建 build/sdk ✓"） |
| 完整构建 | `exit=0`；`build/sdk/iraf_sdk-0.2.0-py3-none-any.whl`（6 个成员）、`iraf-runtime-0.2.0-aarch64.tar.gz`（106 个文件） |
| manifest | `schema_version=iraf.package-manifest/v1`、`artifacts=2`、逐文件条目 **112**、`target.verified=false`、`signature.scheme=pending`、`sbom.status=placeholder`、`wheelhouse.status=pending_fetch` |
| 可复算 | 连续两次构建 SHA-256 **逐位相同：True**（gzip `mtime=0` + 成员 `mtime=0/uid=gid=0` + 路径排序 + 固定 `ZipInfo.date_time`） |
| 绝对路径 | `grep -c '/home/coretek' build/sdk/manifest.json = 0`；结构化扫描 `absolute_path_values=0` |
| 自校验 | `--verify` → `exit=0` |
| 负向 1（包内 deflate 字节被改 + 外层哈希重写） | `exit=4`，命中 `产物内文件被篡改：… :: src/iraf_adapters/mujoco/mujoco_backend.py` |
| 负向 2（翻转一字节、清单未改） | `exit=4`，同时命中"产物 SHA-256 不符"与"产物内文件被篡改" |
| 负向 3（重打包改成员内容 + 外层哈希重写） | `exit=4`，命中 `config/piper-left-contact.json` |
| 负向 4（manifest 不存在） | `exit=2`（预检失败，不是校验失败） |
| 正向（x86 侧离线安装 + 导入） | `pip install --no-index --no-deps --target` → `exit=0`；导入探针 `{"version": "0.2.0", "known_codes": 19, "heavy_loaded": []}`，安装文件 12 个 |
| 步骤单测 | `Ran 43 tests … OK`（含 6 条 wheel 负向 + 2 条篡改负向 + 各自正向对照） |

排除项：bundle 排除了 138 个非运行时文件（`__pycache__/*.pyc` 与 `src/iraf_core/profile.py.orig`
等编辑器备份），全部登记在 `manifest.bundle.excluded` 里（排除可审计，不是静默过滤）。

## 7. 未验证 / 已知边界（如实登记，不得当作通过）

1. **gzip 尾部字节翻转检测不到**：只把 gzip 尾部 ISIZE 字节翻转、同时重写 manifest 外层 sha256 时，
   `--verify` 返回 0。原因：`tarfile` 读到归档结束标记即停止，从不校验 gzip 尾部 CRC/ISIZE，
   逐文件比对看不到任何差异。**该情形只能依赖可信来源的外层 sha256**，而清单自身的可信性依赖签名
   （`signature.scheme=pending`，本轮未落地）。证据：验收 7b 段（明确标注"已知边界，不是通过"）。
2. **IDL stub 不在 runtime bundle 内**：步骤只声明 `src/skills/profiles/config/deploy` 五个目录，
   未含 `build/generated/python`。`src/iraf_adapters/grpc/**` 依赖 `iraf.v1.*`，
   因此步骤 08 的 `install.sh`/`package_board_bundle.sh` 必须显式声明 stub 安装位置，
   否则装完的 runtime 起不了 gRPC 面。**本轮不擅自扩大包内容，按缺口登记。**
3. **gRPC 层未复现**：无 `grpc_python_plugin`，`*_pb2_grpc.py` 是 prebuilt；RPC 通道在本机从未跑通。
4. **目标端一切未验证**：轮询/镜像均未在 aarch64 上执行过；板卡不在场 ⇒ 安装、`/health`、
   AgentOS 联通全部 `DEFERRED`。本记录中的"安装通过"**只**指 x86 开发端离线安装 + 导入。
5. wheelhouse 为空：真依赖清单由步骤 07 抓取后回填（manifest 的 `wheelhouse.status=pending_fetch`，
   SBOM 是占位）。

## 8. 复跑命令

```bash
# 计划（不落盘）
bash deploy/sdk/build_sdk.sh --dry-run
# 完整构建 + 自校验（PYTHON 可指定解释器；两个解释器都带 PyYAML/jsonschema）
PYTHON=/usr/bin/python3 bash deploy/sdk/build_sdk.sh --json-out build/iraf-24h/06/preflight.json
# 单步
PYTHON=/usr/bin/python3 bash deploy/sdk/build_sdk.sh --verify
/usr/bin/python3 deploy/sdk/lib_manifest.py plan --out build/sdk
# 全量验收（含 4 组负向 + 安装检查 + 步骤单测）
bash build/iraf-24h/06/run_acceptance.sh | tee build/iraf-24h/06/build-sdk.txt
# 步骤单测
PYTHONPATH=src /usr/bin/python3 -m unittest tests.unit.test_sdk_manifest -v
```
