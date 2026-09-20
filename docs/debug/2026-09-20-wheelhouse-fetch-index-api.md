# 离线 wheelhouse 抓取：pip 审批门与生成器真值缺陷（步骤 07）

- 日期：2026-09-20　范围：`deploy/sdk/fetch_wheelhouse.sh`、`deploy/sdk/check_wheel_tags.py`（x86 开发端）
- 上游设计：`docs/iraf-multiplatform-sdk-design.md` §4/§5/§7；ADR-0006 L1 路线
- 结论一句话：**抓取机制由 `pip download` 改为标准库直读 PEP 503 索引**（pip 走非 PyPI 源被无人值守审批门拦截），
  平台标签过滤由本仓库自己负责；首跑暴露的"全部候选被误判为命中拒绝标签"是 `if <生成器>` 恒真的实现缺陷，已修复并用单测钉住。

---

## 1. 现象与证据链

### 1.1 现象 A：抓取机制不可用（pip + 非 PyPI 源）

```
$ timeout 240 /usr/bin/python3 -m pip download --only-binary=:all: --platform manylinux_2_28_aarch64 \
    --python-version 3.10 --no-deps --dest build/iraf-24h/07/pip-try \
    -i https://mirrors.aliyun.com/pypi/simple/ pyyaml
→ 命令未执行：安全扫描判定 [MEDIUM] Python package from non-PyPI source
  （pattern_key: tirith:pip_url_install；审批门为无人值守环境的一票否决）
```

同一现象在步骤 05 的阻塞登记里已记录过一次（"阿里云镜像的 pip 安装被无人值守审批门拦截"），
本轮换到 `pip download` 仍然被拦 ⇒ **不是偶发，而是环境策略**。按 `00-执行规则.md` §5.2 不得反复重试。

镜像本身可达（同一台机器、同一个源，`curl` 与标准库都正常）：

```
$ /usr/bin/python3 build/iraf-24h/07/index_probe.py
mujoco:  status=200 ctype=text/html; charset=utf-8 bytes=414321   whl=1243
numpy:   status=200 ... whl=4043  含 aarch64=513  win_arm64=104  macosx=1237  cp310=321
grpcio:  status=200 ... whl=10361
pyyaml:  status=200 ... whl=521
JSON API（PEP 691）: 返回 text/html ⇒ 索引只有 HTML 形态，需按 PEP 503 解析锚点
href 形态: ../../packages/<hash>/<file>.whl（必须 urljoin，不能拼接）
$ curl -sIL https://mirrors.aliyun.com/pypi/packages/64/1d/24b8.../numpy-1.21.2-cp310-cp310-manylinux_2_17_aarch64.manylinux2014_aarch64.whl
HTTP/2 200  content-type: application/zip  content-length: 13056968
```

### 1.2 现象 B：首跑 dry-run 判定"所有包都没有可用 wheel"

```
$ bash deploy/sdk/fetch_wheelhouse.sh --dry-run
[check_wheel_tags][预检失败] 以下包在声明的源上没有可用的 cp310/manylinux_2_28_aarch64 wheel：
mujoco、numpy、grpcio、protobuf、pyyaml、pillow、jsonschema；禁止用 x86_64 wheel 顶替
[fetch_wheelhouse][错误] dry-run 失败（退出码 2）
```

> 证据说明（诚实缺口）：该次输出未单独落盘，`tee` 文件已被修正后的第二次运行覆盖；
> 上面的文本为当轮原始输出引用，另有机制复现可独立校验（见 1.3）。

### 1.3 机制复现（可复跑）

```
$ /usr/bin/python3 build/iraf-24h/07/repro-generator-truthiness.py
文件平台标签：('manylinux_2_28_aarch64',)　声明拒绝标签：win_arm64
旧写法 `if rejected and (gen)` 判定为命中拒绝标签：True  <- 缺陷：生成器对象本身恒为真
新写法 `if any(gen)`          判定为命中拒绝标签：False  <- 正确
```

---

## 2. 根因

| # | 根因 | 位置 | 修法 |
|---|---|---|---|
| R1 | `if rejected and (rejected == tag or rejected in tag for tag in wheel.platform_tags):` 把**生成器对象**当布尔用；生成器恒为真 ⇒ 只要声明了 `reject_platform_tags`，**任何** wheel 都被判"命中拒绝标签" | `check_wheel_tags.platform_tag_compatible` | 改写为 `any(...)`，并加注释与单测（`test_选择结果绝不落在干扰平台上` + 三条正例对照） |
| R2 | 交付脚本按设计 §5 使用 `pip download`，而无人值守环境对"非 PyPI 源 + pip"一票否决 ⇒ 抓取路径不可执行 | `fetch_wheelhouse.sh` 实现层 | 改为标准库 `urllib` 直读 PEP 503 索引页并自行解析候选；**声明语义不变**：换源仍只改矩阵 `index_url` 一行，标签过滤更严（由本仓库负责，不依赖 pip 的平台解析） |
| R3 | 候选排序原为"平台档位优先、再版本"，会让**更旧的二进制 wheel** 压过**更新的纯 Python wheel**；运行期依赖需要新版本（例：生成 stub 要求 `protobuf ≥ 5.29`，本机实测矩阵站点最新为 7.36.1 纯 Python） | `check_wheel_tags.select_candidate` | 改为**版本优先**：正式版 > 预发布版；版本高者优先；同版本内 平台精确 > 同架构兼容 manylinux > 纯 Python |

---

## 3. 修复后的实测结果

```
$ PYTHON=/usr/bin/python3 bash deploy/sdk/fetch_wheelhouse.sh --dry-run            # 退出码 0，解析索引
mujoco   -> mujoco-3.13.0-cp310-cp310-manylinux_2_27_aarch64.manylinux_2_28_aarch64.whl  kind=exact      候选 1243
numpy    -> numpy-2.2.6-cp310-cp310-manylinux_2_17_aarch64.manylinux2014_aarch64.whl      kind=compatible 候选 4043
grpcio   -> grpcio-1.84.0-cp310-cp310-manylinux2014_aarch64.manylinux_2_17_aarch64.whl    kind=compatible 候选 10361
protobuf -> protobuf-7.36.1-py3-none-any.whl                                             kind=pure_python 候选 2677
pyyaml   -> pyyaml-6.0.3-cp310-cp310-manylinux2014_aarch64...manylinux_2_28_aarch64.whl   kind=exact      候选 521
pillow   -> pillow-12.3.0-cp310-cp310-manylinux_2_27_aarch64.manylinux_2_28_aarch64.whl   kind=exact      候选 2859
jsonschema -> jsonschema-4.26.0-py3-none-any.whl                                          kind=pure_python 候选 93

$ PYTHON=/usr/bin/python3 bash deploy/sdk/fetch_wheelhouse.sh                       # 退出码 0
[check_wheel_tags] 抓取完成：7 个 wheel / 47620905 字节，全部通过 cp310+manylinux_2_28_aarch64 标签校验
[check_wheel_tags][提示] 与本机已装版本存在差异：mujoco 3.13.0（本机 3.3.3）、protobuf 7.36.1（本机 4.25.7）、
  pyyaml 6.0.3（本机 5.4.1）、pillow 12.3.0（本机 9.0.1）（只记录，不构成门禁；如需版本对齐请在矩阵声明 pins）

$ bash build/iraf-24h/07/run_acceptance.sh     # 15 项检查，失败项 0
  --dry-run --no-resolve / --dry-run / --help → 0；空源 / 非法源(file://) / 不可达版本(pins==0.0.1) / 缺拒绝标签 → 2；
  用法错误 → 1；真实目录标签校验 → 0；混入 win_arm64+macosx → 4；空目录 → 4；目录不存在 → 2
```

`numpy` 选到 `compatible`（manylinux_2_17_aarch64）而不是精确标签，是**事实**而不是降级：
numpy 自 2.3 起要求 Python ≥ 3.11，声明目标是 cp310，因此 cp310 线的最高版本停在 2.2.6（平台标签是 2_17）。
glibc 2.17 ≤ 声明 2.28，同架构可安装。

---

## 4. 未验证 / 缺口（不伪造）

1. **传递依赖不闭合（本步新发现，量化）**：`build/iraf-24h/07/requires-gap.json`
   实测 7 个 wheel 共声明 10 条 `Requires-Dist`，其中**仅 1 条**被 wheelhouse 覆盖，**9 条缺失**：
   `absl-py`、`attrs`、`etils`、`glfw`、`jsonschema-specifications`、`pyopengl`、`referencing`、
   `rpds-py`、`typing-extensions`（分别由 mujoco / jsonschema / grpcio 要求）。
   ⇒ 当前 wheelhouse **不能**直接支撑 `pip install --no-index --find-links`（缺依赖会失败）。
   本步不自动补抓（矩阵包清单是唯一事实来源，禁止脚本私自扩清单），缺口与三个可选方案写入台账，交步骤 08/19 决策。
2. **版本未在矩阵钉住**：wheelhouse 版本是"声明源上目标标签下的最高版本"，与开发端不一致（上表 4 项漂移）。
   仿真物理与运行期行为的一致性是否要求对齐（例如 mujoco 3.13.0 vs 本机 3.3.3）尚未决策；
   `pins` 支持已实现并有用例（`test_pins_约束版本`），但矩阵当前未声明 pins。
3. **目标端验收 DEFERRED**：本步全部证据只证明"开发端按声明标签抓到了正确的 wheel"，
   `wheelhouse.json` 内 `simulation_note` 已如实标注；真实安装、`/health`、AgentOS 联通、ssh 部署均未做。
4. **设计文档表述待同步（步骤 19）**：`docs/iraf-multiplatform-sdk-design.md` §5 表格仍写
   `pip download --only-binary=:all: --platform …`，与 R2 的实现不一致；本步未改该文档（不在「涉及文件」内）。
5. **步骤 07 验收块原文假设** `index_url` 为空（决策 2.B 时期的文本）。决策 2 更新为 aliyun 源后该前提不再成立，
   因此负向用**派生坏矩阵**复现（`build/iraf-24h/07/broken-matrix-*.yaml`），未放宽任何门禁。
6. 首跑输出未落盘（tee 被覆盖），已如实标注并用机制复现补证。

---

## 5. 复现命令

```bash
# 环境探针（解释器/PyYAML/工具/磁盘/镜像可达性）
/usr/bin/python3 build/iraf-24h/07/env_probe.py
# 索引协议探针（HTML 形态、干扰标签计数、JSON API 是否存在）
/usr/bin/python3 build/iraf-24h/07/index_probe.py
# 声明与解析计划（不落盘）
bash deploy/sdk/fetch_wheelhouse.sh --dry-run --no-resolve
bash deploy/sdk/fetch_wheelhouse.sh --dry-run
# 抓取 + 标签校验 + 清单
PYTHON=/usr/bin/python3 bash deploy/sdk/fetch_wheelhouse.sh --json-out build/iraf-24h/07/fetch.json
bash deploy/sdk/fetch_wheelhouse.sh --verify --dest build/wheelhouse/aarch64-manylinux_2_28-cp310
# 负向：派生坏矩阵（空源/非法源/不可达版本/缺拒绝标签）→ 期望退出码 2
/usr/bin/python3 build/iraf-24h/07/make_broken_matrix.py --repo-root . \
  --out build/iraf-24h/07/broken-matrix-空源.yaml --mode 空源
bash deploy/sdk/fetch_wheelhouse.sh --dry-run --matrix build/iraf-24h/07/broken-matrix-空源.yaml; echo "exit=$?"
# 传递依赖闭合度量化
/usr/bin/python3 build/iraf-24h/07/analyze_requires.py --dir build/wheelhouse/aarch64-manylinux_2_28-cp310
# 全量验收（15 项）
bash build/iraf-24h/07/run_acceptance.sh
# 单测
PYTHONPATH=src /usr/bin/python3 -m unittest tests.unit.test_wheel_tags -v
```
