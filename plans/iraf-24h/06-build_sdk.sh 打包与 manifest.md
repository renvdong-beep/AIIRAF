# 06 build_sdk.sh 打包与 manifest

- 状态：TODO　　预估：2 个 tick　　归属：本窗口（SDK/宇树线）
- 决策依据：需求 1 设计 §4/§5；决策 1.A、3.A

## 目标
从矩阵声明出发产出 SDK wheel、runtime bundle、manifest.json 与校验和，全部可复算。

## 前置
步骤 02、05 完成。

## 涉及文件（提交时只 add 这些路径）
- 新增：`deploy/sdk/build_sdk.sh`、`deploy/sdk/lib_manifest.py`
- 新增：`tests/unit/test_sdk_manifest.py`

## 步骤
1. 脚本骨架：`set -euo pipefail`、`--help`、`--dry-run`、退出码约定（1 参数/2 预检/3 构建/4 校验）。
2. 生成 proto stub（用已有 `build/generated/python` 的既有流程；**不**在本步骤改 protoc 版本）。
3. 构建 `iraf-sdk-<ver>-py3-none-any.whl`（`python3 -m build` 不可用则用 `python3 -m wheel`/`setup.py bdist_wheel` 的既有可用路径，实测后记录）。
4. 组装 `iraf-runtime-<ver>-<arch>.tar.gz`（`src/` + `skills/` + `profiles/` + `config/` + `deploy/`）。
5. 写 `manifest.json`（schema `iraf.package-manifest/v1`）：版本、IDL 版本、git commit、`dirty`、target 标签、每个文件 SHA-256、SBOM 占位、签名占位。
6. 负向：篡改 bundle 内一个字节 → `build_sdk.sh --verify` 必须退出 4。

## 验收（必须可复跑，以数字为准）
```
bash deploy/sdk/build_sdk.sh --dry-run; echo "exit=$?"        # 期望 exit=0
bash deploy/sdk/build_sdk.sh; echo "exit=$?"                  # 期望 exit=0
python3 -c "import json,sys;d=json.load(open('build/sdk/manifest.json'));print(d['schema_version'],len(d['artifacts']))"
PYTHONPATH=src python3 -m unittest tests.unit.test_sdk_manifest -v   # 期望 全部通过
```
产出物不得含本机绝对路径（`grep -c "/home/coretek" build/sdk/manifest.json` 期望 0）。

## 证据落盘
`build/iraf-24h/06/build-sdk.txt`

## 提交信息
`feat: 新增 SDK 打包入口与产物 manifest`

## 失败 / 阻塞处理
若 `build`/`wheel` 模块缺失且无法离线安装，改为 `zip`+`dist-info` 手工组 wheel 并在调试记录写明差异。
