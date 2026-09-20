# 07 离线 wheelhouse 抓取

- 状态：TODO　　预估：2 个 tick　　归属：本窗口（SDK/宇树线）
- 决策依据：需求 1 设计 §2/§5；决策 2.B

## 目标
按声明标签抓取目标端依赖的离线 wheel，并拒绝错误平台标签。

## 前置
步骤 02 完成；`index_url` 已按决策 2（更新版）声明为 aliyun 镜像（实测可达且含 aarch64 wheel）。

## 涉及文件（提交时只 add 这些路径）
- 新增：`deploy/sdk/fetch_wheelhouse.sh`、`deploy/sdk/check_wheel_tags.py`
- 新增：`tests/unit/test_wheel_tags.py`

## 步骤
1. 脚本从矩阵读 `index_url`、`python_tag`、`platform_tag`、`wheels` 清单；`index_url` 为空即退出 2 并提示「请在 package_matrix.yaml 声明内网私有源（决策 2.B）」。
2. 用 `python3 -m pip download --only-binary=:all: --platform <tag> --python-version <ver> --no-deps --dest <dir>` 抓取（pip 版本能力先实测）。
3. `check_wheel_tags.py` 逐个校验 wheel 文件名标签：非目标平台（如 `win_arm64`、`macosx`）必须报错并列出违规文件。
4. 负向单测：喂入 `numpy-…-win_arm64.whl` 与 `mujoco-…-manylinux_2_28_aarch64.whl`，前者必须失败、后者必须通过。
5. 若源地址缺失：抓取部分标 `BLOCKED`，但负向单测与标签校验必须完成并通过。

## 验收（必须可复跑，以数字为准）
```
PYTHONPATH=src python3 -m unittest tests.unit.test_wheel_tags -v   # 期望 全部通过
bash deploy/sdk/fetch_wheelhouse.sh --dry-run; echo "exit=$?"       # index_url 为空时期望 exit=2
```

## 证据落盘
`build/iraf-24h/07/wheelhouse.txt`

## 提交信息
`feat: 新增离线 wheelhouse 抓取与平台标签校验`

## 失败 / 阻塞处理
下载 aarch64 wheel 不需要板卡（纯网络行为），因此本步可完整执行；若某包在声明的源上缺 aarch64 wheel，记录缺失的包名与版本并以退出码 2 失败，**不得**用 x86_64 wheel 顶替。
