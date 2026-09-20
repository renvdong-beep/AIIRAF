# 2026-09-20 Step B：统一入口（金路径）

计划：`.hermes/plans/2026-09-20_123000-generality-roadmap.md`
关联：AGENTS.md 6.4（对常见场景提供金路径）、6.3（新机型接入不需要复制核心代码）。

## 1. 问题

接入方式按机型分叉，使用者要"挑脚本"：

```
scripts/build_piper_baseline.py      / scripts/build_ur5_baseline.py
scripts/verify_piper_pick.py         / scripts/verify_ur5_pick.py
```

两者各自解析一套参数、各自组装 SkillRuntime 请求、各自写一种报告 schema
（`iraf.piper-pick-acceptance/v1` / `iraf.ur5-pick-acceptance/v1`）。
结果：文档、CI、调用方都得知道"这台机器人该用哪个脚本"。

## 2. 改法

新增两个入口，**实现按配置分派**（配置是唯一真源）：

```
PYTHONPATH=src python3 scripts/build_baseline.py --baseline <config>.yaml
PYTHONPATH=src python3 scripts/verify_pick.py    --baseline <config>.yaml [--rebuild]
```

两个基线配置新增 `build` 段：

```yaml
build:
  baseline_module: build_ur5_baseline        # 提供 build(root, baseline_path, scene_path, calibration_path)
  scene: build/models/ur5-pick-scene.xml
  pose_evidence: build/calibration/ur5-baseline-pose.json
  profile: profiles/ur5_mujoco.yaml
  safety_policy: profiles/safety/simulation_lab.yaml
```

- `build_baseline.py`：读配置 → 导入 `build.baseline_module` → 校验其提供 `build()` →
  调用 → 打印可审计摘要（产物路径、目标 id、模型来源与哈希、姿态证据路径）。
  缺声明即显式失败，**不猜机型**。
- `verify_pick.py`：统一验收壳。profile / safety / 场景 / 时长 / 目标 id 全部来自声明
  （CLI 可覆盖）；`--rebuild` 先按声明的构建器重建场景，因此"构建 → 验收"是一条命令。
  判据仍全部取自 Backend 回传的证据（命中目标 / 双侧接触 / 抬升位移 / 位置误差），
  脚本不自行判定成功。报告 schema 统一为 `iraf.pick-acceptance/v1`。
- `verify_ur5_pick.py` / `verify_piper_pick.py` 降级为**薄包装**：
  打印 deprecated 提示后转发参数（默认值保持不变：UR5 `--output build/acceptance/ur5-pick`、
  Piper `--output build/acceptance/piper-pick` 与 `--duration-ms 2500`），
  既有文档与调用方不会立刻失效。

## 3. 验收（两台机型数值必须复现）

```
scripts/verify_pick.py --baseline config/ur5_simulation_baseline.yaml
  SUCCEEDED  力 17.451911/16.053349N  抬升 0.041208m  位置偏差 0.00011876256957761978  grip_region
scripts/verify_pick.py --baseline config/piper_simulation_baseline.yaml --rebuild
  SUCCEEDED  力 0.240676/0.244112N   抬升 0.087184m  位置偏差 0.002826042685491991   finger_pair
包装脚本（verify_ur5_pick.py / verify_piper_pick.py）转发后数值与上表逐位相同
scripts/build_baseline.py 对两个配置均 BUILT（UR5 摘要含模型 sha256）
单元测试 258 项，失败项仍为既有 1 项 + 4 项导入错误
```

## 4. 遗留

- 视觉验收（`verify_piper_visual_pick.py`）与多目标验收
  （`verify_piper_multi_target_pick.py`）仍是机型脚本，未纳入统一入口；
  它们与 `verify_pick.py` 的差别在于"目标位姿来自视觉/多目标场景"，
  下一步可以给 `verify_pick.py` 增加 `--skill visual_pick --target-id <id>` 覆盖。
- 两个场景生成器（`build_*_pick_scene.py`）仍是内部实现细节，未合并；
  合并的前提是 Piper 场景 XML 逐位不变（当前冒烟验收依赖该 XML 的既有形态）。
- 旧机型脚本的删除计划：待所有调用方（文档、CI、随机抓取脚本）迁移后单独一步做，
  并保留一个 release 周期的过渡期。
