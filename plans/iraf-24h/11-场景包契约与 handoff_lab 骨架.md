# 11 场景包契约与 handoff_lab 骨架

- 状态：TODO　　预估：2 个 tick　　归属：本窗口（SDK/宇树线）
- 决策依据：需求 2 设计 §3；决策 4.B、5.A、6.A

## 目标
建立 `scenes/` 场景包契约与第一个场景骨架，使场景成为声明而不是代码。

## 前置
步骤 01 完成。

## 涉及文件（提交时只 add 这些路径）
- 新增：`scenes/handoff_lab/scene.yaml`、`scenes/handoff_lab/baseline.yaml`、`scenes/handoff_lab/scenario.yaml`、`scenes/handoff_lab/README.md`
- 新增：`config/scene.schema.json`
- 新增：`scripts/scene_check.py`、`tests/unit/test_scene_schema.py`

## 步骤
1. 按设计 §3 定 schema：本体、地形、道具、传感器（相机/雷达/IMU）、光照、随机种子、`simulation: true`。
2. `scene.yaml`：先声明 Piper（既有能力）+ Go2（模型已探测，能力待验收）+ 人形仅作静态道具（标注「仅模型，不代表运动能力」）。
3. `scenario.yaml`：S2 步骤序列（对象、动作、判据、故障注入项），本步骤只写声明与校验，不实现执行器。
4. `scene_check.py`：schema + 引用完整性（本体 profile 存在、道具几何存在、传感器引用的 geom/site 在生成后模型里存在）；缺字段即失败退出 2。
5. 负向单测：缺 `simulation`、本体 profile 不存在、光照为空各一条。

## 验收（必须可复跑，以数字为准）
```
PYTHONPATH=src python3 scripts/scene_check.py --scene scenes/handoff_lab; echo "exit=$?"   # 期望 exit=0
PYTHONPATH=src python3 -m unittest tests.unit.test_scene_schema -v                        # 期望 全部通过（含 3 负向）
```

## 证据落盘
`build/iraf-24h/11/scene-check.txt`

## 提交信息
`feat: 新增场景包契约与 handoff_lab 骨架`

## 失败 / 阻塞处理
若与 `config/*.yaml` 既有字段语义冲突，复用既有字段名而不是新造同义字段，并在 00-日志.md 记录映射。
