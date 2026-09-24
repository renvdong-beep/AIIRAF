# 联合模型（狗 + 臂）首次构建成功与同模型可达实测（2026-09-24）

## 0. 一句话

在**同一个 MJCF** 里装下 Go2 与本仓库的 Piper 臂：`build/scenes/handoff_lab/handoff_lab_joint.xml`
（`nbody=28`、`nq=34`、`nu=20`），并**在同一模型里实测**：臂能够到狗背上的托盘
（FK 采样 20000 次，工具点 `piper_link6` 到托盘顶面 `(0.450, 0.000, 0.346954)` 的**最小距离 = 0.038048 m**）。

这条结论取代了此前 `.hermes/plans/2026-09-23-dock-for-handoff.md` §11.1 的**假设**
（"臂基座在台面原点附近、只按位置可达筛选"）—— 现在是同一模型里的真实位姿 FK。

## 1. 产物与实测（可复跑）

```
构建：PYTHONPATH=src python3 scripts/build_scene.py --scene scenes/handoff_lab --robot unitree_go2 --attach piper
      ⇒ 退出码 0，写 model.joint_output = build/scenes/handoff_lab/handoff_lab_joint.xml
探针：PYTHONPATH=src MUJOCO_GL=glfw python3 build/iraf-a6a14/joint_reach_probe.py
      ⇒ build/iraf-a6a14/joint-reach-probe.json（4/4 判据通过）

nbody=28（狗 20 + 臂 8）｜nq=34（狗 26 + 臂 8）｜nu=20（狗 12 + 臂 8）
臂 body 全部按前缀 `piper_` 命名 ⇒ 无名字冲突
臂基座世界位置 = (0.45, −0.45, 0.123)（`piper_link1` 自身 z 偏置 0.123）
     与声明 `robots.piper.placement.pos_m = (0.45, −0.45, 0)` 的**平面误差 = 0.0 m**
托盘顶面（狗站立姿态、托盘在站位）目标 = (0.450, 0.000, 0.346954)
FK 采样最小距离 = **0.038048 m ≤ 0.05** ⇒ 够得到
```

## 2. 实现要点（`src/iraf_adapters/unitree/scene_builder.py` 的 `_attach_robots`）

用 **MuJoCo 原生 `MjSpec.attach(child, prefix, frame)`** 合成，而不是手写名字前缀重写
（后者极易漏改 mesh/material/actuator 的 joint 引用）。逐门实测记录（每步"显式失败 → 修 → 再过"）：

| 门 | 现象 | 处理 |
| --- | --- | --- |
| 身份 | 场景 id `piper` ≠ Profile 名 `piper_mujoco` | 新增 `robots[].profile_name` **显式映射**，值仍必须等于 Profile 的 `metadata.name` |
| 构建期声明 | 臂的 Profile 无 `spec.model` | 补 `{vendor, file, trunk_body}`（臂自己的产物仍走其基线） |
| 厂商锁 | 臂侧是 `iraf.piper-model-source-lock/v1`（`files[].path` 为**本机绝对路径**） | 校验器**显式支持两套 schema**，映射规则=「模型目录名 + `relative_path`」，并在报告里标 `host_specific: true` |
| 关节初值 | 抓手 `joint7/8` 不在 `spec.home` | 取 `spec.gripper.open_positions`（声明来源；缺键即显式失败，不猜） |
| 资产路径 | `to_xml()` 报 `.../piper_description/mujoco_model/../../../vendor/unitree_go2/.../base_link.STL` 不存在 | 把附加本体的 mesh/texture 拷到产物目录 `assets/<本体 id>/`；`MjsCompiler` **无 `meshdir` 可写属性** ⇒ 暂用绝对路径并登记债务 |
| 关键帧 | `keyframe 0: invalid qpos size, expected length 34` | 按「主模型关键帧 + 各附加本体的**声明**初值」**覆写**（`attach` 可能已自行扩展过 ⇒ 不能追加）；**且收集顺序必须在覆写之前**（我第一版在循环内覆写、用还没收集完的列表 ⇒ 写回 26 而非 34） |

## 3. 诚实边界（未解决项，已登记）

1. **联合产物的资产是绝对路径** ⇒ 产物 **host-specific**：`MjsSpec.compiler` 没有 `meshdir` 可写属性，
   两个本体的资产无法用单一相对 meshdir 表达。报告里 `injections.attached_robots[].assets_path_style
   = "absolute_host_path"` 如实标注。
2. **债 S4（新）**：`vendor/agilex_piper/piper_description` 是指向 `~/piper_ros/src/piper_description`
   的**符号链接** ⇒ 臂的厂商资产**并未入库**、其锁写死本机路径 ⇒ 换机/新克隆无法复现臂侧仿真。
   迁移到厂商 U1 约定（资产实体入库 + 重写锁 + 许可证 BOM）是独立工作项。
3. 本产物的**动力学**未做任何验收：只证明"装进了同一模型、位姿与声明一致、几何上够得到"。
   交接（把方块放进托盘）的真实接触/摩擦、以及 `s03_pick` 的三项判据口径，都还没做。
