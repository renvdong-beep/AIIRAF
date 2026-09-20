# 场景构建器注入传感器与道具（步骤 13）调试记录

- 日期：2026-09-20
- 范围：`scenes/handoff_lab` 场景包 → `build/scenes/handoff_lab/handoff_lab.xml`
- 结论适用范围：**仅 x86_64 开发端仿真**（`simulation: true`）；目标端/真机验收 DEFERRED（板卡不在场），
  本记录中的任何数字都不得表述为真机或实时能力。

## 1. 需求与做法

按 `scene.yaml` 生成可加载的 MJCF：厂商 MJCF（`vendor/unitree_go2/…/go2.xml`，锁在
`vendor/unitree_go2/source-lock.json`）只读，相机/雷达/托盘/道具/工作台/光照由场景构建器注入。

- 实现层：`src/iraf_adapters/unitree/scene_builder.py`（可导入，便于逐项断言）
- 入口层：`scripts/build_scene.py`（只做参数解析与退出码映射：0/1/2/3/4/5）
- 声明：`profiles/unitree_go2_mujoco.yaml`（机型：关节/限位/躯干 body/挂载参考系）
  + `scenes/handoff_lab/scene.yaml`（场景：地形/道具/传感器/光照），脚本内无机型专有名称。

## 2. 症状 → 证据链 → 根因 → 修法

### 2.1 `scene_check` 的模型名字表对任何合法 MJCF 恒为 0（步骤 11 遗留缺陷）

- 症状：生成模型已落盘、锚点确实存在，`scene_check.py` 仍报"生成模型里找不到传感器锚点"，
  退出码 3；报告里 `names = {body: 0, camera: 0, geom: 0, site: 0}`。
- 证据：`build/iraf-24h/13/scene-check-after.json`（修复前，`anchors[].present` 全 false）。
- 根因：`_expand_mjcf()` 只平铺**文档根的直接子元素**，而合法 MJCF 的 body/geom/site/camera
  全都嵌在 `<worldbody>` 里（camera/site 只能作为 body/worldbody 的子元素）⇒ 名字表恒为空，
  门禁表现为"永远失败"，`--model` 这条验收路径实际上无法通过。
- 修法：改为递归整棵树（`elements.extend(list(element.iter()))`），文档字符串记下实测证据。
  这是**收紧**门禁：修复后它才能真正区分"注入了"与"没注入"。
- 修复后实测：`scene_check --model …` 退出码 0；`names = {body: 19, camera: 1, geom: 7, site: 3}`，
  三个锚点 `present` 全为 true（证据 `build/iraf-24h/13/scene-check-model.json`）。

### 2.2 注入自由关节后厂商关键帧维度不符（预期坑，按既有做法修）

- 症状：`Error: keyframe 0: invalid qpos size, expected length 26`（Element name 'home', id 0）。
- 根因：厂商 `<keyframe>` 的 qpos 长度 = 19（自由基座 7 + 12 关节），注入 `box_01` 的自由关节后 nq = 26。
- 修法：摘掉 keyframe 编译一次拿真实 `nq/nu`，按维度补 0 放回；
  **新注入的自由道具必须写回声明位姿**（否则 `mj_resetDataKeyframe` 会把方块搬回世界原点）。
  厂商模型若本无 keyframe 则**不新增**（不替厂商决定初始位形）。
- 断言：`mj_resetDataKeyframe(model, data, 0)` + `mj_forward` 后 `box_01` 的 `xpos = (0.19, 0, 0.025)`，
  与 `scene.yaml` 声明一致（`tests/unit/test_scene_builder_injection.py::test_generated_model_is_loadable_and_keyframe_dimension_fixed`）。

### 2.3 声明改动引发的一处**测试环境依赖**（不是实现缺陷）

- 症状：`test_model_layer_is_pending_not_checked_when_model_absent` 在生成产物落盘后由通过变失败
  （`'checked' != 'pending_generation'`），`test_require_model_without_model_fails` 由 4 变 0。
- 根因：用例在临时副本上校验声明，但 `scene_check` 的默认模型路径按**仓库根**解析
  `model.output`，于是"本机是否跑过构建"（`build/` 是 gitignore 的证据区）决定了断言结果。
- 修法：夹具把副本的 `model.output` 指到必然不存在的 `build/scenes/handoff_lab/absent-<tmp>/model.xml`，
  让"模型尚未生成"成为确定性前提。

### 2.4 声明与实现的同文交付（步骤 13 多出的 3 个场景包路径）

- `scene.yaml`：`model.builder` 由占位改为 `scripts/build_scene.py`；本体 id `go2` → `unitree_go2`
  （与设计文档 §3 声明的本体名一致，也是 `--robot unitree_go2` 与 Profile 身份的绑定依据）；
  托盘挂载与传感器锚点同步改名。
- `baseline.yaml` / `scenario.yaml`：键与 `robot:` 同步改名（`scene_check` 强制键集合与
  `scene.robots[].id` 完全一致，改一处必须同步，否则"少写一个本体"）。
- `tests/unit/test_scene_schema.py`：改名 + "构建器已交付"这一契约变化；并把
  `test_builder_placeholder_is_reported` 改成在副本上重新写占位来验证门禁本身仍有效
  （只保留"已交付路径"一种状态会让该门禁失去触发路径）。

## 3. 为什么 `scene.robots[unitree_go2].profile` 仍是结构化占位

`scene.schema.json` 的门禁规定"已交付 profile 的本体必须至少声明一项能力"，而 Go2 的运动能力
（stand/stop/locomote）要到步骤 16/17 才逐项验收 —— 在验收前写任何能力都是**未验收能力**。
因此保持占位、把 profile 的关断点改成步骤 16，并由构建器按**声明身份**
（`profiles/*_mujoco.yaml` 中 `metadata.name == <robot>`，0 个或多个命中都显式失败）解析房源文件；
报告里的 `robot.profile_source` 记录走了哪条来源（`scene.robots[].profile` 或
`declared_identity_lookup`），**不静默**。能力验收后把占位改成路径即可，构建器无需改动。

## 4. 关键实测数字（本步骤）

| 项 | 值 | 来源 |
|---|---|---|
| 厂商模型 SHA-256 | `2014a3d7…693d4b`（= 锁内值，match=true） | `build-scene.txt` |
| 模型维度（生成后） | nq 26 / nv 24 / nu 12 / ncam 1 / nsite 3 / nbody 20 / njnt 14 | `check_report.py` |
| 注入的传感器 | `overhead_camera`（world camera）、`payload_lidar_site`（躯干 site） | 同上 |
| 厂商只读引用 | `imu`（厂商自带 site，未改动） | 同上 |
| 道具 | `box_01`（world + freejoint）、`tray_01`（挂到 `tray_frame`，z=0.057 m） | 同上 |
| `git status --short vendor/` | 0 行（厂商文件未被改写） | `acceptance.txt` A7 |
| 步骤单测 | Ran 20 / OK | `acceptance.txt` A4 |
| 场景包契约回归 | Ran 42 / OK | `acceptance.txt` A5 |

## 5. 诚实边界（不伪造）

1. 目标端/真机验收 DEFERRED（板卡不在场）；本步骤产物 100% 是仿真产物。
2. 雷达只注入"可挂载 site + 扫描契约（360 线 / 8 m）"，**没有**伪造 MuJoCo 传感器类型：
   点云与射线统计属步骤 14（用 `mujoco.mj_ray` 实测），本记录不声称已有雷达测量结果。
3. `emit_backend_config.py` 的兼容性是"键集合兼容（不抛错）"：四足场景的
   `target_id`/`gripper`/`vision` 显式为 `null` 且带 `manipulation_absent_reason`，
   不得据此装配操作后端（运行期以 Piper 抓取场景报告为准）。
4. 人形仍为静态实体占位（决策 4.B），本步骤未触碰人形资产。
