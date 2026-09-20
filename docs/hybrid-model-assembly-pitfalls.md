# 异构构型拼装排错手册（UR5e + Robotiq 2F-85 实例）

本文档固化一次真实的排错过程。最终根因与**最初的三次误判**都记录在案，
因为"误判长什么样"比"答案是什么"更有复用价值。

适用场景：把 Menagerie（或任何来源）的两个独立 MJCF 模型拼成一个时，
出现**"没有编译报错、但物理完全不对"**的症状。这类问题最难查，
因为 MuJoCo 对很多错误是**静默**的。

---

## 1. 症状与结论速查

| 症状 | 根因 | 修法 |
| --- | --- | --- |
| 夹爪 tendon 完全驱不动，`driver` 停在限位外 | 网格单位错误 → 质量放大 1e9 倍 | 每个 `<mesh>` 元素**显式**写 `scale` |
| 夹爪基座形状/质量变成机械臂的 | 两模型有同名网格（如 `base.stl`） | mesh 的 `file` **和 `name`** 都加前缀并复制文件 |
| 编译报 `mesh 'X' not found in geom N` | 只改了 mesh 资源名，没改 geom 的 `mesh=` 引用 | 同步改写 `geom/mesh=` |
| 无 ctrl 时 `QACC` 出现 Nan/Inf | 臂关节 `damping=0`，模型不耗散 | 给臂关节补小阻尼 |

---

## 2. 本次的完整证据链

### 2.1 官方模型（单独加载）完全正常

动态 `mj_step` 扫描 tendon 执行器 `ctrl`：

```
ctrl=  0  gap 0.085400 -> 0.085196   driver=[0.0023]  follower=[-0.0027]
ctrl= 90  gap 0.085400 -> 0.057005   driver=[0.2845]  follower=[-0.2724]
ctrl=180  gap 0.085400 -> 0.025085   driver=[0.5666]  follower=[-0.5438]
ctrl=255  gap 0.085400 -> 0.000401   driver=[0.7810]  follower=[-0.7557]
行程 84.795mm，严格单调，driver 全程在 range [0, 0.8] 内
```

### 2.2 组装后（修复前）完全失效

```
ctrl=  0  gap 0.085400 -> 0.103714   driver=[-1.3022]  follower=[+0.8408]   ← 符号翻转
ctrl=255  gap 0.085400 -> 0.085400   driver=[-0.1144]  follower=[+0.0256]   ← 回到原位
行程 19.1mm，非单调，driver 跑到 range 外（-1.32 < 0）
```

### 2.3 决定性证据：逐项物理量对比

```
[bodies] base_mount
   official: mass=0.150003 kg
   assembly: mass=150002855.16 kg     ← 放大约 1e9 倍
[bodies] silicone_pad
   official: mass=0.001303 kg
   assembly: mass=1302811.84 kg       ← 同上
[meshes] base_mount
   official: span=[0.014109, 0.075, 0.075038] m      ← 米
   assembly: span=[14.109529, 75.000328, 75.03775]   ← 毫米，放大 1000 倍
[meshes] base
   official: verts=17120  span=[0.075, 0.084792, 0.094146]
   assembly: verts=17120  span=[75.0, 84.79213, 94.145767]  ← 同一份顶点，单位错
```

`mass ∝ volume ∝ span³`，`span` 放大 1000 倍 → 质量放大 `1e9`。
`1e9` 倍惯量把夹爪的 4 杆机构彻底锁死：`equality` 约束形同虚设，
`driver` 被反向拖到限位外，`tendon` 的 ±5N 相对 `1.5e8 kg` 的惯量
完全无意义。

---

## 3. 三个坑的细节

### 坑 A：同名网格文件被静默复用

UR5e 与 2F-85 **都有** `base.stl`（甚至 `base_0.obj` / `base_1.obj` 这类
OBJ 拆分文件也可能撞名）。

只给 body/joint 改名而不管网格时，两个模型会加载**同一份** `base.stl`。
本次实测表现：夹爪基座的网格顶点数从 `10899` 变成 `6532`
（`6532` 是 UR5e 的 `base_0.obj`），即夹爪基座变成了机械臂基座的一部分。

修法必须做三件事，缺一不可：

1. 网格 `file` 加前缀：`base.stl` → `rq2f85_base.stl`
2. 网格 `name` 也加前缀。MuJoCo 里 `<mesh>` 的 `name` 缺省等于 `file`，
   而 `<geom mesh="...">` 解引用的是 **name**。只改 `file` 不改 `name`，
   编译报 `mesh 'base_mount' not found in geom 29`
3. 磁盘文件按前缀各存一份，并把 `geom/mesh=` 引用同步改写到新名字

### 坑 B：mesh 单位缩放不能依赖 default 类继承

官方 2F-85 把缩放写在 default 类里：

```xml
<default class="2f85">
  <mesh scale="0.001 0.001 0.001"/>   <!-- 毫米 → 米 -->
</default>
```

这个 default 子树搬进 UR5e 的 default 树后，会被 UR5e 自己的顶层
`class="ur5e"` 的 mesh 默认值（未写 `scale`，等价于 `1.0`）覆盖，
**缩放从此失效**，网格保持 STL 原始毫米单位。

这类覆盖是静默的：没有任何警告，只是物理量悄悄错了三个数量级。

修法：在每个 `<mesh>` 元素上**显式**写 `scale`，元素属性优先级最高，
不受类继承影响。

### 坑 C：glTF OBJ 拆分文件的隐含依赖

Menagerie 的 UR5e 用 `base_0.obj` / `base_1.obj` 这类拆分网格。
把它们搬进统一 assets 目录时，只要 `file` 名保持不变就不会出问题；
但**一旦决定加前缀，必须把同一 stem 的所有零件一起改**
（`base_0` / `base_1` 不能只改一个）。

本次踩到的具体报错是 `mesh 'base_mount' not found in geom 29`，
根因在坑 A 的第 2 点（`name` 未改），但这个错误信息本身很容易
被误解成"网格文件丢了"。

---

## 4. 防复发：装配期物理自检门禁

根因能跑到抓取验收才暴露，是因为**没有任何一步校验物理量的量级**。
现在装配脚本在编译后立即断言：

```
夹爪网格 max span  <= 0.2 m      （官方最大约 0.095m）
夹爪 body max mass <= 5.0 kg     （官方最大约 0.777kg）
```

不通过直接抛异常并给出"疑似 mesh scale 未生效"的提示，绝不产出模型。

配套自检（同一脚本内）：

- 夹爪网格文件集合与机械臂网格文件集合**不得有交集**
- 带前缀的夹爪网格数 / body 数必须非零（防止前缀改写逻辑失效）

---

## 5. 排错方法论（可复用到任何夹爪）

按代价从低到高，**不要跳步**：

1. **单模型对照**：把源夹爪模型**单独**加载跑一次，确认它自己是对的。
   本次正是这一步把"官方模型有问题"的假设直接否掉。
2. **纯运动学敏感性**：逐关节写 `qpos` 后只做 `mj_forward`，看目标
   （如 pad 间隙）是否变化。列出"能影响 / 不影响"的关节清单。
   注意：`mj_forward` **不求解 equality 约束的约束力**，
   所以 driver 这类"靠 equality 传递"的关节在这一步必然显示"不影响"，
   这不代表它坏了。
3. **动态对照**：用 `mj_step` 跑执行器，官方 vs 组装逐点比。
   这是唯一能验证 equality 链路的测试。
4. **逐项物理量 diff**：打印两侧每个 body 的 `mass` / `inertia` /
   `pos`、每个 mesh 的编译后顶点包围盒、每个 actuator 的
   `gainprm` / `biasprm` / `forcerange`。
   **量级差异（1e3、1e9）几乎总是单位或缩放问题**，形状差异才是姿态问题。
5. **只在以上都干净后才怀疑参数**：本次的 `forcerange=±5N` 一开始被
   误判为"太弱"，实际官方在该力限下能完成 84.8mm 全行程夹合。
   **在没有干净对照之前，不要改官方参数** —— 改了会把真正的 bug 掩盖掉。

### 三次误判的教训

| 误判 | 表面证据 | 为什么错 |
| --- | --- | --- |
| "forcerange ±5N 太弱" | 提高力限后 pad 间距从 0.9mm 变成 19mm | 19mm 仍远小于 84.8mm，只是"更离谱"；因变量没打到根因上 |
| "equality 引用失效" | driver 跑到限位外、assembly `neq=3` | 引用全部有效（`neq=3`，obj 名字都解析到了） |
| "法兰 180° 旋转污染参考系" | `follower` 符号翻转（-0.76 → +0.84） | 换单位四元数后症状不变；符号翻转是**质量爆炸**的后果，不是原因 |

共同特征：**都在"改了某个东西、症状变了"之后就下结论**，
没有做"官方单一变量对照"。而符号翻转、行程变小这类现象，
既可能是约束错乱、也可能是惯量错乱 —— 只有逐项物理量 diff 能区分。

---

## 6. 复现命令

```bash
cd ~/AIIRAF
# 1) 装配（内含物理自检门禁，不通过直接失败）
python3 scripts/assemble_ur5e_2f85.py

# 2) 官方单模型 vs 组装：动态对照（验证 tendon/equality 链路）
python3 scripts/probe_ur5_gripper_dynamic.py

# 3) 逐项物理量 diff（定位量级/单位错误）
python3 scripts/probe_gripper_diff.py

# 4) mesh 单位与缩放溯源
python3 scripts/probe_mass_scale.py

# 5) 纯运动学敏感性矩阵（哪些关节真正驱动末端）
python3 scripts/probe_ur5_gripper_sensitivity.py
```

期望结果（修复后）：

```
官方   行程 0.084795 m
组装   行程 0.084729 m      偏差 0.5%
driver 全程落在 [0, 0.8] 内，无 QACC 发散告警
夹爪网格 max span 0.094146 m，body max mass 0.777441 kg
```
