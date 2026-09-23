# Go2 交互窗口"看不到机器人/只有红色方块"的定位（显示层，2026-09-23）

- 触发：使用者反馈「窗口没有 Go2 的显示；视角也不能旋转」，并补充关键线索
  「**昨天调试 go2 原地踏步都可以正常显示**，为什么今天不正常了」。
- 结论：两条**独立**缺陷叠加，第二条为主因。
  1. **相机模式**（我 2026-09-23 引入）：把窗口相机从"声明的固定相机"改成"自由相机"，
     而原实现每帧把 `viewer.cam.type` 设成 `mjCAMERA_FIXED` ⇒ 按 MuJoCo 语义**禁用鼠标旋转**
     （使用者"转不动"的机制即此）。已改为：默认回到固定相机（昨天可用那套），`--free-camera` 才用自由相机。
  2. **GL 路径（主因，环境级）**：本机默认 GL 路径下**窗口的 3D 视口几乎不亮**，
     而**离屏渲染（`mujoco.Renderer`）一直正常**。用最小场景（浅灰地板 + 绿盒 + 红球，
     与 Go2 场景无关）二分确认是**环境级**而非场景级：

     | 窗口相机 | GL 路径 | 视口亮度 mean | 非黑(>40) |
     |---|---|---|---|
     | 声明固定相机 `overhead_camera` | 默认 | 22.2 | 29.7% |
     | 声明固定相机 `overhead_camera` | **软件（llvmpipe）** | **95.1** | **68.3%** ← 采用 |
     | 自由相机（profile camera 段） | 默认 | 36.4 | 32.6% |
     | 自由相机（profile camera 段） | 软件 | 42.5 | 37.4% |
     | 最小场景（floor+box+ball） | 默认 | 3.8 | 3.2% |
     | 最小场景（floor+box+ball） | 软件 | **72.5** | **48.9%** |

     ⇒ 修复：`config/go2_loopback.yaml` 的 `render.software_gl: true`（声明开关，可关）
     ⇒ `scripts/view_go2_gait.py` 在**创建 GL 上下文之前**设 `LIBGL_ALWAYS_SOFTWARE=1` +
     `GALLIUM_DRIVER=llvmpipe`，并打 `VIEWER_GL software|default` 日志标记。
     ⚠ 必须在 `import mujoco.viewer` **之前**设，否则上下文已建、改环境无效。

## 诊断口径（本轮建立，避免下次再靠肉眼争论）

- **窗口在屏幕上的位置必须用 `xwininfo -id <id> -stats` 读 "Absolute upper-left"**：
  `xwininfo -root -tree` 行尾两个数字是「相对父窗口 + 绝对」，我按前者量过 ⇒ 量错位置、
  得出过"视口全黑"的错误结论（同一现象当时其实在另一个坐标系下的窗口里）。
- **同屏可能有多个 MuJoCo 窗口**（多次运行未清理 ⇒ 相互遮挡，且窗口会被 WM 级联到不同位置）。
  测量前先按 PID 清理（**不要用 `pkill -f "脚本名"`：当前 shell 的命令行里就含该字符串，会自杀**，
  本项目已两次踩到）。清理方式：`ps -o pid,cmd -C python3 | awk '/视图脚本名/ {print $1}' | xargs -r kill`。
- **窗口内容用"视口亮度 mean + 非黑像素占比 + 粗彩图"量化**（视口取窗口内 x340..1000、y60..680，
  避开左右 UI 面板）；3D 正常 vs 全黑在这两个数上差异是数量级（95.1/68.3% vs 3.8/3.2%）。
- 机器人"在不在画面里"用**分割渲染**数像素：`enable_segmentation_rendering()` +
  `geom_bodyid ∈ trunk 子树` 统计占比与 bbox（不是靠肉眼）。同类判据还包括：
  **躯干像素 > 托盘像素**（狗背上的 `tray_01` 会挡住躯干 —— 首版自由相机 el=−35° 就选了这种"高覆盖但主体是托盘"的取景）。

## 仍未解释（诚实标注，不装作已闭环）

- **为什么默认 GL 路径今天坏了**：未定位到具体原因（硬件驱动路径/会话状态/远程桌面栈均可能）。
  本轮的处置是**声明式绕过**（可一键关回默认 GL），不是修掉驱动。
  若后续在其它机型/会话复现，按上表复测该四格矩阵即可判定是环境还是场景。
- 本次"昨天正常、今天不正常"的时间线证据：昨天窗口走的是"每帧固定 `overhead_camera`"的旧路径；
  今天先被我改成自由相机（进一步变暗），再叠加默认 GL 的渲染异常 ⇒ 两条叠加才呈现"只有一个红色方块"。

## 交付物（本轮，供使用者直接查看）

- `build/iraf-a6a4/go2-gait-offscreen.mp4`：**离屏渲染**的真实 wave 步态 4.0 s（1280×720，100 帧，
  帧率 25 fps，路径与窗口 GL 无关，因此必然可见）；关键帧 `build/iraf-a6a4/keyframe-gait.png`。
- `build/iraf-a6a4/window-final-visible.png`：修复后窗口实拍（视口亮度 95.1、非黑 68.3%）。
- 复跑命令：
  `DISPLAY=:0 XAUTHORITY=/run/user/1000/gdm/Xauthority MUJOCO_GL=glfw PYTHONPATH=src /usr/bin/python3 scripts/view_go2_gait.py --config config/go2_loopback.yaml --seconds 300`
  （默认固定相机 + 软件渲染；要旋转加 `--free-camera`）
