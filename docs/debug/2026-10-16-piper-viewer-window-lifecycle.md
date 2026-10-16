# 2026-10-16 Piper 抓取在本机 x86 桌面的窗口生命周期（实测：能看见 / live 路径静默成功）

- 目的：确认"本机 x86 桌面上 MuJoCo 是否有显示"，并说清两条显示路径的真实行为
- 环境（实测）：Ubuntu 22.04.5 / Python 3.10.12 / mujoco **3.3.3** / GLFW 3.4.0
  桌面会话：`coretek` 于 **`:0`**（Xfce + `ToDesk` 远控；`who` 显示 `coretek :0 2026-08-31`）
  本终端自身 `DISPLAY=localhost:10.0`（SSH 转发）⇒ 必须显式 `DISPLAY=:0 XAUTHORITY=/run/user/1000/gdm/Xauthority`
  渲染：`MUJOCO_GL=glfw`，另加 `LIBGL_ALWAYS_SOFTWARE=1`（桌面为远控/无独显场景，软件渲染可用）
- 入口：`scripts/view_mujoco.py --baseline config/piper_simulation_baseline.yaml [--live] [--seconds N]`
  链路：TaskFlow → SkillRuntime → Policy/Authority → Provider(MuJoCo) → 证据（不经任何旁路）

## 1. 结论（两条路径的窗口生命周期，实测）

| 路径 | 抓取耗时 | 开窗时机 | 窗口 | 日志尾 | 退出码 |
|---|---|---|---|---|---|
| `--seconds 120`（默认静态） | 约 2 秒（`realtime=False`） | 抓取完成后立即 | **建窗并持窗 120 秒**，`Map State: IsViewable` | `VIEWER_OK` | 0 |
| `--live`（软件渲染亦同） | 约 **92 秒**（`realtime=True`） | **抓取完成后**才开窗 | 已建窗但**瞬关**（无留存） | `VIEWER_OK` | 0 |

- 静态路径**可确认看见**：窗口 `WM_CLASS = "MuJoCo"`，几何 **1280x720 @ (5,355)**，`IsViewable`；
  对窗口抓屏的像素统计：均值亮度 **54.5**、唯一色 **3522**、中央区（机械臂+工作台）唯一色 **2784**
  ⇒ **非黑屏**，是真实 3D 渲染。
- 静态路径的**完整生命周期**：持窗满 120 秒后窗口正常关闭、进程 `exit=0`（无残留窗口/进程）。

## 2. 为什么 `--live` 前一分半看不到窗口（设计事实，非故障）

`src/iraf_adapters/mujoco/viewer_runner.py` 的 live 路径是**两阶段串行**（代码注释写明依据）：
```
阶段一：抓取在独立线程完成，**不启动任何 viewer**。
  依据：mujoco.viewer.launch_passive 会创建自己的 GLFW 渲染线程，该线程任意时刻调用
        mjv_updateScene（内部 mj_copyDataVisual）读取 MjData；即使持锁也无法阻止与抓取线程的
        mj_step/mj_forward 交错 —— 实测三次中两次报 "attempting to copy mjData while stack is in use"。
  ⇒ 窗口必须与抓取串行化：先完成抓取，再开窗展示结果。
阶段二：launch_passive 打开窗口，主循环 while viewer.is_running(): step(); viewer.sync()
  若给了 --seconds 则在到点后 break，否则由窗口关闭事件结束循环。
```
`--live` 声明 `realtime=True`（`viewer.live_realtime`），pick_object 需约 **92 秒**（脚本内注释同值）
⇒ **前 ~92 秒桌面上必然没有窗口**。本次我第一轮就是在这个窗口期之外观察，差点误判为"MuJoCo 起不来"。

## 3. 缺陷（本次新发现，**未修**，需另行授权）

`--live` 路径下窗口**瞬关**（`viewer.is_running()` 一进循环即为假）时：
```
日志：VIEWER_OK
退出码：0
X 客户端列表：整轮未出现 Python（说明窗口未真正建立/未维持）
```
⇒ 这属于**静默成功**，与本项目"禁止伪造成功 / 未取得证据不得判成功"的铁律冲突
（`AGENTS.md` 铁律 1.5、1.6；同类教训见 §11.88 与 `docs/debug/2026-09-24-*`）。
建议修法（待授权，属小改动）：
```
在 run_live 的持窗阶段加判据：窗口至少存活 X 秒 且 至少渲染 N 帧（viewer.sync() 计数），
否则显式告警（stderr + 非零退出码或明确的 VIEWER_DEGRADED 标记），不得只打 VIEWER_OK。
拒绝路径用例：构造"窗口立即关闭"的桩 → 断言报错且退出码非 0。
```

## 4. 观测方法（本机可用；供下次排障复用）

```
# 窗口是否存在/名字/几何（_NET_CLIENT_LIST 是唯一可靠来源）
DISPLAY=:0 XAUTHORITY=/run/user/1000/gdm/Xauthority xprop -root _NET_CLIENT_LIST
DISPLAY=:0 XAUTHORITY=/run/user/1000/gdm/Xauthority xwininfo -id <id> | grep -E 'Absolute|Width|Height|Map State'
# 抓窗口内容（有合成器时 xwd -root 只拿到根窗口，**看不到窗口内容**；改用 x11grab）
DISPLAY=:0 XAUTHORITY=/run/user/1000/gdm/Xauthority ffmpeg -y -f x11grab -video_size 1280x720 -i :0+5,355 -frames:v 1 win.png
DISPLAY=:0 XAUTHORITY=/run/user/1000/gdm/Xauthority ffmpeg -y -f x11grab -video_size 1280x720 -i :0+5,355 -t 5 -r 20 win.mp4
# 是否黑屏（客观判据，不依赖人眼/视觉模型）
python3 -c "from PIL import Image; import numpy as np; a=np.asarray(Image.open('win.png').convert('RGB')); print(a.mean(), len(np.unique(a.reshape(-1,3),axis=0)))"
```
本次证据（可复核）：`/tmp/mujoco_win.png`（窗口单帧 59225 B）、`/tmp/mujoco_win.mp4`（5 秒 20fps 36459 B）、
日志 `/tmp/piper-static.log`（静态）、`/tmp/piper-live.log`、`/tmp/piper-soft.log`（live）

## 5. 复现命令（照抄即可，立刻可见那条）

```bash
cd /home/coretek/AIIRAF
export DISPLAY=:0 XAUTHORITY=/run/user/1000/gdm/Xauthority MUJOCO_GL=glfw LIBGL_ALWAYS_SOFTWARE=1 PYTHONPATH=src
# 立刻可见：抓取约 2 秒后开窗，并持窗 120 秒
python3 scripts/view_mujoco.py --baseline config/piper_simulation_baseline.yaml --seconds 120
# 实时播放：先等约 92 秒（realtime 抓取），再开窗回放末态
python3 scripts/view_mujoco.py --baseline config/piper_simulation_baseline.yaml --live
```

> 备注：仿真结论一律 `simulation=true`；本记录只证"本机 x86 桌面能看见 MuJoCo 渲染与抓取末态"，
> 不构成任何真机/板卡能力证据（铁律 1.7）。

## 6. 修复（2026-10-16 当日）：live 路径"窗口从未创建却报成功"的根因与改法

**根因（实测定位）**：`scripts/view_mujoco.py` 自己探测到 `display_mode=interactive_viewer` 并据此选择 live 路径，
但 `viewer_runner.run_interactive_live()` **没有把该结论透传** 给 `run_request_live()`；
后者内部又 `mode = display_mode or resolve_display_mode()` **二次探测**，那次探测判成"非交互"，
于是**静默走离屏分支**（`if mode != DISPLAY_INTERACTIVE:`）：抓取线程照常执行（realtime ≈92 秒）、
窗口**从未尝试创建**、报告里 `window_opened=False / frames=0`，而外层仍打印 `VIEWER_OK` + exit 0。
诊断证据（修前一轮，`/tmp/piper-live-diag.log`）：
```
VIEWER_WINDOW_OPENED False
VIEWER_FRAMES 0
VIEWER_DEGRADED 窗口未真正建立或未渲染任何帧（window_opened=False frames=0）…   ← 新增的显式失败（exit=3）
```
⇒ "92 秒看不到窗口"与"窗口瞬关"是**同一件事**：窗口根本没建；先前误判为"开了又关"。

**改法（两处，最小改动）**
1. `run_interactive_live(..., display_mode=None)` 透传给 `run_request_live(..., display_mode=display_mode)`
   —— 候选结论只探测一次，不允许内外不一致；
2. `run_interactive_live` 回传**完整 report**（原来只回 `(holder, held)`，把 `window_opened`/`frames` 丢了）；
   `view_mujoco.py` 据此打印 `VIEWER_WINDOW_OPENED` / `VIEWER_FRAMES` / `VIEWER_STEPPING_NOTE`，
   并在 `window_opened=False` 或 `frames==0` 时判 **`VIEWER_DEGRADED` + 退出码 3**（替掉假绿 `VIEWER_OK`）。

**修复后实测（`--live --pick-duration-ms 3000 --seconds 25`，日志 `/tmp/piper-live-fix.log`）**
```
VIEWER_WINDOW_OPENED True
VIEWER_FRAMES 2291
VIEWER_OK        exit=0
```
抓取**进行中**窗口即存在：`WM_CLASS="MuJoCo"`，1280x720 @ (5,355)；
中途截图（`/tmp/mujoco_live_mid.png`）均值亮度 **54.5**、唯一色 **3593**、中央区唯一色 **2826** ⇒ 真实渲染。
⇒ **"边抓边看"成立**（窗口先开、抓取期间实时镜像；realtime 时全长 ≈92 秒，可按需用 `--pick-duration-ms` 缩短）。

**回归**：静态路径行为不变（`--seconds 120`：3.79 秒出窗、持窗后正常关闭、exit 0）。

## 7. 判读纪律：`/tmp` 里的旧日志会骗人，用**时间戳定界**（2026-10-16 11:32）

现象：修复提交后，一条**后台**诊断进程的完成通知才回来，日志 `/tmp/piper-live-diag.log` 里赫然是
`VIEWER_WINDOW_OPENED False / VIEWER_FRAMES 0 / exit=3`，看着像"改完又复发"。**是误报**，定界证据：

| 事实 | 值 |
|---|---|
| 失败日志最后写入时间（`stat -c %y`） | 11:26:07 |
| 修复提交 7314c5f 的时间 | 11:27:54（晚 1 分 47 秒） |

⇒ 那次跑的是**"已加判据、尚未透传 display_mode"的中间态工作区**，`VIEWER_DEGRADED exit=3` 正是它如实暴露根因；
它是"修前复现"，不是"修后复发"。

**同一命令用已提交代码重跑（新证据，`/tmp/piper-live-rerun.log`）**
```bash
DISPLAY=:0 XAUTHORITY=/run/user/1000/gdm/Xauthority MUJOCO_GL=glfw LIBGL_ALWAYS_SOFTWARE=1 \
  PYTHONPATH=src python3 scripts/view_mujoco.py --baseline config/piper_simulation_baseline.yaml \
  --live --pick-duration-ms 2000 --seconds 20
```
```
DISPLAY_MODE interactive_viewer ｜ VIEWER_WINDOW_OPENED True ｜ VIEWER_FRAMES 1678 ｜ VIEWER_OK
exit=0 ｜ 墙钟 45 s
```

**结论**：live 显示路径在已提交代码上稳定为 True/千帧级；见到 `VIEWER_DEGRADED` 先比对日志时间与
`git log --format='%ad' --date=format:'%m-%d %H:%M:%S' -1`，不要凭 /tmp 文件存在就判复发。
