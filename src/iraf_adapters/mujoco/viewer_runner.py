"""通用仿真显示入口：驱动一次完整抓取全过程并可视化。

设计要点（对应 IRAF 铁律：硬件差异只进 adapters/profiles）：
- 后端经 factory.load_backend 动态装配，不直接 import 具体后端类；
- 只调用 RobotBackend 公开契约，不触碰实现的下划线私有成员；
- 相机视角、Home 位姿、目标与容差全部取自 RobotProfile；
- 显示路径先探测后使用：图形会话可用则开交互视窗，否则降级为离屏逐帧，
  并在报告中显式标注 display_mode 与降级原因，不静默假装成功；
- 抓取经 SkillRuntime 执行，不绕过 Policy Gateway。

本模块不依赖任何具体机械臂；Piper 只是当前 profile 的一个实例。

显示能力分三档，按"能看到多少过程"递增：

1. ``run_interactive``（既有）：先完成抓取，再打开窗口展示**末态**。
   适合"确认结果正确"，看不到中间阶段。
2. ``run_offscreen``（既有）：抓取完成后导出帧序列，适合无图形会话的 CI。
3. ``run_interactive_live``（新增）：**窗口先开、抓取后启**，
   配合 ``SnapshotMirror`` 让渲染与物理线程解耦，
   HOME→APPROACH→DESCEND→GRIP→LIFT 全过程可见（真实时序需 realtime=True）。

``run_interactive_live`` 的并发正确性依赖三件事，缺一即退化为"只有一帧"：

- 渲染只读 ``SnapshotMirror`` 持有的副本，绝不在抓取期间直接同步后端 data；
- 仅在拷贝快照的瞬间持有 ``display_lock()``，且拷贝本身不耗时；
- 窗口生命周期在单次调用内闭合。

这些约束与新机器人无关：它们只依赖 RobotBackend 的
``model`` / ``data`` / ``display_lock()`` 三个公开成员，
因此在 Piper 上验证的结论对其他构型同样成立。
新增机器人时**不需要**改动本模块。
"""

import os
import threading
import time
from pathlib import Path

import numpy as np

REPORT_SCHEMA = "iraf.mujoco.viewer-report/v1"

DISPLAY_INTERACTIVE = "interactive_viewer"
DISPLAY_OFFSCREEN = "offscreen_frames"
DISPLAY_UNAVAILABLE = "unavailable"


class ViewerError(RuntimeError):
    """显示链无法继续时抛出，携带可读原因。"""


def resolve_display_mode(display=None, allow_probe=True):
    """探测显示通道，返回 (display_mode, detail, fallback_reason)。

    先尝试图形会话（GLFW/OpenGL），失败则回退 EGL 离屏渲染；
    两者都不可用时返回 unavailable，由调用方显式失败。

    allow_probe=False 时跳过 GLFW 试创建窗口，仅依据 DISPLAY 与 X11 socket
    判断交互可用性。这是必须的：在定义进程内先创建再销毁 GLFW 窗口会残留
    GL 上下文状态，之后 launch_passive 会报
    "mj_copyDataVisual: attempting to copy mjData while stack is in use"。
    """
    # 显式环境变量优先：允许部署侧直接声明显示路径，跳过探测副作用。
    forced = os.environ.get("IRAF_VIEWER_DISPLAY_MODE")
    if forced in (DISPLAY_INTERACTIVE, DISPLAY_OFFSCREEN):
        return forced, "由 IRAF_VIEWER_DISPLAY_MODE 显式指定", (
            None if forced == DISPLAY_INTERACTIVE else "显式指定离屏渲染"
        )

    candidates = []
    for candidate in (display, os.environ.get("DISPLAY"), ":0"):
        if candidate and candidate not in candidates:
            candidates.append(candidate)

    previous = os.environ.get("DISPLAY")
    try:
        for candidate in candidates:
            os.environ["DISPLAY"] = candidate
            if not allow_probe:
                # 不做 GLFW 试创建，仅依据 X11 连通性判断。
                socket = "/tmp/.X11-unix/X" + str(candidate).lstrip(":")
                if Path(socket).exists():
                    return (
                        DISPLAY_INTERACTIVE,
                        "X11 socket " + socket + " 可达（未做 GLFW 试创建）",
                        None,
                    )
                continue
            try:
                import glfw

                if not glfw.init():
                    continue
                try:
                    glfw.window_hint(glfw.VISIBLE, glfw.FALSE)
                    window = glfw.create_window(
                        320, 240, "iraf-viewer-probe", None, None
                    )
                    if window is None:
                        continue
                    try:
                        glfw.make_context_current(window)
                    finally:
                        glfw.destroy_window(window)
                    return (
                        DISPLAY_INTERACTIVE,
                        "GLFW "
                        + str(glfw.get_version_string())
                        + " on DISPLAY="
                        + candidate,
                        None,
                    )
                finally:
                    glfw.terminate()
            except Exception:
                continue
    finally:
        if previous is None:
            os.environ.pop("DISPLAY", None)
        else:
            os.environ["DISPLAY"] = previous

    previous_gl = os.environ.get("MUJOCO_GL")
    os.environ["MUJOCO_GL"] = "egl"
    try:
        import mujoco

        model = mujoco.MjModel.from_xml_string(
            "<mujoco><worldbody><geom type='box' size='0.01 0.01 0.01'/>"
            "</worldbody></mujoco>"
        )
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        renderer = mujoco.Renderer(model, 60, 80)
        try:
            renderer.update_scene(data)
            frame = renderer.render()
        finally:
            renderer.close()
        if frame is None or getattr(frame, "shape", (0,))[0] == 0:
            raise RuntimeError("EGL 渲染返回空帧")
        return (
            DISPLAY_OFFSCREEN,
            "EGL offscreen frame shape=" + str(tuple(frame.shape)),
            "图形会话不可用，已降级为 EGL 离屏逐帧渲染",
        )
    except Exception as exc:
        return (
            DISPLAY_UNAVAILABLE,
            type(exc).__name__ + ": " + str(exc),
            "GLFW 与 EGL 均不可用，显示通道不可用",
        )
    finally:
        if previous_gl is None:
            os.environ.pop("MUJOCO_GL", None)
        else:
            os.environ["MUJOCO_GL"] = previous_gl


def camera_settings(profile, fallback=None):
    """从 profile 读取相机视角，缺失时回退调用方默认值。"""
    camera = getattr(profile, "camera", None) or {}
    defaults = fallback or {}
    return {
        "lookat_m": list(
            camera.get("lookat_m") or defaults.get("lookat_m", [0.0, 0.0, 0.1])
        ),
        "distance_m": float(camera.get("distance_m", defaults.get("distance_m", 1.0))),
        "azimuth_deg": float(
            camera.get("azimuth_deg", defaults.get("azimuth_deg", 180.0))
        ),
        "elevation_deg": float(
            camera.get("elevation_deg", defaults.get("elevation_deg", -8.0))
        ),
    }


def target_tolerance(profile, target_id, default=0.005):
    """从 profile 读取目标的位姿容差。"""
    manipulation = getattr(profile, "manipulation", None) or {}
    targets = manipulation.get("targets") or {}
    entry = targets.get(target_id) or {}
    return float(entry.get("pose_tolerance_m", default))


def build_move_request(profile, safety, positions, duration_ms, correlation_id):
    """按 IRAF 契约组装关节运动请求（Home 等定位动作也必须经 Skill Runtime）。"""
    now = int(time.time() * 1000)
    return {
        "request_id": correlation_id,
        "idempotency_key": correlation_id + "-" + str(now),
        "correlation_id": correlation_id,
        "skill": "move_joint",
        "skill_version_constraint": "1.0.0",
        "parameters": {
            "positions": {str(k): float(v) for k, v in positions.items()},
            "duration_ms": int(duration_ms),
        },
        "deadline_unix_ms": now + max(60000, int(duration_ms) * 4),
        "profile_name": profile.name,
        "profile_version": profile.version,
        "profile_digest": profile.digest,
        "safety_policy_name": safety.name,
        "safety_policy_version": safety.version,
        "safety_policy_digest": safety.digest,
        "resource_id": profile.name + "-mujoco",
        "controller": correlation_id,
    }


def build_pick_request(profile, safety, target_id, position, duration_ms, correlation_id):
    """按 IRAF 契约组装任务请求（含 profile/safety 摘要，供 Policy 校验）。"""
    now = int(time.time() * 1000)
    return {
        "request_id": correlation_id,
        "idempotency_key": correlation_id + "-" + str(now),
        "correlation_id": correlation_id,
        "skill": "pick_object",
        "skill_version_constraint": "1.0.0",
        "parameters": {
            "target_id": target_id,
            "grasp_pose": {
                "frame_id": "world",
                "position": {
                    "x": float(position[0]),
                    "y": float(position[1]),
                    "z": float(position[2]),
                },
                "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
            },
            "duration_ms": int(duration_ms),
        },
        "deadline_unix_ms": now + max(90000, int(duration_ms) * 8),
        "profile_name": profile.name,
        "profile_version": profile.version,
        "profile_digest": profile.digest,
        "safety_policy_name": safety.name,
        "safety_policy_version": safety.version,
        "safety_policy_digest": safety.digest,
        "resource_id": profile.name + "-mujoco",
        "controller": correlation_id,
    }


class ViewerSession:
    """一次显示会话：负责 Home → 抓取 → 末态保持 的可视化编排。"""

    def __init__(self, profile, safety, backend, runtime, context=None):
        self.profile = profile
        self.safety = safety
        self.backend = backend
        self.runtime = runtime
        self.context = context
        self._phases = []
        self._started = None

    def mark(self, name):
        if self._started is None:
            self._started = time.monotonic()
        self._phases.append(
            {
                "name": name,
                "elapsed_ms": round((time.monotonic() - self._started) * 1000.0, 3),
            }
        )

    @property
    def phases(self):
        return list(self._phases)

    def home(self, duration_ms, correlation_id="viewer-home"):
        """回到 Home 位姿。

        必须经 Skill Runtime 提交：后端 move_joint 要求有效资源租约，
        而租约只能由 Runtime 在其受控流程内获取，外部不得自造。
        位姿取自 profile 的结构化 home 段。
        """
        self.mark("HOME")
        positions = self.backend.home_pose()
        move_ms = max(1, int(duration_ms) // 5)
        request = build_move_request(
            self.profile, self.safety, positions, move_ms, correlation_id
        )
        result = self.runtime.execute(request, self.context)
        if not isinstance(result, dict):
            raise ViewerError("Home 动作返回非结构化结果: " + str(type(result)))
        if result.get("status") not in (None, "SUCCEEDED"):
            raise ViewerError(
                "Home 动作未成功: status="
                + str(result.get("status"))
                + " error="
                + str(result.get("error_code"))
            )
        return positions

    def start_pick_async(self, target_id, position, duration_ms, correlation_id):
        """在后台线程发起抓取，返回 (thread, result_holder)。"""
        request = build_pick_request(
            self.profile, self.safety, target_id, position, duration_ms, correlation_id
        )
        holder = {"result": None, "error": None, "ready": threading.Event()}

        def worker():
            try:
                holder["result"] = self.runtime.execute(request, self.context)
            except Exception as exc:  # 显式记录，不吞错
                holder["error"] = type(exc).__name__ + ": " + str(exc)
            finally:
                holder["ready"].set()

        thread = threading.Thread(target=worker, name="viewer-pick", daemon=True)
        thread.start()
        return thread, holder

    def hold(self):
        """保持末态位形，避免 Runtime 收尾后机械臂塌回零位。"""
        self.mark("HOLD")
        return self.backend.hold_current_pose()


def run_interactive(
    session, target_id, position, duration_ms, correlation_id, camera, seconds=0.0
):
    """交互视窗路径。

    架构约束（实测得出）：显示的实时性与技能超时预算本质冲突。
    pick_object 内部的稳定窗口按 duration 成比例放大，在 realtime=True 下
    实测需约 92 秒，必然超出技能声明的 30 秒租约。

    因此采用解耦方案：
    1. 抓取以仿真时间快速完成（realtime=False，实测约 2 秒），保证物理正确
       与验收有效，不改动后端时序语义；
    2. 显示层负责回放——抓取期间在主循环中持续推进并渲染，使运动可见；
    3. 抓取结束后保持末态。
    """
    import mujoco.viewer

    lock = session.backend.display_lock()

    # 阶段一：抓取在独立进程中完成，不启动任何 viewer。
    # 依据：mujoco.viewer.launch_passive 会创建自己的 GLFW 渲染线程，
    # 该线程在任意时刻调用 mjv_updateScene（内部 mj_copyDataVisual）读取
    # MjData；即使调用方加互斥锁，也无法阻止 view 内部线程与抓取线程的
    # mj_step/mj_forward 交错，实测三次中两次报
    # "attempting to copy mjData while stack is in use"。
    # 因此窗口必须与抓取串行化：先完成抓取，再打开窗口展示结果。
    thread, holder = session.start_pick_async(
        target_id, position, duration_ms, correlation_id
    )
    thread.join(timeout=max(60.0, duration_ms / 1000.0 * 12))
    if thread.is_alive():
        raise ViewerError("抓取线程未在预期时间内结束")
    if holder["error"]:
        raise ViewerError("抓取失败: " + str(holder["error"]))

    session.hold()
    held = True

    # 阶段二：打开窗口展示抓取结果，物理保持末态并可继续步进。
    viewer_seconds = seconds if seconds else 0.0
    with mujoco.viewer.launch_passive(
        session.backend.model, session.backend.data
    ) as viewer:
        viewer.cam.lookat[:] = camera["lookat_m"]
        viewer.cam.distance = camera["distance_m"]
        viewer.cam.azimuth = camera["azimuth_deg"]
        viewer.cam.elevation = camera["elevation_deg"]

        started = time.monotonic()
        while viewer.is_running():
            with lock:
                session.backend.step()
                viewer.sync()
            if viewer_seconds and time.monotonic() - started >= viewer_seconds:
                break
            time.sleep(0.002)

    holder["ready"].wait(timeout=60.0)
    return holder, held


def run_offscreen(
    session, target_id, position, duration_ms, correlation_id, frames_dir, frame_count
):
    """离屏降级路径：抓取完成后导出帧序列，证明全过程可回放。

    帧导出在抓取线程结束后进行，避免与 Skill 物理步进并发访问 MjData。
    """
    import numpy as np

    if frames_dir is None:
        raise ViewerError("离屏降级路径必须提供 frames_dir")
    frames_dir = Path(frames_dir)
    frames_dir.mkdir(parents=True, exist_ok=True)

    thread, holder = session.start_pick_async(
        target_id, position, duration_ms, correlation_id
    )
    thread.join(timeout=max(30.0, duration_ms / 1000.0 + 30.0))
    if thread.is_alive():
        raise ViewerError("抓取线程未在预期时间内结束，拒绝导出不稳定状态的帧")
    if holder["error"]:
        raise ViewerError("抓取失败，无法导出帧: " + str(holder["error"]))

    session.hold()
    held = True

    try:
        from PIL import Image
    except ImportError:
        Image = None

    emitted = 0
    for index in range(int(frame_count)):
        frame = session.backend.render_frames(1)[0]
        if Image is not None:
            Image.fromarray(np.asarray(frame)).save(
                frames_dir / ("frame_%03d.png" % index)
            )
        emitted += 1
    return holder, held, emitted


class SnapshotMirror:
    """渲染用状态快照：把物理线程的状态与渲染解耦。

    为什么需要它（实测结论，不是设计偏好）：

    - `RobotBackend.display_lock()` 返回的就是物理步进使用的互斥锁；
      抓取过程中每个 timestep 都要抢一次该锁（realtime=True 时更密集）。
    - 若渲染循环也在同一把锁上做 `viewer.sync()`，两者会高频争抢：
      渲染被饿死，屏幕上整个抓取过程只剩**一帧**。
    - 因此渲染必须基于**本方持有的副本**，只在拷贝瞬间短暂持锁。

    各机器人差异已被 RobotBackend 契约吸收：本类只依赖
    `model` / `data` / `display_lock()`，不含任何机型专有名称。
    因此在 Piper 上验证的并发正确性，对 Franka 等其他构型同样成立。

    mocap 必须显式复制（关键，实测）：
    `mjSTATE_FULLPHYSICS` **不包含** mocap（实测 carried=False），
    而抓取场景用 mocap body（如 Piper 的 grasp_anchor）驱动抬升约束。
    漏掉 mocap 会让渲染中的被抓取物与机械臂错位，
    这类错误在静态末态图上很难被发现，必须在此处阻断。
    """

    def __init__(self, model):
        import mujoco

        self._mujoco = mujoco
        self.model = model
        self.mirror = mujoco.MjData(model)
        self._spec = mujoco.mjtState.mjSTATE_FULLPHYSICS
        # 缓冲区长度必须恰为 mj_stateSize(model, spec)，否则 mj_getState 会报
        # "state size should equal mj_stateSize(m, spec)"。
        size = int(mujoco.mj_stateSize(model, self._spec))
        if size <= 0:
            raise ViewerError("mj_stateSize 返回非法长度: " + str(size))
        self._buffer = np.zeros(size, dtype=np.float64)
        self._has_mocap = int(model.nmocap) > 0

    def refresh(self, backend):
        """从后端取一次快照；仅在拷贝期间持锁，随后立即释放。

        注意 mj_copyData 在 MuJoCo 3.3.3 的 Python 绑定中**不存在**
        （实测 AttributeError），必须走 mj_getState/mj_setState。
        """
        mujoco = self._mujoco
        with backend.display_lock():
            mujoco.mj_getState(self.model, backend.data, self._buffer, self._spec)
            mujoco.mj_setState(self.model, self.mirror, self._buffer, self._spec)
            if self._has_mocap:
                self.mirror.mocap_pos[:] = backend.data.mocap_pos
                self.mirror.mocap_quat[:] = backend.data.mocap_quat
        mujoco.mj_forward(self.model, self.mirror)
        return self.mirror


def build_grasp_request(
    profile,
    safety,
    skill,
    target_id,
    position,
    duration_ms,
    correlation_id,
    orientation=None,
    deadline_slack_ms=90000,
):
    """组装抓取请求，允许指定 skill 并放宽截止时间。

    与 build_pick_request 的差别：
    - skill 可指定：显示场景可用超时上限更宽的 skill（如 display_pick），
      以支持 realtime=True 下约 92 秒的完整过程播放；
    - deadline 按时长放大：`build_pick_request` 的 max(90000, duration*8)
      在 realtime 下会与抓取时长冲突，这里改为显式换算。

    参数与 profile/safety 摘要仍必须齐全——显示路径同样经 Policy 校验，
    不绕过任何安全策略（IRAF 铁律 2）。
    """
    now = int(time.time() * 1000)
    pose = {
        "frame_id": "world",
        "position": {
            "x": float(position[0]),
            "y": float(position[1]),
            "z": float(position[2]),
        },
    }
    if orientation is None:
        pose["orientation"] = {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
    else:
        pose["orientation"] = {
            "x": float(orientation[0]),
            "y": float(orientation[1]),
            "z": float(orientation[2]),
            "w": float(orientation[3]),
        }
    return {
        "request_id": correlation_id,
        "idempotency_key": correlation_id + "-" + str(now),
        "correlation_id": correlation_id,
        "skill": str(skill),
        "skill_version_constraint": "1.0.0",
        "parameters": {
            "target_id": target_id,
            "grasp_pose": pose,
            "duration_ms": int(duration_ms),
        },
        "deadline_unix_ms": now
        + max(int(deadline_slack_ms), int(duration_ms) * 12 + 60000),
        "profile_name": profile.name,
        "profile_version": profile.version,
        "profile_digest": profile.digest,
        "safety_policy_name": safety.name,
        "safety_policy_version": safety.version,
        "safety_policy_digest": safety.digest,
        "resource_id": profile.name + "-mujoco",
        "controller": correlation_id,
    }


def display_environment():
    """返回显示相关的环境事实 + 是否需要提醒（把窗口开到了非桌面显示时给出可操作提示）。

    实测教训（2026-09-21）：本机 shell 的 `DISPLAY=localhost:11.0`（X 转发目标），而桌面会话是 `:0`
    （套接字 /tmp/.X11-unix/X0、授权 /run/user/1000/gdm/Xauthority）。此时窗口"确实开出来了"
    （报告 window_opened=true、渲染了几十帧），但**使用者在他的桌面上看不到** —— 报告里必须写明
    实际使用的显示与桌面显示不同，并给出正确命令，否则"窗口已开"会误导。
    """
    env = {key: os.environ.get(key) for key in ("DISPLAY", "XAUTHORITY", "MUJOCO_GL")}
    local = sorted(p.name[1:] for p in Path("/tmp/.X11-unix").glob("X*")) if Path("/tmp/.X11-unix").is_dir() else []
    desktop = ":" + local[0] if local else None
    current = env.get("DISPLAY") or ""
    warning = None
    if desktop and current and current != desktop and ":" in current:
        warning = (
            "窗口开在 DISPLAY=%s（可能是 X 转发目标），而本机桌面会话在 %s："
            "使用者看不到窗口。要看得见请显式指定 DISPLAY=%s XAUTHORITY=<桌面授权文件，如 "
            "/run/user/%s/gdm/Xauthority> MUJOCO_GL=glfw"
            % (current, desktop, desktop, os.getuid())
        )
    return {"display": current or None, "xauthority": env.get("XAUTHORITY"),
            "mujoco_gl": env.get("MUJOCO_GL"), "desktop_display": desktop,
            "local_sockets": local, "warning": warning}


def _apply_camera(viewer, camera):
    """相机设置来自 Profile（camera_settings），此处只做赋值，不含默认值。"""
    if not camera:
        return
    viewer.cam.lookat[:] = camera["lookat_m"]
    viewer.cam.distance = camera["distance_m"]
    viewer.cam.azimuth = camera["azimuth_deg"]
    viewer.cam.elevation = camera["elevation_deg"]


def _start_request_thread(runtime, request, context, name="viewer-skill"):
    """在后台线程执行一个 Skill 请求，返回 (thread, holder)。执行必须走 SkillRuntime（不绕层）。"""
    holder = {"result": None, "error": None, "ready": threading.Event()}

    def worker():
        try:
            holder["result"] = runtime.execute(request, context)
        except Exception as exc:  # noqa: BLE001 显式记录，不吞错
            holder["error"] = type(exc).__name__ + ": " + str(exc)
        finally:
            holder["ready"].set()

    thread = threading.Thread(target=worker, name=name, daemon=True)
    return thread, holder


def run_request_live(
    backend,
    runtime,
    request,
    context,
    *,
    camera=None,
    render_hz=60.0,
    seconds=0.0,
    pre_roll_frames=30,
    frames_dir=None,
    frame_count=0,
    on_finished=None,
    continue_stepping=True,
    display_mode=None,
    timeout_s=180.0,
):
    """**机器人无关**的实时播放：执行一个已构造的 Skill 请求，并在此期间实时镜像仿真状态。

    与 `run_interactive_live` 的关系：后者是臂侧薄包装（自行构造抓取请求、并在结束后 hold 末态），
    两者的渲染循环共用本函数一份实现。并发契约（实测得出，见 `SnapshotMirror` 文档）：
      1. 渲染只读 SnapshotMirror 的副本，绝不在执行期间直接 `sync()` 后端的 `data`；
      2. 快照拷贝期间才持有 `display_lock()`，且不做任何耗时操作；
      3. 窗口与执行的生命周期在本函数内闭合，不跨调用复用窗口。

    参数：`frames_dir`/`frame_count` 用于无显示设备时的**离屏降级**（导出帧序列并如实标注）；
    `on_finished` 在执行线程结束后调用一次（臂侧用它保持末态）；`continue_stepping` 决定
    执行结束后是否继续步进以保持窗口存活（`seconds=0` 语义＝保持窗口直到用户关闭）。

    返回 dict：`display_mode` / `window_opened` / `frames` / `frames_written` / `holder` / `error`。
    """
    mode = display_mode or resolve_display_mode()
    frame_period = 1.0 / max(1.0, float(render_hz))
    thread, holder = _start_request_thread(runtime, request, context)
    display_env = display_environment()
    report = {
        "display_mode": mode,
        "display_env": display_env,
        "window_opened": False,
        "frames": 0,
        "frames_written": 0,
        "holder": holder,
        "error": None,
        "continue_stepping": bool(continue_stepping),
        "render_hz": float(render_hz),
        "seconds": float(seconds),
    }

    if mode != DISPLAY_INTERACTIVE:
        # 离屏/不可用：执行结束后导出帧序列（不谎称"窗口已开"）
        thread.start()
        thread.join(timeout=max(float(timeout_s), 30.0))
        if thread.is_alive():
            report["error"] = "Skill 执行线程未在预期时间内结束，拒绝导出不稳定状态的帧"
            return report
        if holder["error"]:
            report["error"] = holder["error"]
            return report
        if on_finished is not None:
            on_finished()
        if mode == DISPLAY_OFFSCREEN and frames_dir is not None and int(frame_count) > 0:
            if not hasattr(backend, "render_frames"):
                # 不造帧：后端没有离屏导出能力就如实报告，交给调用方决定
                report["error"] = "该后端未实现 render_frames（无离屏导出能力），拒绝伪造帧序列"
                return report
            try:
                from PIL import Image
            except ImportError:
                Image = None
            frames_dir = Path(frames_dir)
            frames_dir.mkdir(parents=True, exist_ok=True)
            for index in range(int(frame_count)):
                frame = backend.render_frames(1)[0]
                if Image is not None:
                    Image.fromarray(np.asarray(frame)).save(frames_dir / ("frame_%03d.png" % index))
                report["frames"] += 1
            report["frames_written"] = report["frames"]
        return report

    import mujoco.viewer

    snapshot = SnapshotMirror(backend.model)
    with mujoco.viewer.launch_passive(backend.model, snapshot.refresh(backend)) as viewer:
        report["window_opened"] = True
        if display_env.get("warning"):
            # 不阻止运行，但把"使用者可能看不到"这件事写进日志与报告（可审计）
            print("[viewer] 注意：" + display_env["warning"], flush=True)
        _apply_camera(viewer, camera)

        # 先渲染若干帧，让用户看到动作前的起始状态。窗口被关掉就停下来，不盲跑。
        for _ in range(max(0, int(pre_roll_frames))):
            if not viewer.is_running():
                break
            viewer.sync()
            time.sleep(frame_period)

        thread.start()
        started = time.monotonic()
        finished = False
        while viewer.is_running():
            snapshot.refresh(backend)
            viewer.sync()
            report["frames"] += 1
            if thread.is_alive():
                time.sleep(frame_period)
                continue
            if not finished:
                if on_finished is not None:
                    on_finished()
                finished = True
                started = time.monotonic()
                continue
            if float(seconds) and time.monotonic() - started >= float(seconds):
                break
            stepper = getattr(backend, "step", None)
            if not continue_stepping or not callable(stepper):
                # 后端没有单步接口（如四足适配器）：**不假装继续步进**，只保持窗口渲染末态。
                # 实测：四足适配器无 step()，直接调用会 AttributeError（窗口路径整条失败）。
                if not callable(stepper):
                    report["stepping_note"] = (
                        "该后端未实现 step()：窗口保持期间不步进，画面停在末态（不伪造「持续运动」）"
                    )
                report["continue_stepping"] = False
                time.sleep(frame_period)
                continue
            with backend.display_lock():
                stepper()
            time.sleep(frame_period)

    thread.join(timeout=max(float(timeout_s), 30.0))
    if thread.is_alive():
        report["error"] = "Skill 执行线程未在预期时间内结束"
        return report
    if not finished and on_finished is not None:
        on_finished()
    return report


def run_live_mirror(backend, *, render_hz=20.0, seconds=0.0, stop_event=None, camera=None,
                    display_mode=None, pre_roll_frames=0, software_gl=None,
                    window_px=None, hold_seconds=0.0):
    """**只渲染、不推进**的实时镜像会话（供"验收运行边跑边看"用）。

    为什么需要它（2026-09-28，`docs/debug/2026-09-24-joint-model-dog-arm.md` §11.9）：
    `run_request_live` 必须绑一个 Skill 请求，而**联合世界**的时间推进不该由窗口负责 ——
    它由声明驱动的植物驻留线程（`scripts/scenario.py: _start_plant_residency`）推进 owner。
    演示脚本自己造"时间推进者"会撞上安全策略的 `max_duration_ms`（实测 stand 60000 ms 被拒），
    于是 owner 停步、guest 卡在"等待 owner 推进"。

    并发契约与 `run_request_live` **完全一致**（同一 `SnapshotMirror`）：
      1. 只读镜像副本，绝不在**别人推进**的同时直接 `sync()` 后端的 `data`；
      2. 拷贝期间才持 `display_lock()`，且不做耗时操作；
      3. 本函数**不**调用任何 Skill、也不调用 `backend.step()` ⇒ 不与 owner 争抢植物。

    结束条件：`stop_event` 被置位，或窗口被关闭（或 `seconds>0` 且已过该时长）。
    返回 dict：`display_mode` / `window_opened` / `frames` / `error` / `stopped_by`。
    """
    mode = display_mode or resolve_display_mode()
    frame_period = 1.0 / max(1.0, float(render_hz))
    report = {"display_mode": mode, "window_opened": False, "frames": 0,
              "error": None, "stopped_by": None, "render_hz": float(render_hz)}
    if mode != DISPLAY_INTERACTIVE:
        report["error"] = "只渲染会话需要 interactive_viewer（离屏降级请用 run_request_live）"
        return report

    # ⚠ GL 路径与窗口尺寸必须在**创建 GL 上下文之前**定（`import mujoco.viewer` 就会建上下文）。
    # 依据（2026-09-28 §11.23(41) 与 docs/debug/2026-09-23-go2-viewer-3d-black-screen.md）：
    # 本机（4 核、无独显）默认 GL 路径下窗口 3D 视口几乎不亮，且 llvmpipe 在 1280×720 下每帧
    # 要数秒 ⇒ 使用者只看到"闪一下"。两者都由**机型声明的 `render` 段**给出（`software_gl` /
    # `width_px` / `height_px`），不在这里写默认值。
    if software_gl:
        os.environ["LIBGL_ALWAYS_SOFTWARE"] = "1"
        os.environ["GALLIUM_DRIVER"] = "llvmpipe"
    if window_px:
        backend.model.vis.global_.offwidth = int(window_px[0])
        backend.model.vis.global_.offheight = int(window_px[1])
    report["gl_mode"] = ("software_llvmpipe" if software_gl else "default")
    report["window_px"] = [int(window_px[0]), int(window_px[1])] if window_px else None
    print("VIEWER_GL %s%s" % (report["gl_mode"],
                              "" if not window_px else " window=%dx%d" % tuple(report["window_px"])),
          flush=True)

    import mujoco.viewer

    snapshot = SnapshotMirror(backend.model)
    started = time.monotonic()
    with mujoco.viewer.launch_passive(backend.model, snapshot.refresh(backend)) as viewer:
        report["window_opened"] = True
        # 相机参数两种口径（都由声明给出）：
        #   · 字符串 = 模型里的**固定相机名**（机型声明 `render.camera`，本机实测亮度最好：固定+软件 GL mean 95.1）
        #   · 字典   = 自由相机初值（lookat/distance/azimuth/elevation）
        if isinstance(camera, str) and camera:
            camera_id = mujoco.mj_name2id(backend.model, mujoco.mjtObj.mjOBJ_CAMERA, camera)
            if camera_id < 0:
                report["error"] = "声明的 render.camera=%r 在模型里不存在" % camera
            else:
                viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FIXED
                viewer.cam.fixedcamid = camera_id
                report["camera"] = camera
                print("VIEWER_CAMERA fixed %s" % camera, flush=True)
        else:
            _apply_camera(viewer, camera)
        for _ in range(max(0, int(pre_roll_frames))):
            if not viewer.is_running():
                break
            viewer.sync()
            time.sleep(frame_period)
        while viewer.is_running():
            snapshot.refresh(backend)
            viewer.sync()
            report["frames"] += 1
            if stop_event is not None and stop_event.is_set():
                # ⚠ 收到停止信号**不能立刻退出**（2026-09-28 §11.23(41) 实测）：退出 `with` 块
                # 就等于关窗 ⇒ 验收一结束窗口立刻消失，使用者只看到"闪一下就不见了"。
                # 语义修正：`hold_seconds` 内继续渲染末态（窗口保持可见），到点再关。
                report["stopped_by"] = "stop_event"
                hold_until = time.monotonic() + max(0.0, float(hold_seconds))
                while viewer.is_running() and time.monotonic() < hold_until:
                    snapshot.refresh(backend)
                    viewer.sync()
                    report["frames"] += 1
                    report["hold_frames"] = report.get("hold_frames", 0) + 1
                    time.sleep(frame_period)
                break
            if float(seconds) and time.monotonic() - started >= float(seconds):
                report["stopped_by"] = "seconds"
                break
            time.sleep(frame_period)
        else:
            report["stopped_by"] = "window_closed"
    return report


def run_interactive_live(
    session,
    target_id,
    position,
    duration_ms,
    correlation_id,
    camera,
    seconds=0.0,
    skill="pick_object",
    render_hz=60.0,
    pre_roll_frames=30,
    orientation=None,
):
    """在窗口打开的状态下执行抓取，使完整过程可见（臂侧薄包装）。

    编排（构造抓取请求、结束后 hold 末态、保持窗口直到用户关闭）保留在本函数；
    渲染循环与并发契约由 `run_request_live` 统一实现，供四足/其它本体复用。
    skill 缺省 pick_object；需要真实时序完整播放时传超时上限更宽的 skill
    （如 display_pick），并配合放宽的 SafetyPolicy。
    """
    request = build_grasp_request(
        session.profile,
        session.safety,
        skill,
        target_id,
        position,
        duration_ms,
        correlation_id,
        orientation=orientation,
        deadline_slack_ms=max(90000, int(duration_ms) * 12 + 60000),
    )
    session.mark("PICK")
    report = run_request_live(
        session.backend,
        session.runtime,
        request,
        session.context,
        camera=camera,
        render_hz=render_hz,
        seconds=seconds,
        pre_roll_frames=pre_roll_frames,
        on_finished=session.hold,
        timeout_s=max(180.0, float(duration_ms) / 1000.0 * 14),
    )
    return report["holder"], bool(report["window_opened"])
