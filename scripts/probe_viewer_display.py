"""显示准入探测：判定 MuJoCo Viewer 在本机可用的显示路径。

只读探测，不修改任何文件、不启动物理仿真。用于在打通可视化之前
确认图形会话是否真的可用，避免"假设能显示"。

判定顺序：
  1. interactive_viewer —— X11 会话可达且 GLFW 能创建 OpenGL 上下文
  2. offscreen_frames   —— 图形会话不可用，但 EGL 离屏渲染可用
  3. unavailable        —— 两条路径都不可用（显式失败，不静默兜底）

退出码：0 表示存在可用路径；1 表示两条路径都不可用。
"""

import argparse
import json
import os
import sys
from pathlib import Path

DISPLAY_MODE_INTERACTIVE = "interactive_viewer"
DISPLAY_MODE_OFFSCREEN = "offscreen_frames"
DISPLAY_MODE_UNAVAILABLE = "unavailable"


def _probe_glfw(display):
    """尝试在指定 DISPLAY 上创建 OpenGL 上下文，返回 (ok, detail)。"""
    previous = os.environ.get("DISPLAY")
    if display:
        os.environ["DISPLAY"] = display
    try:
        import glfw

        if not glfw.init():
            return False, "glfw.init() returned False"
        try:
            glfw.window_hint(glfw.VISIBLE, glfw.FALSE)
            window = glfw.create_window(320, 240, "iraf-display-probe", None, None)
            if window is None:
                return False, "glfw.create_window returned None (no GL context)"
            try:
                glfw.make_context_current(window)
                version = glfw.get_version_string()
            finally:
                glfw.destroy_window(window)
            return True, "GL context created, GLFW " + str(version)
        finally:
            glfw.terminate()
    except ImportError as exc:
        return False, "glfw unavailable: " + str(exc)
    except Exception as exc:
        return False, type(exc).__name__ + ": " + str(exc)
    finally:
        if previous is None:
            os.environ.pop("DISPLAY", None)
        else:
            os.environ["DISPLAY"] = previous


def _probe_egl_offscreen():
    """尝试用 EGL 离屏渲染一帧，返回 (ok, detail)。"""
    previous = os.environ.get("MUJOCO_GL")
    os.environ["MUJOCO_GL"] = "egl"
    try:
        import mujoco

        model = mujoco.MjModel.from_xml_string(
            "<mujoco><worldbody><body name='probe'>"
            "<geom type='box' size='0.05 0.05 0.05'/></body></worldbody></mujoco>"
        )
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        renderer = mujoco.Renderer(model, 120, 160)
        try:
            renderer.update_scene(data)
            frame = renderer.render()
        finally:
            renderer.close()
        if frame is None or getattr(frame, "shape", (0,))[0] == 0:
            return False, "renderer returned empty frame"
        return True, "EGL offscreen frame shape=" + str(tuple(frame.shape))
    except ImportError as exc:
        return False, "mujoco unavailable: " + str(exc)
    except Exception as exc:
        return False, type(exc).__name__ + ": " + str(exc)
    finally:
        if previous is None:
            os.environ.pop("MUJOCO_GL", None)
        else:
            os.environ["MUJOCO_GL"] = previous


def _x11_socket_present():
    return Path("/tmp/.X11-unix").is_dir() and any(Path("/tmp/.X11-unix").glob("X*"))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--display",
        default=None,
        help="要尝试接入的 X11 显示（默认按 :0、当前 DISPLAY 顺序尝试）",
    )
    parser.add_argument("--output", type=Path, default=None, help="报告输出路径")
    args = parser.parse_args(argv)

    candidates = []
    if args.display:
        candidates.append(args.display)
    else:
        for candidate in (":0", os.environ.get("DISPLAY")):
            if candidate and candidate not in candidates:
                candidates.append(candidate)

    report = {
        "schema_version": "iraf.mujoco.display-probe/v1",
        "x11_socket_present": _x11_socket_present(),
        "environment_display": os.environ.get("DISPLAY"),
        "glfw_attempts": [],
        "egl_offscreen": None,
        "display_mode": DISPLAY_MODE_UNAVAILABLE,
        "fallback_reason": None,
    }

    interactive_detail = None
    for candidate in candidates:
        ok, detail = _probe_glfw(candidate)
        report["glfw_attempts"].append(
            {"display": candidate, "ok": bool(ok), "detail": detail}
        )
        if ok:
            report["display_mode"] = DISPLAY_MODE_INTERACTIVE
            report["selected_display"] = candidate
            interactive_detail = detail
            break

    egl_ok, egl_detail = _probe_egl_offscreen()
    report["egl_offscreen"] = {"ok": bool(egl_ok), "detail": egl_detail}

    if report["display_mode"] == DISPLAY_MODE_UNAVAILABLE:
        if egl_ok:
            report["display_mode"] = DISPLAY_MODE_OFFSCREEN
            report["fallback_reason"] = (
                "GLFW/OpenGL 上下文不可用，已降级为 EGL 离屏逐帧渲染"
            )
        else:
            report["fallback_reason"] = (
                "GLFW 与 EGL 均不可用，显示通道不可用"
            )

    report["passed"] = report["display_mode"] != DISPLAY_MODE_UNAVAILABLE

    output = args.output
    if output:
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(
            json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
        )

    print("DISPLAY_MODE " + report["display_mode"], flush=True)
    print("X11_SOCKET " + str(report["x11_socket_present"]), flush=True)
    for attempt in report["glfw_attempts"]:
        print(
            "GLFW display="
            + str(attempt["display"])
            + " ok="
            + str(attempt["ok"])
            + " detail="
            + attempt["detail"],
            flush=True,
        )
    print("EGL_OFFSCREEN " + str(egl_ok) + " detail=" + egl_detail, flush=True)
    if report["fallback_reason"]:
        print("FALLBACK_REASON " + report["fallback_reason"], flush=True)
    print("PASSED " + str(report["passed"]), flush=True)
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
