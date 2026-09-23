"""Go2 步态**显示/演示**入口：把当前步态实现实时放进 MuJoCo 窗口。

================================ 与验收的关系（先读） ================================
本入口**不是验收**：它只负责让你在屏幕上看见步态在做什么。数值口径、判据与通过与否以
`scripts/verify_go2_trot_in_place.py` 为准（同一份实现、同一条租约路径、同一份声明）。
报告里显式标注 `display_only: true`，任何"看起来在动"都不构成能力证据。

参数全部来自声明（禁止第二份数字）：
- 步态参数：`config/go2_loopback.yaml` 的 `gait` 段；
- 显示参数（相机名/分辨率/渲染频率）：同文件的 `render` 段；
- 关节身份与限位：`profiles/unitree_go2_mujoco.yaml`；移动边界：`profiles/safety/quadruped_lab.yaml`。

自限时（避免留下长期驻留进程）：`--seconds` 到期即关闭窗口；窗口被手动关掉也会立即退出。

用法（桌面可见时必须显式指定桌面显示；SSH/cron 会话的 DISPLAY 常是 X 转发目标）::

    DISPLAY=:0 XAUTHORITY=/run/user/$(id -u)/gdm/Xauthority MUJOCO_GL=glfw \
    PYTHONPATH=src /usr/bin/python3 scripts/view_go2_gait.py \
        --config config/go2_loopback.yaml --seconds 15

退出码：0 正常结束；1 用法错误；2 声明非法；3 引用完整性失败；4 后端装配失败；5 步态执行被拒。
"""

from __future__ import annotations

import argparse
import datetime
import json
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from iraf_adapters.factory import KNOWN_BACKENDS, load_backend  # noqa: E402
from iraf_adapters.mujoco.viewer_runner import (  # noqa: E402
    DISPLAY_INTERACTIVE,
    SnapshotMirror,
    camera_settings,
    display_environment,
    resolve_display_mode,
)
from iraf_adapters.unitree import gait  # noqa: E402
from iraf_adapters.unitree import quadruped as quadruped_contract  # noqa: E402
from iraf_adapters.unitree import unitree_go2  # noqa: E402
from iraf_core.authority import ControlAuthorityManager  # noqa: E402
from iraf_core.profile import ProfileError, load_robot_profile, load_safety_policy  # noqa: E402
from iraf_skills import quadruped as quadruped_skills  # noqa: E402

EXIT_OK, EXIT_USAGE, EXIT_DECLARATION, EXIT_REFERENCE, EXIT_BACKEND, EXIT_REJECTED = 0, 1, 2, 3, 4, 5
REPORT_SCHEMA = "iraf.go2-gait-view/v1"
#: 租约 TTL 余量：显示路径的 TTL 只需覆盖一次步态执行（声明时长 × 余量 + 固定余量）。
#: 这不是"声明事实"，而是运行时安全余量，因此写在这里而不是配置里。
LEASE_TTL_MARGIN = 3.0
LEASE_TTL_FLOOR_S = 30.0


def _dig(document, dotted, label="声明"):
    node = document
    for part in dotted.split("."):
        if not isinstance(node, dict) or part not in node:
            raise quadruped_contract.DeclarationError("%s 缺少键: %s" % (label, dotted))
        node = node[part]
    return node


def _resolve(root, value):
    path = Path(value)
    return path if path.is_absolute() else Path(root) / path


def main(argv=None):
    parser = argparse.ArgumentParser(description="Go2 步态显示/演示（不是验收）")
    parser.add_argument("--config", type=Path, default=Path("config/go2_loopback.yaml"))
    parser.add_argument("--root", type=Path, default=None)
    parser.add_argument("--seconds", type=float, default=15.0, help="窗口保持秒数（到期自动关闭）")
    parser.add_argument("--render-hz", type=float, default=None, help="渲染频率（缺省取声明 render.render_hz）")
    parser.add_argument("--report", type=Path, default=None, help="显示证据输出路径")
    parser.add_argument("--display", choices=("auto", "none"), default="auto",
                        help="auto=有显示设备即开窗；none=只跑步态不开窗（用于无人值守自查）")
    args = parser.parse_args(argv)

    root = args.root or unitree_go2.repo_root()
    config_path = _resolve(root, args.config)
    if not config_path.is_file():
        print("用法错误：声明文件不存在: %s" % config_path, file=sys.stderr)
        return EXIT_USAGE

    try:
        declaration, _ = unitree_go2.load_declaration(config_path)
        profile_path = _resolve(root, _dig(declaration, "robot.profile"))
        safety_path = _resolve(root, _dig(declaration, "skills.safety_policy"))
        backend_key = str(_dig(declaration, "robot.backend"))
        for path, label in ((profile_path, "Profile"), (safety_path, "安全策略")):
            if not path.is_file():
                print("引用完整性失败：%s 不存在: %s" % (label, path), file=sys.stderr)
                return EXIT_REFERENCE
        profile = load_robot_profile(profile_path)
        safety = load_safety_policy(safety_path)
        movement_limits = quadruped_skills.load_movement_limits(safety_path)
        params = gait.load_gait_declaration(declaration, profile.joints)
        render = _dig(declaration, "render")
        camera, width, height, render_hz_declared = (
            str(render["camera"]), int(render["width_px"]), int(render["height_px"]),
            float(render["render_hz"]),
        )
    except quadruped_contract.ModelUnavailableError as exc:
        print("引用完整性失败：%s" % exc, file=sys.stderr)
        return EXIT_REFERENCE
    except ProfileError as exc:
        print("声明非法：%s" % exc, file=sys.stderr)
        return EXIT_DECLARATION
    except quadruped_skills.SkillContractError as exc:
        print("声明非法（移动边界/停止语义）：%s" % exc, file=sys.stderr)
        return EXIT_DECLARATION
    except quadruped_contract.DeclarationError as exc:
        print("声明非法：%s" % exc, file=sys.stderr)
        return EXIT_DECLARATION

    authority = ControlAuthorityManager()
    try:
        backend = load_backend(KNOWN_BACKENDS[backend_key], str(config_path), profile, authority)
    except Exception as exc:  # noqa: BLE001 装配失败即显式退出，不返回半成品
        print("后端装配失败：%s" % exc, file=sys.stderr)
        return EXIT_BACKEND

    duration_s = float(params["verification"]["duration_s"])
    lease = authority.acquire(
        profile.name + "-mujoco",
        "gait-view-demo",
        ttl_seconds=max(LEASE_TTL_FLOOR_S, duration_s * LEASE_TTL_MARGIN),
    )

    holder = {"result": None, "error": None}
    thread = threading.Thread(
        target=lambda: _run_trot(backend, lease, holder), name="gait-view-trot", daemon=True
    )
    thread.start()

    # 注意：resolve_display_mode() 返回 (mode, reason, extra) 元组（实测踩过：当字符串比较会静默跳过开窗分支）
    resolved = resolve_display_mode() if args.display == "auto" else ("none", "显式 --display none", None)
    mode = resolved[0] if isinstance(resolved, tuple) else str(resolved)
    env = display_environment()
    frames = 0
    view = None            # 交互窗口的自由相机初值（来自 profile 的 camera 段；非交互时为 None）
    if mode == DISPLAY_INTERACTIVE:
        import mujoco.viewer

        render_hz = float(args.render_hz or render_hz_declared)
        snapshot = SnapshotMirror(backend.model)
        if not getattr(profile, "camera", None):
            # 铁律 6.2：不靠隐藏默认值。缺声明 ⇒ 显式失败并告诉用户该声明什么。
            print("声明非法：profile %s 缺 `camera` 段（交互窗口的自由相机初值必须来自声明；"
                  "所需键：lookat_m / distance_m / azimuth_deg / elevation_deg）" % profile_path,
                  file=sys.stderr)
            return EXIT_DECLARATION
        view = camera_settings(profile)
        with mujoco.viewer.launch_passive(backend.model, snapshot.refresh(backend)) as viewer:
            # 自由相机（**可鼠标旋转/缩放**）：只**初始化**一次，之后不再每帧覆盖 —— 原实现每帧
            # 设 `cam.type = mjCAMERA_FIXED` 指向声明相机，按 MuJoCo 语义固定相机禁用鼠标旋转，
            # 用户反馈"视角不能旋转"即此（2026-09-23）。
            viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
            viewer.cam.lookat[:] = view["lookat_m"]
            viewer.cam.distance = view["distance_m"]
            viewer.cam.azimuth = view["azimuth_deg"]
            viewer.cam.elevation = view["elevation_deg"]
            if env.get("warning"):
                print("[viewer] 注意：" + env["warning"], flush=True)
            print("VIEWER_CAMERA free lookat=%s distance=%.3f azimuth=%.1f elevation=%.1f"
                  % (view["lookat_m"], view["distance_m"], view["azimuth_deg"],
                     view["elevation_deg"]), flush=True)
            started = time.monotonic()
            period = 1.0 / max(1.0, render_hz)
            while viewer.is_running():
                snapshot.refresh(backend)
                viewer.sync()
                frames += 1
                if not thread.is_alive() and time.monotonic() - started >= float(args.seconds):
                    break
                time.sleep(period)
    thread.join(timeout=max(60.0, duration_s * LEASE_TTL_MARGIN + 30.0))

    report = {
        "schema_version": REPORT_SCHEMA,
        "display_only": True,
        "simulation": True,
        "generated_at": datetime.datetime.now().astimezone().isoformat(timespec="seconds"),
        "robot": {"id": str(_dig(declaration, "robot.id")), "profile": str(profile_path.name)},
        "config": str(config_path),
        "gait": {
            "kind": params["kind"], "frequency_hz": params["frequency_hz"],
            "period_s": params["period_s"], "step_height_m": params["step_height_m"],
            "duty_factor": params["duty_factor"], "duration_s": duration_s,
        },
        "display": {"mode": mode, "env": env, "frames": frames,
                    "camera": camera, "resolution_px": [width, height],
                    "free_camera": view},
        "trot": {
            "finished": not thread.is_alive(),
            "error": holder["error"],
            "samples": len((holder["result"] or {}).get("samples") or []),
        },
        "note": "显示/演示证据：数字不是验收数字；验收见 scripts/verify_go2_trot_in_place.py",
    }
    report_path = _resolve(root, args.report or "build/iraf-24h-2/22-gait-view/report.json")
    report_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")

    print("显示模式：%s；窗口帧数：%d；步态执行完成：%s；采样 %d 条"
          % (mode, frames, report["trot"]["finished"], report["trot"]["samples"]))
    if holder["error"]:
        print("步态执行被拒/失败：%s" % holder["error"], file=sys.stderr)
        return EXIT_REJECTED
    print("显示证据：%s" % report_path)
    print("提醒：本入口只用于观看；能力验收以 verify_go2_trot_in_place.py 的数字为准。")
    return EXIT_OK


def _run_trot(backend, lease, holder):
    try:
        holder["result"] = backend.trot_in_place(lease)
    except Exception as exc:  # noqa: BLE001
        holder["error"] = type(exc).__name__ + ": " + str(exc)


if __name__ == "__main__":
    raise SystemExit(main())
