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
import os
import sys
import threading
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

import numpy as np  # noqa: E402

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
    parser.add_argument("--free-camera", action="store_true",
                        help="窗口用自由相机（可鼠标旋转/缩放；初值取 profile 的 camera 段）。"
                            "缺省用声明的固定相机（本机默认 GL 下最亮）")
    parser.add_argument("--walk", default=None, metavar="vx,vy,wz[;vx,vy,wz...]",
                        help="改为**行走**演示：三个规范速度字段（见 quadruped.VELOCITY_FIELDS）。"
                             "可用分号串联多段并**循环**，例如往返：`--walk 0.2,0,0;-0.2,0,0`"
                             "（前进↔倒退交替，避免单向走出台面）。"
                             "单段例：0.2,0,0 前进｜-0.2,0,0 倒退｜0,0,0.5 左转｜0,0,-0.5 右转。"
                             "走的是 `locomote` 的 MPC 正式路径（50 Hz 求解 + 100 Hz 消费）")
    parser.add_argument("--walk-seconds", type=float, default=1.5,
                        help="行走演示**每段**的时长（秒）；往返演示用默认 1.5 s 足够看到两个方向")
    args = parser.parse_args(argv)
    walk_sequence = None
    if args.walk is not None:
        walk_sequence = []
        for segment in str(args.walk).split(";"):
            parts = [chunk.strip() for chunk in segment.split(",")]
            if len(parts) != 3:
                print("用法错误：--walk 每段需要三个数 vx,vy,wz（段内用逗号、段间用分号）",
                      file=sys.stderr)
                return EXIT_USAGE
            try:
                walk_sequence.append(dict(zip(quadruped_contract.VELOCITY_FIELDS,
                                              (float(chunk) for chunk in parts))))
            except ValueError:
                print("用法错误：--walk 的元素必须是数值", file=sys.stderr)
                return EXIT_USAGE

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
    # 连续跑：显示窗口开着就把步态一轮轮接着跑（否则 10 s 仿真在墙钟上 1~2 秒就结束，
    # 使用者只看到"闪一下"，之后一直是静止末态 —— 2026-09-23 实测反馈）。
    # 租约 TTL 必须覆盖**整个显示窗口**（仍是 authority 强制的有限 TTL，不是永久租约）。
    display_s = max(0.0, float(args.seconds))
    # 租约 TTL 必须覆盖**整个显示窗口**（仍是 authority 强制的有限 TTL，不是永久租约）；
    # 行走演示是"多段循环"，每段时长来自 `--walk-seconds`，故按它估算循环次数（实测踩过：
    # 只按声明时长算 TTL ⇒ 50 段后 `lease expired` 被 authority 拦下、显示循环随之中断）。
    per_cycle_s = duration_s
    if walk_sequence:
        per_cycle_s = max(per_cycle_s, float(args.walk_seconds) * len(walk_sequence))
    # 原地踏步演示：整个显示窗用一份租约。
    # 行走演示：**不在这里取**（否则会占住资源，循环里按段 `acquire` 会直接 `LeaseConflict`）；
    # 由 `_run_gait_loop` 按段取/释放（每段一个执行、一个新 fencing token）。
    lease = None
    if not walk_sequence:
        lease = authority.acquire(
            profile.name + "-mujoco",
            "gait-view-demo",
            ttl_seconds=max(LEASE_TTL_FLOOR_S, display_s + per_cycle_s * LEASE_TTL_MARGIN),
        )

    holder = {"result": None, "error": None}
    stop = threading.Event()
    thread = threading.Thread(
        target=_run_gait_loop, args=(backend, lease, holder, stop, walk_sequence,
                                     float(args.walk_seconds)),
        kwargs={"authority": authority, "resource": profile.name + "-mujoco",
                "lease_ttl_s": max(LEASE_TTL_FLOOR_S,
                                   float(args.walk_seconds) * LEASE_TTL_MARGIN)},
        name="gait-view-gait",
        daemon=True,
    )
    thread.start()

    # 注意：resolve_display_mode() 返回 (mode, reason, extra) 元组（实测踩过：当字符串比较会静默跳过开窗分支）
    resolved = resolve_display_mode() if args.display == "auto" else ("none", "显式 --display none", None)
    mode = resolved[0] if isinstance(resolved, tuple) else str(resolved)
    env = display_environment()
    frames = 0
    view = None            # 交互窗口的自由相机初值（仅在 free 模式下使用）
    if mode == DISPLAY_INTERACTIVE:
        # GL 路径必须在**创建 GL 上下文之前**定（`import mujoco.viewer` 就会建上下文）。
        # 本机实测（docs/debug/2026-09-23-go2-viewer-3d-black-screen.md）：默认 GL 路径下窗口 3D
        # 视口几乎不亮（mean 22.2/36.4），强制 Mesa llvmpipe 后正常（95.1/42.5）⇒ 由声明开关控制。
        if bool(_dig(declaration, "render.software_gl")):
            os.environ["LIBGL_ALWAYS_SOFTWARE"] = "1"
            os.environ["GALLIUM_DRIVER"] = "llvmpipe"
            print("VIEWER_GL software (LIBGL_ALWAYS_SOFTWARE=1 GALLIUM_DRIVER=llvmpipe)",
                  flush=True)
        else:
            print("VIEWER_GL default", flush=True)
        import mujoco.viewer

        render_hz = float(args.render_hz or render_hz_declared)
        # 窗口尺寸取声明的 `render.width_px/height_px`：软件渲染（llvmpipe）在 1280×720 下
        # 每帧要数秒（实测：进程 238% CPU 但 5 秒内视口 0% 变化 ⇒ 画面近乎冻结，使用者只看到
        # "闪一下"），缩到 640×480 后单帧成本约降 4 倍。窗口尺寸本身不是物理量，取声明值即可。
        backend.model.vis.global_.offwidth = int(width)
        backend.model.vis.global_.offheight = int(height)
        snapshot = SnapshotMirror(backend.model)
        view = camera_settings(profile)          # profile 的 camera 段（free 模式初值）
        with mujoco.viewer.launch_passive(backend.model, snapshot.refresh(backend)) as viewer:
            # 窗口相机模式：**默认"声明的固定相机"**（= 2026-09-22 起可用、使用者见过的画面），
            # 可选 `--free-camera` 换成自由相机（可鼠标旋转/缩放）。
            # 为什么默认回到固定：2026-09-23 实测四格矩阵（同一场景/同一拍，见
            # docs/debug/2026-09-23-go2-viewer-3d-black-screen.md）——视口亮度：
            #   固定+默认GL 22.2 / 固定+软件GL **95.1** / 自由+默认GL 36.4 / 自由+软件GL 42.5
            if args.free_camera:
                viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
                viewer.cam.lookat[:] = view["lookat_m"]
                viewer.cam.distance = view["distance_m"]
                viewer.cam.azimuth = view["azimuth_deg"]
                viewer.cam.elevation = view["elevation_deg"]
                print("VIEWER_CAMERA free lookat=%s distance=%.3f azimuth=%.1f elevation=%.1f"
                      % (view["lookat_m"], view["distance_m"], view["azimuth_deg"],
                         view["elevation_deg"]), flush=True)
            else:
                camera_id = mujoco.mj_name2id(backend.model, mujoco.mjtObj.mjOBJ_CAMERA, camera)
                if camera_id < 0:
                    print("声明非法：render.camera=%r 在模型里不存在" % camera, file=sys.stderr)
                    return EXIT_DECLARATION
                viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FIXED
                viewer.cam.fixedcamid = camera_id
                print("VIEWER_CAMERA fixed %s（鼠标旋转不可用；要旋转加 --free-camera）" % camera,
                      flush=True)
            if env.get("warning"):
                print("[viewer] 注意：" + env["warning"], flush=True)
            started = time.monotonic()
            period = 1.0 / max(1.0, render_hz)
            while viewer.is_running():
                snapshot.refresh(backend)
                viewer.sync()
                frames += 1
                # 自限时：到点就关窗（**不看线程是否还活着**）。
                # 实测踩过：`not thread.is_alive() and elapsed >= seconds` 在"连续循环"的步态下
                # 永不成立（线程只在 `stop` 置位时退出，而 `stop` 只在跳出本循环后才置位）⇒
                # 演示窗口永不自动关闭（18:09 启动、`--seconds 150`，到 18:18 仍在跑）。
                if time.monotonic() - started >= float(args.seconds):
                    break
                time.sleep(period)
    stop.set()                       # 窗口关闭 ⇒ 停掉连续步态，不留空转线程
    thread.join(timeout=30.0)

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
            "cycles": int(holder.get("cycles") or 0),
        },
        # 演示证据：逐段位移（否则「看不看得见」只能靠肉眼，判断不出走了多少）。
        "segments": holder.get("segments") or [],
        "note": "显示/演示证据：数字不是验收数字；验收见 scripts/verify_go2_locomote.py",
    }
    report_path = _resolve(root, args.report or "build/iraf-24h-2/22-gait-view/report.json")
    report_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")

    print("显示模式：%s；窗口帧数：%d；步态执行完成：%s；采样 %d 条"
          % (mode, frames, report["trot"]["finished"], report["trot"]["samples"]))
    for item in report["segments"]:
        print("  段 %d 指令=%s 状态=%s 位移(Δx,Δy)=(%+.4f, %+.4f) m"
              % (item["segment"], item["command"], item["status"],
                 item["delta_xy_m"][0] if item["delta_xy_m"] else float("nan"),
                 item["delta_xy_m"][1] if item["delta_xy_m"] else float("nan")))
    if holder["error"]:
        print("步态执行被拒/失败：%s" % holder["error"], file=sys.stderr)
        return EXIT_REJECTED
    print("显示证据：%s" % report_path)
    print("提醒：本入口只用于观看；能力验收以 verify_go2_trot_in_place.py 的数字为准。")
    return EXIT_OK


def _run_gait_loop(backend, lease, holder, stop, walk_sequence=None, walk_seconds=1.5,
                   authority=None, resource=None, lease_ttl_s=None):
    """连续跑（原地踏步或 `locomote` 行走）直到 `stop` 置位或显式失败。

    行走演示按 `walk_sequence` **循环**给出多段速度（往返演示 = 前进段 + 倒退段），
    并在每段结束检查状态有限性：**仿真发散（NaN/Inf）必须显式停止**，不让它静默传播
    （实测：单向无限行走 33 s 后走出台面 ⇒ `Nan, Inf or huge value in QACC`）。

    每段是**一次独立执行**：`authority` 给定时按段重新取租约（新 fencing token）、段末释放。
    实测（2026-09-23）：把租约 TTL 按「仿真秒」估算覆盖不了**墙钟** —— 软件 GL + 100 Hz 控制下
    10 s 仿真约耗 40 s 墙钟（≈4×），6 段后租约过期 ⇒ 演示被 authority **正确**拦下但演示中断
    （`ControlAuthorityError: invalid fencing token`）。按段取租约同时让执行边界与证据边界对齐。
    """
    try:
        segment = 0
        while not stop.is_set():
            if walk_sequence is None:
                holder["result"] = backend.trot_in_place(lease)
            else:
                velocity = walk_sequence[segment % len(walk_sequence)]
                segment += 1
                segment_lease = lease
                if authority is not None:
                    segment_lease = authority.acquire(resource, "walk-seg-%d" % segment,
                                                      ttl_seconds=lease_ttl_s)
                try:
                    result = backend.locomote(velocity, walk_seconds * 1000.0, segment_lease)
                finally:
                    if authority is not None:
                        authority.release(segment_lease)
                holder["result"] = result
                samples = (result or {}).get("samples") or []
                entries = []
                if samples:
                    start = samples[0].get("base_position_xy_m") or (0.0, 0.0)
                    end = samples[-1].get("base_position_xy_m") or (0.0, 0.0)
                    entries = [float(end[0]) - float(start[0]), float(end[1]) - float(start[1])]
                holder.setdefault("segments", []).append({
                    "segment": segment,
                    "command": dict(velocity),
                    "status": ("SUCCEEDED" if (result or {}).get("failure") is None
                               else (result or {})["failure"]["decision"]),
                    "failure": (result or {}).get("failure"),
                    "samples": len(samples),
                    "delta_xy_m": entries,
                })
                qpos = np.asarray(backend.data.qpos, dtype=float)
                qvel = np.asarray(backend.data.qvel, dtype=float)
                if not (np.all(np.isfinite(qpos)) and np.all(np.isfinite(qvel))):
                    # 实测：发散先出现在 **QACC/QVEL**（`Nan, Inf or huge value in QACC at DOF 1`，
                    # 仿真时刻 101.0020），此时 `qpos` 仍有限 ⇒ 只查 qpos 会漏掉，继续跑下去。
                    holder["error"] = ("仿真发散（qpos/qvel 含 NaN/Inf）⇒ 显式停止；"
                                       "最常见原因：走出支撑台面边缘")
                    break
            holder["cycles"] = int(holder.get("cycles") or 0) + 1
    except Exception as exc:  # noqa: BLE001
        holder["error"] = type(exc).__name__ + ": " + str(exc)


if __name__ == "__main__":
    raise SystemExit(main())
