#!/usr/bin/env python3
"""停靠目标帧门禁的验收入口（正反两向）：

  正向：**生产声明**里的目标帧必须**世界固定**（所属 body = worldbody，不在机器人子树内）
        —— 只读模型，不跑仿真；
  负向：夹具声明（把目标帧改回挂在躯干上的 `tray_frame`）必须在**执行前**显式失败，
        且零物理步进（`data.time` / `ncon` 前后不变）。

为什么要正反两向（2026-09-24 起）：本脚本原先只有负向（那时生产声明指向自指帧 `tray_frame`，
断言"必被拒"）。站位帧落地后生产目标已合法，旧的单向断言会变成**过期门禁**（永远失败）——
门禁必须跟着事实走：正向证明"合法目标不被误拦"，负向证明"自指目标一定被拦"。

用法（仓库根）：PYTHONPATH=src MUJOCO_GL=glfw /usr/bin/python3 scripts/verify_dock_target_frame.py
退出码：0 = 两向都对；1 = 有判据失败。
"""
import copy
import json
import math
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

import yaml  # noqa: E402

from iraf_adapters.factory import KNOWN_BACKENDS, load_backend  # noqa: E402
from iraf_adapters.unitree.dock import DockDeclarationError  # noqa: E402
from iraf_adapters.unitree.mpc.state_bridge import trunk_subtree_bodies  # noqa: E402
from iraf_core.authority import ControlAuthorityManager  # noqa: E402
from iraf_core.profile import load_robot_profile  # noqa: E402

CONFIG = ROOT / "config/go2_loopback.yaml"
REPORT = ROOT / "build/acceptance/go2-dock-guard/report.json"
#: 负向夹具用的目标帧：挂在四足躯干上的托盘参考系（与机身同一刚体 ⇒ 自指量）
SELF_FRAME = "tray_frame"


def main():
    cfg = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))
    profile = load_robot_profile(ROOT / cfg["robot"]["profile"])
    checks = []

    # ---- 正向：生产声明的目标帧必须世界固定（只读模型）----
    authority = ControlAuthorityManager()
    backend = load_backend(KNOWN_BACKENDS[cfg["robot"]["backend"]], str(CONFIG), profile, authority)
    declared = str(cfg["dock_for_handoff"]["target_frame"])
    model, mj = backend.model, backend.mujoco
    site = mj.mj_name2id(model, mj.mjtObj.mjOBJ_SITE, declared)
    body = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, declared)
    frame_kind = "site" if site >= 0 else ("body" if body >= 0 else None)
    frame_body = (int(model.site_bodyid[site]) if frame_kind == "site"
                  else (int(body) if frame_kind == "body" else None))
    # 躯干 body 名是 **Profile** 的构建期声明（`spec.model.trunk_body`），不在 config 里
    profile_doc = yaml.safe_load((ROOT / cfg["robot"]["profile"]).read_text(encoding="utf-8"))
    trunk_name = str(profile_doc["spec"]["model"]["trunk_body"])
    trunk = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, trunk_name)
    robot_bodies = set(trunk_subtree_bodies(model, trunk))
    resolved = (declared, frame_kind, frame_body, robot_bodies, trunk)
    checks.append(("正向：生产声明 %s 在世界固定帧里可解析" % declared, frame_body is not None))
    checks.append(("正向：%s 属于 worldbody（世界固定）" % declared, frame_body == 0))
    checks.append(("正向：%s 不在机器人子树内" % declared, frame_body not in robot_bodies))

    # ---- 负向：把目标帧改回挂在躯干上的自指帧 ⇒ 执行前显式失败 + 零物理步进 ----
    fixture = copy.deepcopy(cfg)
    fixture["dock_for_handoff"]["target_frame"] = SELF_FRAME
    # ⚠ 必须用**夹具后端**（`load_backend` 把 config 原样透传给 `from_config`，故可直接传 dict）：
    # 只改本地 dict 而继续用生产后端 ⇒ 负向用例其实跑的是合法目标（实测：白跑 15 s 停靠）。
    fixture_authority = ControlAuthorityManager()
    fixture_backend = load_backend(KNOWN_BACKENDS[cfg["robot"]["backend"]], fixture, profile,
                                  fixture_authority)
    before = (float(fixture_backend.data.time), int(fixture_backend.data.ncon))
    lease = fixture_authority.acquire(profile.name + "-mujoco", "dock-target-frame-verify",
                                      ttl_seconds=60.0)
    started = time.perf_counter()
    error = None
    try:
        fixture_backend.dock_for_handoff(lease=lease, position_tolerance_m=0.03,
                                         yaw_tolerance_rad=math.radians(2.0), max_final_speed_mps=0.05)
    except DockDeclarationError as exc:
        error = exc
    except Exception as exc:  # noqa: BLE001 —— 其他异常也要看见（不吞）
        error = exc
    finally:
        fixture_authority.release(lease)
    wall = time.perf_counter() - started
    after = (float(fixture_backend.data.time), int(fixture_backend.data.ncon))
    message = str(error) if error is not None else ""
    # 夹具生效性：先证明"夹具真的换成了自指帧"，否则这条负向用例可能什么都没测到
    fixture_ok = str(fixture["dock_for_handoff"]["target_frame"]) == SELF_FRAME and SELF_FRAME != declared
    checks.append(("负向夹具生效（目标帧被换成 %s）" % SELF_FRAME, fixture_ok))
    checks.append(("负向：自指目标帧被显式拒绝（DockDeclarationError）", isinstance(error, DockDeclarationError)))
    checks.append(("负向：报错含「刚性挂在机器人」判定", "刚性挂在机器人" in message))
    checks.append(("负向：零物理步进（data.time 不变）", before[0] == after[0]))
    checks.append(("负向：零接触计算（ncon 不变）", before[1] == after[1]))

    report = {
        "simulation": True,
        "criterion": "生产目标帧必须世界固定；自指目标帧必须在执行前被拒（零物理步进）",
        "declaration_config": str(CONFIG.relative_to(ROOT)),
        "declared_target_frame": declared,
        "resolved_frame_kind": frame_kind,
        "resolved_frame_body": frame_body,
        "robot_trunk_body": trunk,
        "negative_fixture_frame": SELF_FRAME,
        "negative_error": dict(type=type(error).__name__ if error is not None else None, message=message),
        "negative_wall_seconds": wall,
        "data_time_before_s": before[0], "data_time_after_s": after[0],
        "ncon_before": before[1], "ncon_after": after[1],
        "checks": [{"name": n, "passed": bool(p)} for n, p in checks],
    }
    report["passed"] = all(p for _n, p in checks)
    REPORT.parent.mkdir(parents=True, exist_ok=True)
    REPORT.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")

    for name, passed in checks:
        print("%s %s" % ("通过" if passed else "失败", name))
    print("\n生产目标帧 %s：kind=%s body=%s（躯干 body=%s，机器人子树 %d 个 body）"
          % (declared, frame_kind, frame_body, trunk, len(robot_bodies)))
    print("负向夹具 %s：%s（墙钟 %.3f s）" % (SELF_FRAME, type(error).__name__ if error else "-", wall))
    if message:
        print("报错原文：%s" % message[:200])
    print("\n报告：%s" % REPORT.relative_to(ROOT))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
