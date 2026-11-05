"""G1 平衡鲁棒性验收（准静态步态的基础证据）：站立 → 水平冲量 → 恢复测量。

用法：PYTHONPATH=src python3 scripts/verify_g1_balance.py
判据：恢复窗口内最大倾角 ≤ 5°、最终不翻倒。全程 simulation=true。
"""
import json
import sys

import mujoco
import numpy as np

from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
MODEL = str(ROOT / "vendor" / "unitree_g1" / "scene.xml")


def _tilt(data, body_id):
    w, x, y, z = data.xquat[body_id]
    return float(np.degrees(np.arccos(np.clip(abs(w * w - x * x - y * y + z * z), -1.0, 1.0))))


def main():
    m = mujoco.MjModel.from_xml_path(MODEL)
    d = mujoco.MjData(m)
    pelvis = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "pelvis")

    mujoco.mj_resetDataKeyframe(m, d, 0)
    mujoco.mj_forward(m, d)
    ctrl = np.zeros(m.nu)
    for i in range(m.nu):
        ctrl[i] = d.qpos[m.jnt_qposadr[m.actuator_trnid[i][0]]]
    d.ctrl[:] = ctrl

    dt = m.opt.timestep
    impulse_n = 30.0
    impulse_s = 0.2
    recover_s = 3.0

    def step(seconds, fx=0.0):
        for _ in range(int(seconds / dt)):
            if fx:
                d.xfrc_applied[pelvis, 0] = fx
            mujoco.mj_step(m, d)
            d.xfrc_applied[pelvis, 0] = 0.0

    step(2.0)  # 站稳
    h0 = float(d.xpos[pelvis, 2])
    step(impulse_s, fx=impulse_n)  # 冲量
    tilt_after = _tilt(d, pelvis)
    max_tilt = 0.0
    fell = False
    for _ in range(int(recover_s / dt)):
        mujoco.mj_step(m, d)
        max_tilt = max(max_tilt, _tilt(d, pelvis))
        if d.xpos[pelvis, 2] < 0.30 or _tilt(d, pelvis) > 45.0:
            fell = True
            break

    report = {
        "simulation": True,
        "capability": "balance",
        "impulse_force_n": impulse_n,
        "impulse_duration_s": impulse_s,
        "height_before_m": h0,
        "tilt_after_impulse_deg": tilt_after,
        "max_tilt_deg": max_tilt,
        "final_tilt_deg": _tilt(d, pelvis),
        "final_height_m": float(d.xpos[pelvis, 2]),
        "fell": fell,
        "passed": (not fell) and max_tilt <= 5.0,
    }
    print(json.dumps(report, ensure_ascii=False, indent=2))
    if report["passed"]:
        print("\nG1_BALANCE_PASSED", file=sys.stderr)
        return 0
    print("\nG1_BALANCE_FAILED", file=sys.stderr)
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
