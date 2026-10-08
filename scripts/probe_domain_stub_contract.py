#!/usr/bin/env python3
"""探针：控制域 Provider 桩是否满足框架契约（成功路径 + 拒绝路径）。

为什么需要：`src/iraf_adapters/domain_stub.py` 是双域容器化验证用的替身，
它能否被 `iraf_adapters.factory.load_backend()` 接受，**由框架自己的
verify_backend_contract() 判定**（不许我自己说"应该可以"）。

用法（仓库根）：
  PYTHONPATH=src python3 scripts/probe_domain_stub_contract.py [profile.yaml]
缺省 profile = profiles/unitree_go2_mujoco.yaml（Go2，五个能力）。
退出码：0 全部符合预期；1 有不符合项。
"""

from __future__ import annotations

import pathlib
import sys

REPO = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "src"))

from iraf_adapters.domain_stub import ControlDomainStubBackend  # noqa: E402
from iraf_adapters.factory import verify_backend_contract  # noqa: E402
from iraf_core.profile import load_robot_profile  # noqa: E402

FAIL: list = []


def check(label: str, ok: bool, detail: str = "") -> None:
    print("  [%s] %s%s" % ("✓" if ok else "✗", label, ("  —— " + detail) if detail else ""))
    if not ok:
        FAIL.append(label)


def main(argv: list) -> int:
    profile_path = argv[1] if len(argv) > 1 else "profiles/unitree_go2_mujoco.yaml"
    profile = load_robot_profile(str(REPO / profile_path))
    print("profile = %s（声明能力：%s）" % (profile_path, list(getattr(profile, "capabilities", []))))

    print("① 契约校验（框架自己的 verify_backend_contract）")
    try:
        report = verify_backend_contract(ControlDomainStubBackend, profile)
        check("装配契约通过", True, "schema=%s backend=%s" % (report.get("schema_version"), report.get("backend")))
    except Exception as error:  # noqa: BLE001
        check("装配契约通过", False, "%s: %s" % (type(error).__name__, error))

    print("② 成功路径：from_config + 五个能力各调一次")
    backend = ControlDomainStubBackend.from_config({"fault": "none"}, profile, None)
    calls = (
        ("stand", lambda: backend.stand(lease=object())),
        ("stop", lambda: backend.stop(lease=object())),
        ("locomote", lambda: backend.locomote(velocity=(0.1, 0.0, 0.0), duration_ms=1000, lease=object())),
        ("dock_for_handoff", lambda: backend.dock_for_handoff(lease=object(), position_tolerance_m=0.03,
                                                             yaw_tolerance_rad=0.034, max_final_speed_mps=0.05,
                                                             station="handoff_b")),
        ("accept_payload", lambda: backend.accept_payload("box_01", "tray_01", object())),
    )
    for name, fn in calls:
        try:
            fact = fn()
            check("%s 返回事实" % name, bool(fact.get("observed") is not None and fact.get("simulation") is True),
                  "simulation=%s timestamp_s=%s" % (fact.get("simulation"), fact.get("timestamp_s")))
        except Exception as error:  # noqa: BLE001
            check("%s 返回事实" % name, False, "%s: %s" % (type(error).__name__, error))
    inv = backend.runtime_inventory()
    check("runtime_inventory 自报已实现能力", inv.get("implemented_capabilities") == ["stand", "stop", "locomote",
                                                                                  "dock_for_handoff", "accept_payload"],
          str(inv.get("implemented_capabilities")))

    print("③ 拒绝路径（fail-closed）")
    try:
        backend.stand(lease=None)
        check("无租约被拒", False, "竟然通过了")
    except PermissionError as error:
        check("无租约被拒", True, str(error)[:60])
    try:
        ControlDomainStubBackend.from_config({"fault": "boom"}, profile, None)
        check("非法故障名被拒", False, "竟然通过了")
    except ValueError as error:
        check("非法故障名被拒", True, str(error)[:60])
    for fault in ("timeout", "unreachable"):
        bad = ControlDomainStubBackend.from_config({"fault": fault}, profile, None)
        try:
            bad.stand(lease=object())
            check("注入 %s 显式失败" % fault, False, "竟然返回了成功")
        except Exception as error:  # noqa: BLE001
            check("注入 %s 显式失败" % fault, type(error).__name__ in ("BackendTimeout", "BackendUnavailable"),
                  "%s: %s" % (type(error).__name__, error))
    # 未声明的能力必须被拒（用 engineering profile 只有 engineering.robot_adapter 做反例）
    try:
        other = load_robot_profile(str(REPO / "profiles/engineering_tooling.yaml"))
        strict = ControlDomainStubBackend.from_config({}, other, None)
        try:
            strict.stand(lease=object())
            check("未声明能力被拒", False, "竟然通过了")
        except PermissionError:
            check("未声明能力被拒", True, "该 profile 未声明 stand")
    except Exception as error:  # noqa: BLE001
        check("未声明能力被拒", False, "反例 profile 读取失败：%s" % error)

    print("\n结论：%s（%d 项不符）" % ("全部符合预期" if not FAIL else "存在不符合项", len(FAIL)))
    return 0 if not FAIL else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv))
