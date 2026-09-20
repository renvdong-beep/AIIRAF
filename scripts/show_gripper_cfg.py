"""打印场景 report 的 gripper 段，确认关节名已翻译为执行器名。"""

import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def main():
    report = json.loads(
        (ROOT / "build/models/ur5-pick-scene.json").read_text(encoding="utf-8")
    )
    print(json.dumps(report["gripper"], ensure_ascii=False, indent=2))
    print("\narm_gains_injected:", report["arm_gains_injected"])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
