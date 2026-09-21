"""[deprecated] 已由 `scripts/verify_go2_gait_in_place.py` 取代（Gait 泛化，步骤 02）。

本文件是**薄包装**：只转发参数并原样返回退出码，不含任何判定逻辑。
保留原因：既有文档/手工命令仍按旧名调用；两个实现会立刻分叉，因此这里只做转发。

用法（等价）::

    PYTHONPATH=src /usr/bin/python3 scripts/verify_go2_trot_in_place.py --config config/go2_loopback.yaml

退出码与 `scripts/verify_go2_gait_in_place.py` 完全一致（0 通过 / 1 用法 / 2 声明 /
3 引用完整性 / 4 后端装配 / 5 判据未通过）。
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from verify_go2_gait_in_place import main  # noqa: E402


if __name__ == "__main__":
    print(
        "[deprecated] scripts/verify_go2_trot_in_place.py 已由 "
        "scripts/verify_go2_gait_in_place.py 取代（步态泛化：声明决定 trot/wave）",
        file=sys.stderr,
    )
    raise SystemExit(main())
