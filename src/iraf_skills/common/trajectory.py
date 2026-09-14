"""平台无关的五次多项式轨迹插值。"""

import math


def quintic_position(start, end, duration_s, elapsed_s):
    """返回零速度/零加速度边界的平滑位置。"""
    duration = float(duration_s)
    elapsed = float(elapsed_s)
    if not math.isfinite(duration) or duration <= 0:
        raise ValueError("轨迹时长必须为正有限数")
    if not math.isfinite(elapsed):
        raise ValueError("轨迹时间必须为有限数")
    if len(start) != len(end) or not start:
        raise ValueError("轨迹端点维度必须一致且非空")
    if not all(math.isfinite(float(v)) for v in (*start, *end)):
        raise ValueError("轨迹端点必须为有限数")
    s = min(1.0, max(0.0, elapsed / duration))
    blend = 10.0 * s**3 - 15.0 * s**4 + 6.0 * s**5
    return [float(a) + (float(b) - float(a)) * blend for a, b in zip(start, end)]
