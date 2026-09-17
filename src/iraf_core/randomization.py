"""场景随机化采样器：机器人无关的"随机摆放"能力。

设计约束（来自 config/piper_multi_target.yaml 的既有结论与 AGENTS.md 铁律）：

1. **本期只随机 xy**：yaw 固定为 0。原因不是几何上不可行（正方体 yaw 有 90°
   对称等价类），而是夹爪朝向与目标 yaw 的对齐接线尚未实现——
   verify_piper_multi_target_pick.py 会写入 grasp.yaw_deg，但
   build_piper_baseline / build_piper_pick_scene 都不读取它，
   即随机 yaw 不会改变抓取几何。该接线留待 pick_object 六阶段重构时一并处理；
   在此之前让 yaw 参与随机只会产生"看起来随机、实际无效"的假象。
2. **不随机 roll/pitch**：倾斜目标的姿态对齐尚未实现，
   随机倾斜会被 validate_grasp_pose 全量拦截；
3. **z 贴台面**：由 workbench.top_z_m + half_size 推出，不独立随机；
4. **确定性可复现**：所有随机性经单一 numpy Generator，种子由调用方给定，
   同一 seed 必须产出完全相同的采样序列；
5. **显式失败**：采样耗尽仍不满足门禁时抛错，不静默兜底。
6. **不做采样阶段预筛**：可达性/退化构型不预先排除，一律交给下游门禁判定，
   失败原因按类记录并重采样（上限 max_attempts），
   以保证统计口径是"真实一次采样"而不是"挑过的采样"。

本模块不依赖 MuJoCo，也不依赖任何机器人专有名称，只做几何采样与门禁记账，
因而可被不同机器人的适配层复用。
"""

import json
import math
from dataclasses import dataclass, field
from pathlib import Path

YAW_EQUIVALENCE_CLASS_DEG = (0.0, 90.0, 180.0, 270.0)


class RandomizationError(ValueError):
    """随机化采样层的显式失败。"""


@dataclass(frozen=True)
class RandomizationSpec:
    """随机化范围声明。半径单位米，yaw 以度为单位。"""

    xy_radius_m: float
    #: 本期 yaw 固定为 0（夹爪朝向对齐接线未实现，见模块文档第 1 条）。
    #: 保留该字段以便接线完成后直接启用，不必改动调用方。
    yaw_choices_deg: tuple = (0.0,)
    max_attempts: int = 32
    description: str = ""

    def __post_init__(self):
        if self.xy_radius_m <= 0:
            raise RandomizationError("xy_radius_m 必须为正数")
        if not self.yaw_choices_deg:
            raise RandomizationError("yaw_choices_deg 不能为空")
        if self.max_attempts < 1:
            raise RandomizationError("max_attempts 必须为正整数")

    @classmethod
    def from_config(cls, config, half_size_m):
        """从基线配置的 grasp.randomization 段解析；未声明时给出默认声明。

        默认语义（用户已确认）：以标称抓取点为圆心，xy 半径 = 物品直径，
        即 2 * half_size_m；z 由台面推出；yaw 本期固定 0。
        """
        block = ((config or {}).get("grasp") or {}).get("randomization") or {}
        radius = block.get("xy_radius_m")
        if radius is None:
            # 标称"物品直径的 2 倍范围"，以半径表达即为物品直径本身。
            radius = 2.0 * float(half_size_m)
        # yaw 未显式声明时按本期口径固定为 0，而不是取 90° 等价类：
        # 接线缺失时随机 yaw 不改变抓取几何，只会制造"已随机"的假象。
        choices = block.get("yaw_choices_deg")
        if choices is None:
            choices = (float(block.get("yaw_deg", 0.0)),)
        return cls(
            xy_radius_m=float(radius),
            yaw_choices_deg=tuple(float(value) for value in choices),
            max_attempts=int(block.get("max_attempts", 32)),
            description=str(block.get("description", "")),
        )

    @property
    def yaw_randomized(self):
        return len(set(self.yaw_choices_deg)) > 1

    def to_dict(self):
        return {
            "xy_radius_m": self.xy_radius_m,
            "yaw_choices_deg": [float(value) for value in self.yaw_choices_deg],
            "yaw_randomized": self.yaw_randomized,
            "max_attempts": self.max_attempts,
            "description": self.description,
            "randomized_axes": ["xy"],
            "fixed_axes": ["z", "yaw", "roll", "pitch"],
            "deferred_axes": {
                "yaw": "夹爪朝向对齐接线未实现，留待 pick_object 六阶段重构",
            },
        }


@dataclass
class Sample:
    """一次随机采样结果（几何量，不含任何仿真真值）。"""

    index: int
    attempt: int
    x_m: float
    y_m: float
    z_m: float
    yaw_deg: float
    nominal_x_m: float
    nominal_y_m: float
    offset_m: float
    diagnostics: dict = field(default_factory=dict)

    def to_dict(self):
        return {
            "index": self.index,
            "attempt": self.attempt,
            "x_m": round(self.x_m, 9),
            "y_m": round(self.y_m, 9),
            "z_m": round(self.z_m, 9),
            "yaw_deg": round(self.yaw_deg, 6),
            "offset_m": round(self.offset_m, 9),
            "nominal_x_m": round(self.nominal_x_m, 9),
            "nominal_y_m": round(self.nominal_y_m, 9),
            "diagnostics": self.diagnostics,
        }


class UniformDiscSampler:
    """确定性的圆盘均匀采样器（面积均匀，不是半径均匀）。

    半径均匀采样会让点向圆心聚集（周长随半径减小），
    面积均匀需要 r = R * sqrt(u)，这是本类存在的唯一原因。
    """

    def __init__(self, spec: RandomizationSpec, seed: int):
        self.spec = spec
        self.seed = int(seed)
        self._rng = _generator(self.seed)
        self._draws = 0

    @property
    def draws(self):
        return self._draws

    def draw(self, nominal_xy, top_z, half_size_m, index, attempt):
        """以 nominal_xy 为圆心采一个点；z 严格贴台面。"""
        if len(nominal_xy) != 2:
            raise RandomizationError("nominal_xy 必须是 2 个数值")
        radius = self.spec.xy_radius_m
        theta = float(self._rng.uniform(0.0, 2.0 * math.pi))
        # 面积均匀：r = R * sqrt(u)
        r = radius * math.sqrt(float(self._rng.uniform(0.0, 1.0)))
        yaw = float(self._rng.choice(self.spec.yaw_choices_deg))
        self._draws += 1
        offset_x = r * math.cos(theta)
        offset_y = r * math.sin(theta)
        return Sample(
            index=int(index),
            attempt=int(attempt),
            x_m=float(nominal_xy[0]) + offset_x,
            y_m=float(nominal_xy[1]) + offset_y,
            z_m=float(top_z) + float(half_size_m),
            yaw_deg=yaw,
            nominal_x_m=float(nominal_xy[0]),
            nominal_y_m=float(nominal_xy[1]),
            offset_m=float(r),
        )


def _generator(seed):
    """独立的 Generator 工厂，便于测试替换而不触及全局随机状态。"""
    import numpy as np

    return np.random.default_rng(int(seed))


def euler_deg_for_yaw(yaw_deg):
    """把 yaw 转成场景生成器使用的 [rx, ry, rz]（度）。

    build_piper_pick_scene._euler_zyx_quat 走的是 mju_euler2Quat(..., "xyz")，
    等价于 Rz*Ry*Rx，因此纯 yaw 只需 rz = yaw，rx = ry = 0。
    """
    return [0.0, 0.0, float(yaw_deg)]


#: 拒绝原因分类。顺序即匹配优先级：越具体的判定放在越前，
#: 避免"退化构型"被更宽泛的"对齐不足"吞掉。
REJECTION_PATTERNS = (
    ("IK_DEGENERATE", ("退化", "degenerate")),
    ("IK_NOT_CONVERGED", ("未收敛", "not converge", "配平")),
    ("ALIGNMENT_TOLERANCE", ("未到达目标抓取位姿", "center_distance")),
    ("POSITION_TOLERANCE", ("抓取位姿与目标位置不一致",)),
    ("FINGER_TABLE_COLLISION", ("扎入工作台", "扎进工作台")),
    ("FINGER_CONTACT", ("手指与场景发生接触", "接触")),
    ("VISION_FAILED", ("视觉", "vision", "颜色分割", "掩码")),
    ("MOTION_FAILED", ("接触力", "lift", "抬升", "bilateral")),
)


def classify_rejection(reason):
    """把下游异常文本归入稳定的失败类别，便于聚合统计与回归对比。

    未匹配到任何已知模式时返回 OTHER，并保留原文，不做静默丢弃。
    """
    text = str(reason or "")
    for label, needles in REJECTION_PATTERNS:
        if any(needle in text for needle in needles):
            return label
    return "OTHER"


def rejection_counts(records, key="rejection_category"):
    """统计一组记录里的失败类别分布（只统计非空类别）。"""
    counts = {}
    for record in records:
        label = record.get(key)
        if not label:
            continue
        counts[label] = counts.get(label, 0) + 1
    return dict(sorted(counts.items()))


def describe_distribution(values):
    """给出均值/标准差/最大/最小；空输入返回 None 而非 0，避免伪造指标。"""
    clean = [float(value) for value in values if value is not None]
    if not clean:
        return None
    count = len(clean)
    mean = sum(clean) / count
    if count > 1:
        variance = sum((value - mean) ** 2 for value in clean) / (count - 1)
        std = math.sqrt(variance)
    else:
        std = 0.0
    return {
        "count": count,
        "mean": round(mean, 9),
        "std": round(std, 9),
        "min": round(min(clean), 9),
        "max": round(max(clean), 9),
    }


def write_report(path, payload):
    """统一的报告落盘：UTF-8、ASCII 转义、末尾换行。"""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(payload, ensure_ascii=True, indent=2) + "\n", encoding="utf-8"
    )
    return path
