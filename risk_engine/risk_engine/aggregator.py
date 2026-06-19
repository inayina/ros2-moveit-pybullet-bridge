"""Aggregate five risk dimensions into composite R0-R3 score."""

from __future__ import annotations

from dataclasses import dataclass, field


DIMENSIONS = (
    'distribution_shift',
    'tracking_error',
    'dynamics_anomaly',
    'comm_health',
    'planning_failure',
)

RECOMMENDATIONS = {
    'distribution_shift': '检查域随机化参数范围；考虑重标定噪声模型',
    'tracking_error': '检查轨迹跟踪控制器增益与关节限位',
    'dynamics_anomaly': '检查力矩饱和与负载变化',
    'comm_health': '检查话题延迟与网络负载',
    'planning_failure': '检查规划场景与碰撞体配置',
}


@dataclass
class RiskWeights:
    distribution_shift: float = 0.35
    tracking_error: float = 0.25
    dynamics_anomaly: float = 0.20
    comm_health: float = 0.10
    planning_failure: float = 0.10


@dataclass
class DimensionScore:
    dimension: str
    raw_score: float
    weight: float

    @property
    def weighted_score(self) -> float:
        return self.raw_score * self.weight


@dataclass
class AggregatedRisk:
    level: int
    composite_score: float
    dimensions: list[DimensionScore] = field(default_factory=list)
    primary_driver: str = ''
    recommendation: str = ''


def clip01(value: float) -> float:
    return max(0.0, min(1.0, value))


def score_to_level(score: float, thresholds: tuple[float, float, float]) -> int:
    if score < thresholds[0]:
        return 0
    if score < thresholds[1]:
        return 1
    if score < thresholds[2]:
        return 2
    return 3


class RiskAggregator:
    """Combine normalized dimension scores into composite risk."""

    def __init__(
        self,
        weights: RiskWeights | None = None,
        level_thresholds: tuple[float, float, float] = (0.25, 0.50, 0.75),
    ) -> None:
        self.weights = weights or RiskWeights()
        self.level_thresholds = level_thresholds

    def aggregate(self, raw_scores: dict[str, float]) -> AggregatedRisk:
        dims: list[DimensionScore] = []
        for name in DIMENSIONS:
            w = getattr(self.weights, name)
            raw = clip01(raw_scores.get(name, 0.0))
            dims.append(DimensionScore(dimension=name, raw_score=raw, weight=w))

        composite = sum(d.weighted_score for d in dims)
        primary = max(dims, key=lambda d: d.weighted_score)
        level = score_to_level(composite, self.level_thresholds)

        return AggregatedRisk(
            level=level,
            composite_score=composite,
            dimensions=dims,
            primary_driver=primary.dimension,
            recommendation=RECOMMENDATIONS.get(primary.dimension, ''),
        )
