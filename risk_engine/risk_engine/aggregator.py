"""Aggregate six risk dimensions into composite R0-R3 score."""

from __future__ import annotations

from dataclasses import dataclass, field


DIMENSIONS = (
    'distribution_shift',
    'tracking_error',
    'dynamics_anomaly',
    'comm_health',
    'planning_failure',
    'resource_pressure',
)

RECOMMENDATIONS = {
    'distribution_shift': '检查域随机化参数范围；考虑重标定噪声模型',
    'tracking_error': '检查轨迹跟踪控制器增益与关节限位',
    'dynamics_anomaly': '检查力矩饱和与负载变化',
    'comm_health': '检查话题延迟与网络负载',
    'planning_failure': '检查规划场景与碰撞体配置',
    'resource_pressure': '降低视觉/录制负载并检查进程 CPU、内存与相机新鲜度',
}


@dataclass
class RiskWeights:
    distribution_shift: float = 0.30
    tracking_error: float = 0.25
    dynamics_anomaly: float = 0.20
    comm_health: float = 0.10
    planning_failure: float = 0.05
    resource_pressure: float = 0.10


@dataclass
class DimensionScore:
    dimension: str
    raw_score: float
    weight: float
    source_valid: bool = True
    validity: str = 'VALID'
    reason_code: str = 'none'
    provenance: str = 'legacy'

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
    validity: str = 'VALID'
    reason_code: str = 'none'
    active_dimensions: list[str] = field(default_factory=list)
    invalid_dimensions: list[str] = field(default_factory=list)


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

    def aggregate(
        self,
        raw_scores: dict[str, float],
        source_status: dict[str, dict[str, object]] | None = None,
    ) -> AggregatedRisk:
        status = source_status or {}
        valid_names = [
            name
            for name in DIMENSIONS
            if bool(status.get(name, {}).get('valid', True))
        ]
        valid_weight = sum(getattr(self.weights, name) for name in valid_names)
        dims: list[DimensionScore] = []
        for name in DIMENSIONS:
            source = status.get(name, {})
            valid = bool(source.get('valid', True))
            configured_weight = getattr(self.weights, name)
            w = (
                configured_weight / valid_weight
                if valid and valid_weight > 0.0
                else 0.0
            )
            raw = clip01(raw_scores.get(name, 0.0))
            dims.append(DimensionScore(
                dimension=name,
                raw_score=raw,
                weight=w,
                source_valid=valid,
                validity=str(source.get(
                    'validity', 'VALID' if valid else 'UNAVAILABLE'
                )),
                reason_code=str(source.get(
                    'reason_code', 'none' if valid else 'source_unavailable'
                )),
                provenance=str(source.get('provenance', 'legacy')),
            ))

        composite = sum(d.weighted_score for d in dims)
        active = [d for d in dims if d.source_valid]
        invalid = [d.dimension for d in dims if not d.source_valid]
        if not active:
            return AggregatedRisk(
                level=0,
                composite_score=0.0,
                dimensions=dims,
                validity='UNAVAILABLE',
                reason_code='no_valid_risk_sources',
                active_dimensions=[],
                invalid_dimensions=invalid,
            )
        primary = max(active, key=lambda d: d.weighted_score)
        level = score_to_level(composite, self.level_thresholds)

        return AggregatedRisk(
            level=level,
            composite_score=composite,
            dimensions=dims,
            primary_driver=primary.dimension,
            recommendation=RECOMMENDATIONS.get(primary.dimension, ''),
            validity='VALID' if not invalid else 'DEGRADED',
            reason_code='none' if not invalid else 'partial_sources_unavailable',
            active_dimensions=[d.dimension for d in active],
            invalid_dimensions=invalid,
        )
