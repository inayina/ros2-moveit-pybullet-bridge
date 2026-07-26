"""Deterministic M4A RiskStatus to runtime safety decision mapping."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class ProposedSafetyDecision:
    decision: str
    reason_code: str
    hold_active: bool
    request_estop: bool
    recovery_count: int


class RiskToSafetyStateMachine:
    """Map valid R-levels to RUN/HOLD/E_STOP with debounce and latch."""

    def __init__(self, healthy_recovery_count: int = 5) -> None:
        if healthy_recovery_count < 1:
            raise ValueError('healthy_recovery_count must be positive')
        self.healthy_recovery_count = int(healthy_recovery_count)
        self.hold_active = False
        self.estop_latched = False
        self.recovery_count = 0

    def update(
        self,
        *,
        level: int,
        degraded_mode: bool,
        validity: str,
        has_valid_sources: bool,
        e_stop_active: bool,
    ) -> ProposedSafetyDecision:
        if self.estop_latched:
            return self._decision('E_STOP', 'risk_r3_latched', True, False)
        if e_stop_active or int(level) >= 3:
            self.estop_latched = True
            self.hold_active = True
            self.recovery_count = 0
            return self._decision('E_STOP', 'risk_r3_estop', True, True)
        if not has_valid_sources or validity in {'STALE', 'UNAVAILABLE', 'ERROR'}:
            self.hold_active = True
            self.recovery_count = 0
            return self._decision('HOLD', 'risk_sources_unavailable', True, False)
        if degraded_mode or int(level) >= 2:
            self.hold_active = True
            self.recovery_count = 0
            return self._decision('HOLD', 'risk_r2_hold', True, False)
        if self.hold_active:
            self.recovery_count += 1
            if self.recovery_count < self.healthy_recovery_count:
                return self._decision(
                    'HOLD', 'healthy_recovery_debounce', True, False
                )
            self.hold_active = False
            self.recovery_count = 0
            return self._decision('RUN', 'healthy_recovery_complete', False, False)
        reason = 'risk_r1_warning' if int(level) == 1 else 'none'
        return self._decision('RUN', reason, False, False)

    def observe_manual_estop_reset(self) -> None:
        """Clear the bridge latch only after the upstream safety latch falls."""
        self.estop_latched = False
        self.hold_active = True
        self.recovery_count = 0

    def _decision(
        self,
        decision: str,
        reason_code: str,
        hold_active: bool,
        request_estop: bool,
    ) -> ProposedSafetyDecision:
        return ProposedSafetyDecision(
            decision=decision,
            reason_code=reason_code,
            hold_active=hold_active,
            request_estop=request_estop,
            recovery_count=self.recovery_count,
        )
