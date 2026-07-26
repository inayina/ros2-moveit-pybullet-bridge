"""M4A state-machine tests without robot or simulator processes."""

from risk_engine.safety_bridge import RiskToSafetyStateMachine


def _update(machine, level, **overrides):
    values = {
        'level': level,
        'degraded_mode': False,
        'validity': 'VALID',
        'has_valid_sources': True,
        'e_stop_active': False,
    }
    values.update(overrides)
    return machine.update(**values)


def test_r0_r1_run_and_r2_holds() -> None:
    machine = RiskToSafetyStateMachine()
    assert _update(machine, 0).decision == 'RUN'
    assert _update(machine, 1).decision == 'RUN'
    held = _update(machine, 2)
    assert held.decision == 'HOLD'
    assert held.hold_active is True


def test_invalid_sources_fail_closed_to_hold() -> None:
    machine = RiskToSafetyStateMachine()
    result = _update(
        machine, 0, validity='UNAVAILABLE', has_valid_sources=False
    )
    assert result.decision == 'HOLD'
    assert result.reason_code == 'risk_sources_unavailable'


def test_hold_recovery_requires_consecutive_healthy_samples() -> None:
    machine = RiskToSafetyStateMachine(healthy_recovery_count=3)
    assert _update(machine, 2).decision == 'HOLD'
    assert _update(machine, 0).decision == 'HOLD'
    assert _update(machine, 0).decision == 'HOLD'
    assert _update(machine, 0).decision == 'RUN'


def test_r3_requests_estop_once_and_latches() -> None:
    machine = RiskToSafetyStateMachine()
    first = _update(machine, 3)
    repeated = _update(machine, 3)
    healthy = _update(machine, 0)
    assert first.request_estop is True
    assert repeated.request_estop is False
    assert healthy.decision == 'E_STOP'
    machine.observe_manual_estop_reset()
    assert _update(machine, 0).decision == 'HOLD'


def test_task_gt_is_not_an_input_to_safety_state_machine() -> None:
    parameters = RiskToSafetyStateMachine.update.__annotations__
    assert 'task_status' not in parameters
    assert 'task_gt' not in parameters
