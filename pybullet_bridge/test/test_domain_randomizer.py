"""Unit tests for domain randomization sampling."""

from pybullet_bridge.domain_randomizer import DomainRandomizer, DomainRandomizerConfig


def test_sample_episode_perturbs_damping_and_friction():
    randomizer = DomainRandomizer(DomainRandomizerConfig(random_seed=7))
    sample = randomizer.sample_episode(num_joints=2)

    assert len(sample.joint_damping) == 2
    assert len(sample.joint_friction) == 2
    assert 0.0 <= sample.payload_mass_kg <= 0.5
    assert 0.0 <= sample.actuator_delay_ms <= 50.0
    assert sample.joint_damping != [0.04, 0.04] or sample.joint_friction != [0.1, 0.1]


def test_episode_resample_changes_with_counter():
    randomizer = DomainRandomizer(DomainRandomizerConfig(random_seed=1))
    first = randomizer.sample_episode(2)
    second = randomizer.sample_episode(2)
    assert first.episode_seed != second.episode_seed
    assert (
        first.joint_damping != second.joint_damping
        or first.payload_mass_kg != second.payload_mass_kg
        or first.actuator_delay_ms != second.actuator_delay_ms
    )


def test_inject_shift_scales_damping():
    randomizer = DomainRandomizer(DomainRandomizerConfig(random_seed=3))
    randomizer.set_inject_shift('joint_damping', 30.0, 10.0, now_mono=0.0)
    sample = randomizer.sample_episode(2)
    unshifted = [v / 1.3 for v in sample.joint_damping]
    assert all(0.028 <= v <= 0.052 for v in unshifted)


def test_zero_strength_returns_nominal():
    randomizer = DomainRandomizer(DomainRandomizerConfig(
        random_seed=5, randomization_strength=0.0))
    sample = randomizer.sample_episode(2)
    assert sample.joint_damping == [0.04, 0.04]
    assert sample.joint_friction == [0.1, 0.1]
    assert sample.payload_mass_kg == 0.0
    assert sample.actuator_delay_ms == 0.0
