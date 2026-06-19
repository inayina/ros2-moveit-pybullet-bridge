"""Unit tests for domain randomization noise injection."""

import numpy as np

from pybullet_bridge.noise_injector import NoiseConfig, NoiseInjector


def test_apply_preserves_length():
    injector = NoiseInjector(NoiseConfig(random_seed=42))
    positions = [0.1, 0.2, 0.3]
    velocities = [1.0, 2.0]
    noisy_pos, noisy_vel = injector.apply(positions, velocities)
    assert len(noisy_pos) == 3
    assert len(noisy_vel) == 2


def test_apply_deterministic_with_seed():
    config = NoiseConfig(position_noise_std=0.01, velocity_noise_std=0.05, random_seed=7)
    a = NoiseInjector(config)
    b = NoiseInjector(config)
    pos = [0.0, 1.0]
    vel = [0.5, -0.5]
    assert a.apply(pos, vel) == b.apply(pos, vel)


def test_apply_adds_noise():
    injector = NoiseInjector(NoiseConfig(
        position_noise_std=0.1,
        velocity_noise_std=0.1,
        random_seed=0,
    ))
    positions = [0.0, 0.0, 0.0]
    velocities = [0.0, 0.0]
    noisy_pos, noisy_vel = injector.apply(positions, velocities)
    assert noisy_pos != positions
    assert noisy_vel != velocities


def test_reseed_changes_output():
    injector = NoiseInjector(NoiseConfig(random_seed=1))
    pos = [0.0]
    vel = [0.0]
    first = injector.apply(pos, vel)
    injector.reseed(99)
    second = injector.apply(pos, vel)
    assert first != second
