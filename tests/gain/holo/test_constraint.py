import numpy as np
import pytest

from pyautd3.gain.holo import EmissionConstraint, Naive, NalgebraBackend, pascal
from tests.test_autd import create_controller


@pytest.mark.asyncio()
async def test_constraint():
    with await create_controller() as autd:
        backend = NalgebraBackend()
        g = (
            Naive(backend)
            .add_focus(autd.geometry.center + np.array([30, 0, 150]), 5e3 * pascal)
            .add_focus(autd.geometry.center + np.array([-30, 0, 150]), 5e3 * pascal)
            .with_constraint(EmissionConstraint.uniform(0x80))
        )
        assert await autd.send_async(g)
        for dev in autd.geometry:
            intensities, phases = autd.link.intensities_and_phases(dev.idx, 0)
            assert np.all(intensities == 0x80)
            assert not np.all(phases == 0)

        g = (
            Naive(backend)
            .add_focus(autd.geometry.center + np.array([30, 0, 150]), 5e3 * pascal)
            .add_focus(autd.geometry.center + np.array([-30, 0, 150]), 5e3 * pascal)
            .with_constraint(EmissionConstraint.normalize())
        )
        assert await autd.send_async(g)
        for dev in autd.geometry:
            intensities, phases = autd.link.intensities_and_phases(dev.idx, 0)
            assert not np.all(intensities == 0)
            assert not np.all(phases == 0)

        g = (
            Naive(backend)
            .add_focus(autd.geometry.center + np.array([30, 0, 150]), 5e3 * pascal)
            .add_focus(autd.geometry.center + np.array([-30, 0, 150]), 5e3 * pascal)
            .with_constraint(EmissionConstraint.clamp(67, 85))
        )
        assert await autd.send_async(g)
        for dev in autd.geometry:
            intensities, phases = autd.link.intensities_and_phases(dev.idx, 0)
            assert np.all(intensities >= 67)
            assert np.all(intensities <= 85)
            assert not np.all(phases == 0)

        g = (
            Naive(backend)
            .add_focus(autd.geometry.center + np.array([30, 0, 150]), 5e3 * pascal)
            .add_focus(autd.geometry.center + np.array([-30, 0, 150]), 5e3 * pascal)
            .with_constraint(EmissionConstraint.dont_care())
        )
        assert await autd.send_async(g)
        for dev in autd.geometry:
            intensities, phases = autd.link.intensities_and_phases(dev.idx, 0)
            assert not np.all(intensities == 0)
            assert not np.all(phases == 0)
