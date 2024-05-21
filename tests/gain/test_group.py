from typing import TYPE_CHECKING

import numpy as np
import pytest

from pyautd3 import Controller, Device, Phase, Segment, Transducer
from pyautd3.autd_error import AUTDError
from pyautd3.driver.firmware.fpga.emit_intensity import EmitIntensity
from pyautd3.gain import Group, Null, Uniform
from tests.test_autd import create_controller

if TYPE_CHECKING:
    from pyautd3.link.audit import Audit


def test_group():
    autd: Controller[Audit]
    with create_controller() as autd:
        cx = autd.geometry.center[0]

        autd.send(
            Group(lambda _, tr: "uniform" if tr.position[0] < cx else "null")
            .set("uniform", Uniform(EmitIntensity(0x80)).with_phase(Phase(0x90)))
            .set("null", Null()),
        )
        for dev in autd.geometry:
            intensities, phases = autd.link.drives(dev.idx, Segment.S0, 0)
            for tr in dev:
                if tr.position[0] < cx:
                    assert np.all(intensities[tr.idx] == 0x80)
                    assert np.all(phases[tr.idx] == 0x90)
                else:
                    assert np.all(intensities[tr.idx] == 0)
                    assert np.all(phases[tr.idx] == 0)

        autd.send(
            Group(lambda _, tr: "uniform" if tr.position[0] < cx else None).set(
                "uniform",
                Uniform(EmitIntensity(0x80)).with_phase(Phase(0x90)),
            ),
        )
        for dev in autd.geometry:
            intensities, phases = autd.link.drives(dev.idx, Segment.S0, 0)
            for tr in dev:
                if tr.position[0] < cx:
                    assert np.all(intensities[tr.idx] == 0x80)
                    assert np.all(phases[tr.idx] == 0x90)
                else:
                    assert np.all(intensities[tr.idx] == 0)
                    assert np.all(phases[tr.idx] == 0)


def test_group_unknown_key():
    autd: Controller[Audit]
    with create_controller() as autd, pytest.raises(AUTDError, match="Unknown group key"):
        autd.send(Group(lambda _, _tr: "null").set("uniform", Uniform(EmitIntensity(0x80)).with_phase(Phase(0x90))).set("null", Null()))


def test_group_unspecified_key():
    autd: Controller[Audit]
    with create_controller() as autd, pytest.raises(AUTDError, match="Unspecified group key"):
        autd.send(Group(lambda _, _tr: "null"))


def test_group_check_only_for_enabled():
    autd: Controller[Audit]
    with create_controller() as autd:
        autd.geometry[0].enable = False

        check = np.zeros(autd.geometry.num_devices, dtype=bool)

        def f(dev: Device, _tr: Transducer) -> int:
            check[dev.idx] = True
            return 0

        autd.send(Group(f).set(0, Uniform(EmitIntensity(0x80)).with_phase(Phase(0x90))))

        assert not check[0]
        assert check[1]

        intensities, phases = autd.link.drives(0, Segment.S0, 0)
        assert np.all(intensities == 0)
        assert np.all(phases == 0)

        intensities, phases = autd.link.drives(1, Segment.S0, 0)
        assert np.all(intensities == 0x80)
        assert np.all(phases == 0x90)
