import numpy as np

from pyautd3 import Controller, Focus, FocusSTM, GainSTM, Hz, Silencer, Static


def stm_focus(autd: Controller) -> None:
    config = Silencer.disable()
    autd.send(config)

    m = Static()

    radius = 30.0
    size = 200
    center = autd.geometry.center + np.array([0.0, 0.0, 150.0])
    stm = FocusSTM.from_freq(1.0 * Hz).add_foci_from_iter(
        center + radius * np.array([np.cos(theta), np.sin(theta), 0]) for theta in (2.0 * np.pi * i / size for i in range(size))
    )

    autd.send(m, stm)


def stm_gain(autd: Controller) -> None:
    config = Silencer.disable()
    autd.send(config)

    m = Static()

    radius = 30.0
    size = 50
    center = autd.geometry.center + np.array([0.0, 0.0, 150.0])
    stm = GainSTM.from_freq(1.0 * Hz).add_gains_from_iter(
        Focus(center + radius * np.array([np.cos(theta), np.sin(theta), 0])) for theta in (2.0 * np.pi * i / size for i in range(size))
    )

    autd.send(m, stm)
