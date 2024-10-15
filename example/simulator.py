import numpy as np
from samples import runner  # type: ignore[import,import-not-found]

from pyautd3 import AUTD3, Controller, EulerAngles, rad
from pyautd3.link.simulator import Simulator

if __name__ == "__main__":
    with (
        Controller[Simulator]
        .builder(
            [
                # 0-row
                AUTD3([AUTD3.DEVICE_WIDTH, 0, 0]).with_rotation(EulerAngles.ZYZ(0 * rad, -np.pi / 4 * rad, 0 * rad)),
                AUTD3([0, 0, 0]).with_rotation(EulerAngles.ZYZ(0 * rad, 0 * rad, 0 * rad)),
                AUTD3([-AUTD3.DEVICE_WIDTH, 0, 0]).with_rotation(EulerAngles.ZYZ(0 * rad, 0 * rad, 0 * rad)),
                AUTD3([-(1 + np.sqrt(2) / 2) * AUTD3.DEVICE_WIDTH, 0, np.sqrt(2) / 2 * AUTD3.DEVICE_WIDTH]).with_rotation(
                    EulerAngles.ZYZ(0 * rad, np.pi / 4 * rad, 0 * rad)
                ),
                # 1-row
                AUTD3(
                    [-(1 + np.sqrt(2) / 2) * AUTD3.DEVICE_WIDTH, -AUTD3.DEVICE_HEIGHT, np.sqrt(2) / 2 * AUTD3.DEVICE_WIDTH]
                ).with_rotation(EulerAngles.ZYZ(0 * rad, np.pi / 4 * rad, 0 * rad)),
                AUTD3([-AUTD3.DEVICE_WIDTH, -AUTD3.DEVICE_HEIGHT, 0]).with_rotation(
                    EulerAngles.ZYZ(0 * rad, 0 * rad, 0 * rad)
                ),
                AUTD3([0, -AUTD3.DEVICE_HEIGHT, 0]).with_rotation(EulerAngles.ZYZ(0 * rad, 0 * rad, 0 * rad)),
                AUTD3([AUTD3.DEVICE_WIDTH, -AUTD3.DEVICE_HEIGHT, 0]).with_rotation(
                    EulerAngles.ZYZ(0 * rad, -np.pi / 4 * rad, 0 * rad)
                ),
                # 2-row
                AUTD3([AUTD3.DEVICE_WIDTH, -2 * AUTD3.DEVICE_HEIGHT, 0]).with_rotation(
                    EulerAngles.ZYZ(0 * rad, -np.pi / 4 * rad, 0 * rad)
                ),
                AUTD3([0, -2 * AUTD3.DEVICE_HEIGHT, 0]).with_rotation(EulerAngles.ZYZ(0 * rad, 0 * rad, 0 * rad)),
                AUTD3([-AUTD3.DEVICE_WIDTH, -2 * AUTD3.DEVICE_HEIGHT, 0]).with_rotation(
                    EulerAngles.ZYZ(0 * rad, 0 * rad, 0 * rad)
                ),
                AUTD3(
                    [-(1 + np.sqrt(2) / 2) * AUTD3.DEVICE_WIDTH, -2 * AUTD3.DEVICE_HEIGHT, np.sqrt(2) / 2 * AUTD3.DEVICE_WIDTH]
                ).with_rotation(EulerAngles.ZYZ(0 * rad, np.pi / 4 * rad, 0 * rad)),
            ]
        )
        .open(
            Simulator.builder("127.0.0.1:8080"),
        ) as autd
    ):
        runner.run(autd)
