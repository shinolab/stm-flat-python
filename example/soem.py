import os

import numpy as np
from samples import runner  # type: ignore[import,import-not-found]

from pyautd3 import AUTD3, Controller, EulerAngles, deg, rad, tracing_init
from pyautd3.link.soem import SOEM, Status


def err_handler(slave: int, status: Status, msg: str) -> None:
    match status:
        case Status.Error:
            print(f"Error [{slave}]: {msg}")
        case Status.Lost:
            print(f"Lost [{slave}]: {msg}")
            # You can also wait for the link to recover, without exitting the process
            os._exit(-1)
        case Status.StateChanged:
            print(f"StateChanged  [{slave}]: {msg}")


if __name__ == "__main__":
    os.environ["RUST_LOG"] = "autd3=INFO"

    tracing_init()

    with Controller.builder(
        [
            # AUTD3([-AUTD3.DEVICE_WIDTH, AUTD3.DEVICE_HEIGHT, 0]).with_rotation(EulerAngles.ZYZ(-np.pi / 2 * rad, np.pi / 4 * rad, np.pi / 2 * rad)),
            # AUTD3([0, AUTD3.DEVICE_HEIGHT, 0]).with_rotation(EulerAngles.ZYZ(-np.pi / 2 * rad, np.pi / 4 * rad, np.pi / 2 * rad)),
            AUTD3([AUTD3.DEVICE_WIDTH, 0, 0]).with_rotation(EulerAngles.ZYZ(0 * rad, -np.pi / 4 * rad, 0 * rad)),
            AUTD3([0, 0, 0]).with_rotation(EulerAngles.ZYZ(0 * rad, 0 * rad, 0 * rad)),
            AUTD3([-AUTD3.DEVICE_WIDTH, 0, 0]).with_rotation(EulerAngles.ZYZ(0 * rad, 0 * rad, 0 * rad)),
            AUTD3([-(1 + np.sqrt(2) / 2) * AUTD3.DEVICE_WIDTH, 0, np.sqrt(2) / 2 * AUTD3.DEVICE_WIDTH]).with_rotation(
                EulerAngles.ZYZ(0 * rad, np.pi / 4 * rad, 0 * rad)
            ),
            AUTD3(
                [-(1 + np.sqrt(2) / 2) * AUTD3.DEVICE_WIDTH, -AUTD3.DEVICE_HEIGHT, np.sqrt(2) / 2 * AUTD3.DEVICE_WIDTH]
            ).with_rotation(EulerAngles.ZYZ(0 * rad, np.pi / 4 * rad, 0 * rad)),
            AUTD3([-AUTD3.DEVICE_WIDTH, -AUTD3.DEVICE_HEIGHT, 0]).with_rotation(EulerAngles.ZYZ(0 * rad, 0 * rad, 0 * rad)),
            AUTD3([0, -AUTD3.DEVICE_HEIGHT, 0]).with_rotation(EulerAngles.ZYZ(0 * rad, 0 * rad, 0 * rad)),
            AUTD3([AUTD3.DEVICE_WIDTH, -AUTD3.DEVICE_HEIGHT, 0]).with_rotation(
                EulerAngles.ZYZ(0 * rad, -np.pi / 4 * rad, 0 * rad)
            ),
            # AUTD3([0, -(1 + np.sqrt(2) / 2) * AUTD3.DEVICE_HEIGHT, np.sqrt(2) / 2 * AUTD3.DEVICE_HEIGHT]).with_rotation(EulerAngles.ZYZ(-np.pi / 2 * rad, -np.pi / 4 * rad, np.pi / 2 * rad)),
            # AUTD3([-AUTD3.DEVICE_WIDTH, -(1 + np.sqrt(2) / 2) * AUTD3.DEVICE_HEIGHT, np.sqrt(2) / 2 * AUTD3.DEVICE_HEIGHT]).with_rotation(EulerAngles.ZYZ(-np.pi / 2 * rad, -np.pi / 4 * rad, np.pi / 2 * rad)),
            # AUTD3([AUTD3.DEVICE_WIDTH, 0, 0]).with_rotation(EulerAngles.ZYZ(0 * rad, -np.pi / 4 * rad, 0 * rad)),
            # AUTD3([0, 0, 0]).with_rotation(EulerAngles.ZYZ(0 * rad, 0 * rad, 0 * rad)),
            # AUTD3([-AUTD3.DEVICE_WIDTH, 0, 0]).with_rotation(EulerAngles.ZYZ(0 * rad, 0 * rad, 0 * rad)),
            # AUTD3([-2 * AUTD3.DEVICE_WIDTH, 0, AUTD3.DEVICE_WIDTH / np.sqrt(2)]).with_rotation(
            #     EulerAngles.ZYZ(0 * rad, np.pi / 4 * rad, 0 * rad)
            # ),
            # AUTD3([-2 * AUTD3.DEVICE_WIDTH, AUTD3.DEVICE_HEIGHT, AUTD3.DEVICE_WIDTH / np.sqrt(2)]).with_rotation(
            #     EulerAngles.ZYZ(0 * rad, np.pi / 4 * rad, 0 * rad)
            # ),
            # AUTD3([-AUTD3.DEVICE_WIDTH, AUTD3.DEVICE_HEIGHT, 0]).with_rotation(EulerAngles.ZYZ(0 * rad, 0 * rad, 0 * rad)),
            # AUTD3([0, AUTD3.DEVICE_HEIGHT, 0]).with_rotation(EulerAngles.ZYZ(0 * rad, 0 * rad, 0 * rad)),
            # AUTD3([AUTD3.DEVICE_WIDTH, AUTD3.DEVICE_HEIGHT, 0]).with_rotation(
            #     EulerAngles.ZYZ(0 * rad, -np.pi / 4 * rad, 0 * rad)
            # ),
        ]
    ).open(
        SOEM.builder().with_err_handler(err_handler),
    ) as autd:
        runner.run(autd)
