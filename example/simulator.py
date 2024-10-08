from pyautd3 import AUTD3, Controller
from pyautd3.link.simulator import Simulator
from samples import runner  # type: ignore[import,import-not-found]

if __name__ == "__main__":
    with (
        Controller[Simulator]
        .builder([AUTD3([0.0, 0.0, 0.0]), AUTD3([AUTD3.DEVICE_WIDTH, 0.0, 0.0])])
        .open(
            Simulator.builder("127.0.0.1:8080"),
        ) as autd
    ):
        runner.run(autd)
