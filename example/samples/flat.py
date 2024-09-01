import numpy as np

from pyautd3 import Controller, ControlPoint, ControlPoints4, FociSTM, Focus, GainSTM, Hz, Silencer, Static


def stm_single(autd: Controller) -> None:
    config = Silencer.disable()
    autd.send(config)

    m = Static()

    edge_length = 40
    freq = 200.0
    width = 5

    center = autd.geometry.center + np.array([0.0, 0.0, 150.0])
    stm_iter_list = stm_iter(edge_length, freq, width)

    stm = FociSTM(
        freq * Hz,
        (center + np.array([stm_iter_list[i][0], stm_iter_list[i][1], 0]) for i in range(len(stm_iter_list))),
    )

    autd.send((m, stm))


def stm_multi(autd: Controller) -> None:
    config = Silencer.disable()
    autd.send(config)

    m = Static()

    edge_length = 40
    freq = 100.0
    width = 5

    center = autd.geometry.center + np.array([0.0, 0.0, 150.0])
    stm_iter_list = stm_iter(edge_length / 2, freq, width)

    stm = FociSTM(
        freq * Hz,
        (
            ControlPoints4(
                (
                    ControlPoint(center + np.array([stm_iter_list[i][0], stm_iter_list[i][1], 0])),
                    ControlPoint(center + np.array([edge_length / 2 + stm_iter_list[i][0], stm_iter_list[i][1], 0])),
                    ControlPoint(center + np.array([stm_iter_list[i][0], edge_length / 2 + stm_iter_list[i][1], 0])),
                    ControlPoint(
                        center + np.array([edge_length / 2 + stm_iter_list[i][0], edge_length / 2 + stm_iter_list[i][1], 0])
                    ),
                )
            )
            for i in range(len(stm_iter_list))
        ),
    )

    autd.send((m, stm))


def stm_iter(edge_length, freq, width):
    iter_list = []
    size = 40000 / freq
    interval = edge_length * (edge_length / width + 2) / (size - 1)
    edge_width_size = (edge_length + width) / interval

    for i in range(int(size)):
        r = int(i / edge_width_size)
        l = int(i % edge_width_size)
        if r % 2 == 0:
            if l < edge_length / interval:
                x = l * interval
                y = r * width
            else:
                x = edge_length
                y = width * r + (l - edge_length / interval) * interval
        else:
            if l < edge_length / interval:
                x = edge_length - l * interval
                y = r * width
            else:
                x = 0
                y = width * r + (l - edge_length / interval) * interval
        point = np.array([x, y, 0])
        iter_list.append(point)
    return iter_list
