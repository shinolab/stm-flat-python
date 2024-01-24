"""
File: static.py
Project: modulation
Created Date: 14/09/2023
Author: Shun Suzuki
-----
Last Modified: 24/01/2024
Modified By: Shun Suzuki (suzuki@hapis.k.u-tokyo.ac.jp)
-----
Copyright (c) 2023 Shun Suzuki. All rights reserved.

"""

from pyautd3.emit_intensity import EmitIntensity
from pyautd3.internal.modulation import IModulation
from pyautd3.native_methods.autd3capi import NativeMethods as Base
from pyautd3.native_methods.autd3capi_def import ModulationPtr


class Static(IModulation):
    """Without modulation."""

    _intensity: EmitIntensity

    def __init__(self: "Static", intensity: int | EmitIntensity | None = None) -> None:
        super().__init__()
        self._intensity = EmitIntensity.maximum() if intensity is None else EmitIntensity._cast(intensity)

    @staticmethod
    def with_intensity(intensity: int | EmitIntensity) -> "Static":
        """Static with intensity.

        Arguments:
        ---------
            intensity: Emission intensity
        """
        return Static(intensity)

    def intensity(self: "Static") -> EmitIntensity:
        """Get emission intensity."""
        return self._intensity

    def _modulation_ptr(self: "Static") -> ModulationPtr:
        return Base().modulation_static(self._intensity.value)
