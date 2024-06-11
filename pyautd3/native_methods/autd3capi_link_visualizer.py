# This file is autogenerated
import threading
import ctypes
import os
from pyautd3.native_methods.structs import Vector3, Quaternion
from pyautd3.native_methods.autd3capi_driver import GeometryPtr, LinkBuilderPtr, LinkPtr, ResultI32, Segment

from enum import IntEnum


class Backend(IntEnum):
    Plotters = 0
    Python = 1
    Null = 2

    @classmethod
    def from_param(cls, obj):
        return int(obj)  # pragma: no cover


class Directivity(IntEnum):
    Sphere = 0
    T4010A1 = 1

    @classmethod
    def from_param(cls, obj):
        return int(obj)  # pragma: no cover


class CMap(IntEnum):
    Jet = 0
    Viridis = 1
    Magma = 2
    Inferno = 3
    Plasma = 4
    Cividis = 5
    Turbo = 6
    Circle = 7
    Bluered = 8
    Breeze = 9
    Mist = 10
    Earth = 11
    Hell = 12

    @classmethod
    def from_param(cls, obj):
        return int(obj)  # pragma: no cover


class ConfigPtr(ctypes.Structure):
    _fields_ = [("_0", ctypes.c_void_p)]


class PlotRangePtr(ctypes.Structure):
    _fields_ = [("_0", ctypes.c_void_p)]


class NullPlotConfigPtr(ctypes.Structure):
    _fields_ = [("_0", ctypes.c_void_p)]


class PlotConfigPtr(ctypes.Structure):
    _fields_ = [("_0", ctypes.c_void_p)]


class PyPlotConfigPtr(ctypes.Structure):
    _fields_ = [("_0", ctypes.c_void_p)]


class ResultPlotConfig(ctypes.Structure):
    _fields_ = [("result", PlotConfigPtr), ("err_len", ctypes.c_uint32), ("err", ctypes.c_void_p)]


    def __eq__(self, other: object) -> bool:
        return isinstance(other, ResultPlotConfig) and self._fields_ == other._fields_ # pragma: no cover
                    

class ResultPyPlotConfig(ctypes.Structure):
    _fields_ = [("result", PyPlotConfigPtr), ("err_len", ctypes.c_uint32), ("err", ctypes.c_void_p)]


    def __eq__(self, other: object) -> bool:
        return isinstance(other, ResultPyPlotConfig) and self._fields_ == other._fields_ # pragma: no cover
                    


class Singleton(type):
    _instances = {}  # type: ignore
    _lock = threading.Lock()

    def __call__(cls, *args, **kwargs):
        if cls not in cls._instances:
            with cls._lock:
                if cls not in cls._instances: # pragma: no cover
                    cls._instances[cls] = super(Singleton, cls).__call__(*args, **kwargs)
        return cls._instances[cls]


class NativeMethods(metaclass=Singleton):

    def init_dll(self, bin_location: str, bin_prefix: str, bin_ext: str):
        try:
            self.dll = ctypes.CDLL(os.path.join(bin_location, f'{bin_prefix}autd3capi_link_visualizer{bin_ext}'))
        except Exception:   # pragma: no cover
            return          # pragma: no cover

        self.dll.AUTDLinkVisualizerPlotRange.argtypes = [ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_float] 
        self.dll.AUTDLinkVisualizerPlotRange.restype = PlotRangePtr

        self.dll.AUTDLinkVisualizerPlotRangeObservePointsLen.argtypes = [PlotRangePtr]  # type: ignore 
        self.dll.AUTDLinkVisualizerPlotRangeObservePointsLen.restype = ctypes.c_uint64

        self.dll.AUTDLinkVisualizerPlotRangeObservePoints.argtypes = [PlotRangePtr, ctypes.POINTER(ctypes.c_float)]  # type: ignore 
        self.dll.AUTDLinkVisualizerPlotRangeObservePoints.restype = None

        self.dll.AUTDLinkVisualizerPhasesOf.argtypes = [LinkPtr, Backend, Directivity, Segment, ctypes.c_uint16, ctypes.POINTER(ctypes.c_uint8)]  # type: ignore 
        self.dll.AUTDLinkVisualizerPhasesOf.restype = ctypes.c_uint32

        self.dll.AUTDLinkVisualizerIntensities.argtypes = [LinkPtr, Backend, Directivity, Segment, ctypes.c_uint16, ctypes.POINTER(ctypes.c_uint8)]  # type: ignore 
        self.dll.AUTDLinkVisualizerIntensities.restype = ctypes.c_uint32

        self.dll.AUTDLinkVisualizerModulation.argtypes = [LinkPtr, Backend, Directivity, Segment, ctypes.POINTER(ctypes.c_uint8)]  # type: ignore 
        self.dll.AUTDLinkVisualizerModulation.restype = ctypes.c_uint32

        self.dll.AUTDLinkVisualizerCalcField.argtypes = [LinkPtr, Backend, Directivity, ctypes.POINTER(Vector3), ctypes.c_uint32, GeometryPtr, Segment, ctypes.c_uint16, ctypes.POINTER(ctypes.c_float)]  # type: ignore 
        self.dll.AUTDLinkVisualizerCalcField.restype = ResultI32

        self.dll.AUTDLinkVisualizerPlotField.argtypes = [LinkPtr, Backend, Directivity, ConfigPtr, PlotRangePtr, GeometryPtr, Segment, ctypes.c_uint16]  # type: ignore 
        self.dll.AUTDLinkVisualizerPlotField.restype = ResultI32

        self.dll.AUTDLinkVisualizerPlotPhase.argtypes = [LinkPtr, Backend, Directivity, ConfigPtr, GeometryPtr, Segment, ctypes.c_uint16]  # type: ignore 
        self.dll.AUTDLinkVisualizerPlotPhase.restype = ResultI32

        self.dll.AUTDLinkVisualizerPlotModulation.argtypes = [LinkPtr, Backend, Directivity, ConfigPtr, Segment]  # type: ignore 
        self.dll.AUTDLinkVisualizerPlotModulation.restype = ResultI32

        self.dll.AUTDLinkVisualizerSphereNull.argtypes = [ctypes.c_bool, ctypes.c_int32] 
        self.dll.AUTDLinkVisualizerSphereNull.restype = LinkBuilderPtr

        self.dll.AUTDLinkVisualizerT4010A1Null.argtypes = [ctypes.c_bool, ctypes.c_int32] 
        self.dll.AUTDLinkVisualizerT4010A1Null.restype = LinkBuilderPtr

        self.dll.AUTDLinkVisualizerNullPlotConfig.argtypes = [] 
        self.dll.AUTDLinkVisualizerNullPlotConfig.restype = NullPlotConfigPtr

        self.dll.AUTDLinkVisualizerSpherePlotters.argtypes = [ctypes.c_bool, ctypes.c_int32] 
        self.dll.AUTDLinkVisualizerSpherePlotters.restype = LinkBuilderPtr

        self.dll.AUTDLinkVisualizerT4010A1Plotters.argtypes = [ctypes.c_bool, ctypes.c_int32] 
        self.dll.AUTDLinkVisualizerT4010A1Plotters.restype = LinkBuilderPtr

        self.dll.AUTDLinkVisualizerPlotConfig.argtypes = [ctypes.c_uint32, ctypes.c_uint32, ctypes.c_float, ctypes.c_uint32, ctypes.c_uint32, ctypes.c_uint32, ctypes.c_float, CMap, ctypes.c_char_p]  # type: ignore 
        self.dll.AUTDLinkVisualizerPlotConfig.restype = ResultPlotConfig

        self.dll.AUTDLinkVisualizerPlotConfigIsDefault.argtypes = [PlotConfigPtr]  # type: ignore 
        self.dll.AUTDLinkVisualizerPlotConfigIsDefault.restype = ctypes.c_bool

        self.dll.AUTDLinkVisualizerSpherePython.argtypes = [ctypes.c_bool, ctypes.c_int32] 
        self.dll.AUTDLinkVisualizerSpherePython.restype = LinkBuilderPtr

        self.dll.AUTDLinkVisualizerT4010A1Python.argtypes = [ctypes.c_bool, ctypes.c_int32] 
        self.dll.AUTDLinkVisualizerT4010A1Python.restype = LinkBuilderPtr

        self.dll.AUTDLinkVisualizerPyPlotConfig.argtypes = [ctypes.c_int32, ctypes.c_int32, ctypes.c_int32, ctypes.c_char_p, ctypes.c_char_p, ctypes.c_char_p, ctypes.c_int32, ctypes.c_float, ctypes.c_char_p, ctypes.c_bool, ctypes.c_char_p] 
        self.dll.AUTDLinkVisualizerPyPlotConfig.restype = ResultPyPlotConfig

        self.dll.AUTDLinkVisualizerPyPlotConfigIsDefault.argtypes = [PyPlotConfigPtr]  # type: ignore 
        self.dll.AUTDLinkVisualizerPyPlotConfigIsDefault.restype = ctypes.c_bool

    def link_visualizer_plot_range(self, x_min: float, x_max: float, y_min: float, y_max: float, z_min: float, z_max: float, resolution: float) -> PlotRangePtr:
        return self.dll.AUTDLinkVisualizerPlotRange(x_min, x_max, y_min, y_max, z_min, z_max, resolution)

    def link_visualizer_plot_range_observe_points_len(self, range: PlotRangePtr) -> ctypes.c_uint64:
        return self.dll.AUTDLinkVisualizerPlotRangeObservePointsLen(range)

    def link_visualizer_plot_range_observe_points(self, range: PlotRangePtr, points: ctypes.Array[ctypes.c_float] | None) -> None:
        return self.dll.AUTDLinkVisualizerPlotRangeObservePoints(range, points)

    def link_visualizer_phases_of(self, visualizer: LinkPtr, backend: Backend, directivity: Directivity, segment: Segment, idx: int, buf: ctypes.Array[ctypes.c_uint8] | None) -> ctypes.c_uint32:
        return self.dll.AUTDLinkVisualizerPhasesOf(visualizer, backend, directivity, segment, idx, buf)

    def link_visualizer_intensities(self, visualizer: LinkPtr, backend: Backend, directivity: Directivity, segment: Segment, idx: int, buf: ctypes.Array[ctypes.c_uint8] | None) -> ctypes.c_uint32:
        return self.dll.AUTDLinkVisualizerIntensities(visualizer, backend, directivity, segment, idx, buf)

    def link_visualizer_modulation(self, visualizer: LinkPtr, backend: Backend, directivity: Directivity, segment: Segment, buf: ctypes.Array[ctypes.c_uint8] | None) -> ctypes.c_uint32:
        return self.dll.AUTDLinkVisualizerModulation(visualizer, backend, directivity, segment, buf)

    def link_visualizer_calc_field(self, visualizer: LinkPtr, backend: Backend, directivity: Directivity, points: ctypes.Array | None, points_len: int, geometry: GeometryPtr, segment: Segment, idx: int, buf: ctypes.Array[ctypes.c_float] | None) -> ResultI32:
        return self.dll.AUTDLinkVisualizerCalcField(visualizer, backend, directivity, points, points_len, geometry, segment, idx, buf)

    def link_visualizer_plot_field(self, visualizer: LinkPtr, backend: Backend, directivity: Directivity, config: ConfigPtr, range: PlotRangePtr, geometry: GeometryPtr, segment: Segment, idx: int) -> ResultI32:
        return self.dll.AUTDLinkVisualizerPlotField(visualizer, backend, directivity, config, range, geometry, segment, idx)

    def link_visualizer_plot_phase(self, visualizer: LinkPtr, backend: Backend, directivity: Directivity, config: ConfigPtr, geometry: GeometryPtr, segment: Segment, idx: int) -> ResultI32:
        return self.dll.AUTDLinkVisualizerPlotPhase(visualizer, backend, directivity, config, geometry, segment, idx)

    def link_visualizer_plot_modulation(self, visualizer: LinkPtr, backend: Backend, directivity: Directivity, config: ConfigPtr, segment: Segment) -> ResultI32:
        return self.dll.AUTDLinkVisualizerPlotModulation(visualizer, backend, directivity, config, segment)

    def link_visualizer_sphere_null(self, use_gpu: bool, gpu_idx: int) -> LinkBuilderPtr:
        return self.dll.AUTDLinkVisualizerSphereNull(use_gpu, gpu_idx)

    def link_visualizer_t_4010_a_1_null(self, use_gpu: bool, gpu_idx: int) -> LinkBuilderPtr:
        return self.dll.AUTDLinkVisualizerT4010A1Null(use_gpu, gpu_idx)

    def link_visualizer_null_plot_config(self) -> NullPlotConfigPtr:
        return self.dll.AUTDLinkVisualizerNullPlotConfig()

    def link_visualizer_sphere_plotters(self, use_gpu: bool, gpu_idx: int) -> LinkBuilderPtr:
        return self.dll.AUTDLinkVisualizerSpherePlotters(use_gpu, gpu_idx)

    def link_visualizer_t_4010_a_1_plotters(self, use_gpu: bool, gpu_idx: int) -> LinkBuilderPtr:
        return self.dll.AUTDLinkVisualizerT4010A1Plotters(use_gpu, gpu_idx)

    def link_visualizer_plot_config(self, width: int, height: int, cbar_size: float, font_size: int, label_area_size: int, margin: int, ticks_step: float, cmap: CMap, fname: bytes) -> ResultPlotConfig:
        return self.dll.AUTDLinkVisualizerPlotConfig(width, height, cbar_size, font_size, label_area_size, margin, ticks_step, cmap, fname)

    def link_visualizer_plot_config_is_default(self, config: PlotConfigPtr) -> ctypes.c_bool:
        return self.dll.AUTDLinkVisualizerPlotConfigIsDefault(config)

    def link_visualizer_sphere_python(self, use_gpu: bool, gpu_idx: int) -> LinkBuilderPtr:
        return self.dll.AUTDLinkVisualizerSpherePython(use_gpu, gpu_idx)

    def link_visualizer_t_4010_a_1_python(self, use_gpu: bool, gpu_idx: int) -> LinkBuilderPtr:
        return self.dll.AUTDLinkVisualizerT4010A1Python(use_gpu, gpu_idx)

    def link_visualizer_py_plot_config(self, width: int, height: int, dpi: int, cbar_position: bytes, cbar_size: bytes, cbar_pad: bytes, fontsize: int, ticks_step: float, cmap: bytes, show: bool, fname: bytes) -> ResultPyPlotConfig:
        return self.dll.AUTDLinkVisualizerPyPlotConfig(width, height, dpi, cbar_position, cbar_size, cbar_pad, fontsize, ticks_step, cmap, show, fname)

    def link_visualizer_py_plot_config_is_default(self, config: PyPlotConfigPtr) -> ctypes.c_bool:
        return self.dll.AUTDLinkVisualizerPyPlotConfigIsDefault(config)
