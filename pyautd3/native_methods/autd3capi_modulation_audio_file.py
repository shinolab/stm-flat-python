# This file is autogenerated
import threading
import ctypes
import os
from pyautd3.native_methods.structs import Vector3, Quaternion, FfiFuture, LocalFfiFuture
from pyautd3.native_methods.autd3capi_driver import LoopBehavior, ModulationPtr, ResultModulation



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
            self.dll = ctypes.CDLL(os.path.join(bin_location, f'{bin_prefix}autd3capi_modulation_audio_file{bin_ext}'))
        except Exception:   # pragma: no cover
            return          # pragma: no cover

        self.dll.AUTDModulationWav.argtypes = [ctypes.c_char_p, LoopBehavior]  # type: ignore 
        self.dll.AUTDModulationWav.restype = ResultModulation

        self.dll.AUTDModulationWavIsDefault.argtypes = [ModulationPtr]  # type: ignore 
        self.dll.AUTDModulationWavIsDefault.restype = ctypes.c_bool

        self.dll.AUTDModulationRawPCM.argtypes = [ctypes.c_char_p, ctypes.c_uint32, LoopBehavior]  # type: ignore 
        self.dll.AUTDModulationRawPCM.restype = ResultModulation

        self.dll.AUTDModulationCsv.argtypes = [ctypes.c_char_p, ctypes.c_uint32, ctypes.c_uint8, LoopBehavior]  # type: ignore 
        self.dll.AUTDModulationCsv.restype = ResultModulation

    def modulation_wav(self, path: bytes, loop_behavior: LoopBehavior) -> ResultModulation:
        return self.dll.AUTDModulationWav(path, loop_behavior)

    def modulation_wav_is_default(self, wav: ModulationPtr) -> ctypes.c_bool:
        return self.dll.AUTDModulationWavIsDefault(wav)

    def modulation_raw_pcm(self, path: bytes, sample_rate: int, loop_behavior: LoopBehavior) -> ResultModulation:
        return self.dll.AUTDModulationRawPCM(path, sample_rate, loop_behavior)

    def modulation_csv(self, path: bytes, sample_rate: int, deliminator: int, loop_behavior: LoopBehavior) -> ResultModulation:
        return self.dll.AUTDModulationCsv(path, sample_rate, deliminator, loop_behavior)
