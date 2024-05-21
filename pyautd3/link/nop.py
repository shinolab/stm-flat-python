from pyautd3.driver.link import Link, LinkBuilder
from pyautd3.native_methods.autd3capi import ControllerPtr
from pyautd3.native_methods.autd3capi import (
    NativeMethods as LinkNop,
)
from pyautd3.native_methods.autd3capi_driver import LinkBuilderPtr, LinkPtr


class Nop(Link):
    class _Builder(LinkBuilder["Nop"]):
        def __init__(self: "Nop._Builder") -> None:
            pass

        def _link_builder_ptr(self: "Nop._Builder") -> LinkBuilderPtr:
            return LinkNop().link_nop()

        def _resolve_link(self: "Nop._Builder", _ptr: ControllerPtr) -> "Nop":
            return Nop(LinkNop().link_get(_ptr))

    def __init__(self: "Nop", ptr: LinkPtr) -> None:
        super().__init__(ptr)

    @staticmethod
    def builder() -> _Builder:
        return Nop._Builder()
