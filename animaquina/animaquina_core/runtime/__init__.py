# Copyright (C) 2026 Luis Arturo Pacheco
# SPDX-License-Identifier: GPL-3.0-or-later

"""Animaquina Core runtime — pure-Python utilities (no Blender dependency).

Imports are lazy: robot-specific modules (krl_stream, ur_stream, etc.) are only
loaded on first access, so importing the package never pulls in a brand's
dependencies unless that brand is actually used.
"""

# Shared modules — always present
from . import conversions

# Everything else is lazy-loaded on attribute access.
_LAZY_MODULES = {
    "krl_stream",
    "kuka_krl_parser",
    "kuka_stream_runtime",
    "ur_stream",
    "mujoco_export",
    "program_exports",
    "newton_worker",
}


def __getattr__(name: str):
    if name in _LAZY_MODULES:
        try:
            import importlib
            mod = importlib.import_module(f".{name}", __name__)
            globals()[name] = mod
            return mod
        except ImportError:
            raise AttributeError(
                f"animaquina_core.runtime.{name} is not available"
            )
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")


__all__ = [
    "conversions",
    "krl_stream",
    "kuka_krl_parser",
    "kuka_stream_runtime",
    "ur_stream",
    "mujoco_export",
    "program_exports",
    "newton_worker",
]
