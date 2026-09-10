# Copyright (C) 2026 Luis Arturo Pacheco
# SPDX-License-Identifier: GPL-3.0-or-later
# Animaquina - Blender runtime (Section 10)
# Modules that depend on bpy / mathutils stay here.
# Pure-Python modules (conversions, krl_stream, etc.) live in animaquina_core.runtime.
#
# Robot-specific modules are lazy-loaded so the addon starts even when
# animaquina_core doesn't include files for every robot brand.

from . import rig_apply
from . import simulation
from . import markers
from . import recorder
from . import mujoco_export
from . import newton_validator

# Re-export core shared modules (always present)
from animaquina_core.runtime import conversions

# Robot-specific modules are lazy — imported on first attribute access.
# This allows the addon to load even when animaquina_core doesn't include
# driver/runtime files for every robot brand.
_LAZY_MODULES = {
    # GPL-side robot-specific wrappers
    "krl_export",
    "ur_export",
    # Core robot-specific re-exports
    "krl_stream",
    "kuka_krl_parser",
    "kuka_stream_runtime",
}


def __getattr__(name: str):
    if name in _LAZY_MODULES:
        try:
            import importlib
            mod = importlib.import_module(f".{name}", __name__)
        except ImportError:
            # Try core runtime as fallback for re-exported modules
            try:
                mod = importlib.import_module(f"animaquina_core.runtime.{name}")
            except ImportError:
                raise AttributeError(
                    f"animaquina.runtime.{name} is not available "
                    f"(robot-specific module not installed)"
                )
        globals()[name] = mod
        return mod
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")


__all__ = [
    "rig_apply",
    "simulation",
    "markers",
    "recorder",
    "newton_validator",
    "conversions",
    "krl_export",
    "ur_export",
    "mujoco_export",
    "krl_stream",
    "kuka_krl_parser",
    "kuka_stream_runtime",
]
