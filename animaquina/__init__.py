# Animaquina - multi-robot control and toolpathing for Blender
# Copyright (C) 2026 Luis Arturo Pacheco
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.
#
# SPDX-License-Identifier: GPL-3.0-or-later
#
# SAFETY: this software commands industrial robots capable of causing serious
# injury or death. It provides NO safety function and is NOT a substitute for
# the robot's own safety-rated systems, guarding, or a risk assessment.
# See SAFETY.md before connecting to any machine.
# Animaquina v0.1 — Multi-robot control and toolpathing for Blender
# Section 25: no preview icons, single PropertyGroup, register/unregister only addon classes

import os
import sys

bl_info = {
    "name": "Animaquina",
    "author": "Luis Arturo Pacheco",
    "description": "Connect to industrial/collaborative robots, sync with a digital twin, send paths, export programs.",
    "blender": (5, 2, 0),
    "version": (0, 1, 0),
    "location": "View3D > Sidebar > Animaquina",
    "warning": "Beta — use at your own risk.",
    "doc_url": "https://www.animaquina.com",
    "tracker_url": "",
    "category": "3D View",
}

_core_import_error = None


def _bootstrap_import_paths():
    """Make both side-by-side and staged local-dev layouts importable.

    Centralised path setup — drivers should NOT manipulate sys.path themselves.

    Blender development tools can load the addon through a junction/symlink path,
    so we register both the visible addon path and its resolved real path.
    """
    package_dir = os.path.dirname(__file__)
    real_package_dir = os.path.realpath(package_dir)
    parent_dir = os.path.dirname(package_dir)
    real_parent_dir = os.path.dirname(real_package_dir)
    search_roots = (
        # addon package itself
        package_dir,
        real_package_dir,
        # parent directory (side-by-side animaquina_core)
        parent_dir,
        real_parent_dir,
        # bundled third-party libs (urx, math3d, xarm)
        os.path.join(package_dir, "libs"),
        os.path.join(real_package_dir, "libs"),
        # runtime-installed packages (ur_rtde, paramiko)
        os.path.join(package_dir, "vendor_py"),
        os.path.join(real_package_dir, "vendor_py"),
        # animaquina_core internal libs (kukaproxydriver, krl, urscript)
        os.path.join(package_dir, "animaquina_core", "libs"),
        os.path.join(real_package_dir, "animaquina_core", "libs"),
    )
    for root in search_roots:
        if root and os.path.isdir(root) and root not in sys.path:
            sys.path.insert(0, root)

try:
    import bpy
    from bpy.app.handlers import persistent
except ModuleNotFoundError:
    bpy = None

    def persistent(fn):
        return fn


_bootstrap_import_paths()

if bpy is not None:
    try:
        from .preferences import ANIMAQUINA_Preferences
        from . import properties
        from . import manager
        from . import run_state
        from .ui import PANEL_CLASSES, OPERATOR_CLASSES
    except ModuleNotFoundError as exc:
        missing_name = getattr(exc, "name", "") or ""
        if missing_name == "animaquina_core" or missing_name.startswith("animaquina_core."):
            _core_import_error = exc
            ANIMAQUINA_Preferences = None
            properties = None
            manager = None
            run_state = None
            PANEL_CLASSES = ()
            OPERATOR_CLASSES = ()
        else:
            raise
else:
    ANIMAQUINA_Preferences = None
    properties = None
    manager = None
    run_state = None
    PANEL_CLASSES = ()
    OPERATOR_CLASSES = ()


@persistent
def _animaquina_selection_sync_handler(_scene, _depsgraph):
    properties.sync_active_robot_from_selected_target()


def register():
    if bpy is None:
        raise RuntimeError("animaquina.register() requires Blender (bpy)")
    if _core_import_error is not None:
        raise RuntimeError(
            "animaquina_core could not be imported. The core package ships inside the "
            "add-on at animaquina/animaquina_core - reinstall the add-on if it is missing."
        ) from _core_import_error
    if ANIMAQUINA_Preferences is not None:
        bpy.utils.register_class(ANIMAQUINA_Preferences)
    bpy.utils.register_class(properties.ANIMAQUINA_DebugVar)
    bpy.utils.register_class(properties.ANIMAQUINA_RobotSlot)
    bpy.utils.register_class(properties.ANIMAQUINA_SceneProperties)
    properties.register_properties()
    if _animaquina_selection_sync_handler not in bpy.app.handlers.depsgraph_update_post:
        bpy.app.handlers.depsgraph_update_post.append(_animaquina_selection_sync_handler)
    run_state.register()
    for c in OPERATOR_CLASSES:
        bpy.utils.register_class(c)
    for c in PANEL_CLASSES:
        bpy.utils.register_class(c)


def unregister():
    if bpy is None:
        return
    # Disconnect all slots while scene properties are still available
    if bpy.context.scene and hasattr(bpy.context.scene, "animaquina"):
        for slot in list(bpy.context.scene.animaquina.robots):
            if slot.is_connected:
                manager.disconnect_slot(slot)
    if _animaquina_selection_sync_handler in bpy.app.handlers.depsgraph_update_post:
        bpy.app.handlers.depsgraph_update_post.remove(_animaquina_selection_sync_handler)
    if run_state is not None:
        run_state.unregister()
    for c in reversed(PANEL_CLASSES):
        bpy.utils.unregister_class(c)
    for c in reversed(OPERATOR_CLASSES):
        bpy.utils.unregister_class(c)
    properties.unregister_properties()
    bpy.utils.unregister_class(properties.ANIMAQUINA_SceneProperties)
    bpy.utils.unregister_class(properties.ANIMAQUINA_RobotSlot)
    bpy.utils.unregister_class(properties.ANIMAQUINA_DebugVar)
    if ANIMAQUINA_Preferences is not None:
        bpy.utils.unregister_class(ANIMAQUINA_Preferences)


if __name__ == "__main__":
    register()
