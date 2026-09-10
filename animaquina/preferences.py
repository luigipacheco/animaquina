# Copyright (C) 2026 Luis Arturo Pacheco
# SPDX-License-Identifier: GPL-3.0-or-later
#
# Add-on preferences: robot asset library location.
#
# The robot rigs (robots.blend) ship as a separate download rather than inside
# the add-on zip. They are large, they change on a different cadence than the
# code, and they carry their own redistribution terms — see
# THIRD-PARTY-NOTICES.md. Keeping them out of the extension keeps installs and
# updates small and keeps the code's licensing unambiguous.

import os

import bpy
from bpy.props import StringProperty
from bpy.types import AddonPreferences

ASSET_LIBRARY_NAME = "Animaquina Robots"


def find_registered_library():
    """Return the asset library entry Animaquina registered, or None."""
    libs = getattr(bpy.context.preferences.filepaths, "asset_libraries", None) or ()
    for lib in libs:
        if lib.name == ASSET_LIBRARY_NAME:
            return lib
    return None


def library_dir_from_prefs() -> str:
    prefs = bpy.context.preferences.addons.get("animaquina")
    if prefs is None or getattr(prefs, "preferences", None) is None:
        return ""
    return bpy.path.abspath(str(prefs.preferences.robot_library_path or "").strip())


def library_dir_is_valid(path: str) -> bool:
    if not path or not os.path.isdir(path):
        return False
    return any(f.lower().endswith(".blend") for f in os.listdir(path))


class ANIMAQUINA_Preferences(AddonPreferences):
    bl_idname = "animaquina"

    robot_library_path: StringProperty(
        name="Robot Library Folder",
        description=(
            "Folder containing robots.blend. Download it from the Animaquina "
            "releases page - it is not bundled with the add-on"
        ),
        subtype="DIR_PATH",
        default="",
    )

    def draw(self, _context):
        layout = self.layout

        box = layout.box()
        box.label(text="Robot Asset Library", icon="ASSET_MANAGER")
        box.prop(self, "robot_library_path", text="Folder")

        path = library_dir_from_prefs()
        registered = find_registered_library()

        if not path:
            box.label(text="Download robots.blend and point this at its folder", icon="INFO")
        elif not library_dir_is_valid(path):
            box.label(text="No .blend file found in that folder", icon="ERROR")
        elif registered is not None and bpy.path.abspath(registered.path) == path:
            box.label(text=f"Registered as '{ASSET_LIBRARY_NAME}'", icon="CHECKMARK")
        else:
            box.label(text="Folder looks good - not registered yet", icon="INFO")

        row = box.row(align=True)
        row.operator("animaquina.register_robot_library", icon="ASSET_MANAGER")
        if registered is not None:
            row.operator("animaquina.unregister_robot_library", icon="X")

        box.label(
            text="Registering adds an Asset Library in Preferences > File Paths",
            icon="BLANK1",
        )
