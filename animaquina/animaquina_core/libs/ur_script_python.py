# Copyright (C) 2026 Luis Arturo Pacheco
# SPDX-License-Identifier: GPL-3.0-or-later


class URScriptProgram:
    """Generates valid URScript programs for Universal Robots controllers."""

    def __init__(self, program_name="animaquina"):
        self.program_name = program_name
        # Body accumulated as a line list and joined once in end_program();
        # str += per move is quadratic on large toolpaths.
        self._body_lines = []
        self._program = ""

    def _add_line(self, line):
        """Append an indented line to the program body."""
        self._body_lines.append(f"  {line}\n")

    # -- Setup ----------------------------------------------------------------

    def set_tcp(self, x=0, y=0, z=0, rx=0, ry=0, rz=0):
        """set_tcp(p[x, y, z, rx, ry, rz])"""
        self._add_line(
            f"set_tcp(p[{x:.6f}, {y:.6f}, {z:.6f}, {rx:.6f}, {ry:.6f}, {rz:.6f}])"
        )

    def set_payload(self, mass=0.0, cog=None):
        """set_payload(mass, [cx, cy, cz])"""
        if cog is None:
            cog = [0, 0, 0]
        self._add_line(
            f"set_payload({mass:.4f}, [{cog[0]:.6f}, {cog[1]:.6f}, {cog[2]:.6f}])"
        )

    # -- Motion ---------------------------------------------------------------

    def add_movej(self, joints, a=1.4, v=1.05, r=0.0):
        """
        Joint move.
        joints: sequence of 6 joint angles in radians.
        a: joint acceleration (rad/s²)
        v: joint velocity (rad/s)
        r: blend radius (m)
        """
        j = ", ".join(f"{j:.6f}" for j in joints[:6])
        self._add_line(f"movej([{j}], a={a:.4f}, v={v:.4f}, r={r:.6f})")

    def add_movel(self, x, y, z, rx, ry, rz, a=0.5, v=0.1, r=0.001):
        """
        Linear move in tool space.
        x, y, z: position in metres.
        rx, ry, rz: rotation vector (axis-angle) in radians.
        a: tool acceleration (m/s²)
        v: tool velocity (m/s)
        r: blend radius (m)
        """
        self._add_line(
            f"movel(p[{x:.6f}, {y:.6f}, {z:.6f}, {rx:.6f}, {ry:.6f}, {rz:.6f}], "
            f"a={a:.4f}, v={v:.4f}, r={r:.6f})"
        )

    # -- I/O ------------------------------------------------------------------

    def set_digital_output(self, port, value):
        """set_digital_out(port, True/False)"""
        val = "True" if value else "False"
        self._add_line(f"set_digital_out({port}, {val})")

    def set_analog_output(self, port, value):
        """set_standard_analog_out(port, value)"""
        self._add_line(f"set_standard_analog_out({port}, {value:.4f})")

    def wait_digital_input(self, port, value):
        """Block until digital input matches expected value."""
        val = "True" if value else "False"
        self._add_line(f"while (get_digital_in({port}) != {val}):")
        self._body_lines.append("    sleep(0.01)\n")
        self._body_lines.append("  end\n")

    def wait_time(self, seconds):
        """sleep(seconds)"""
        self._add_line(f"sleep({seconds:.4f})")

    # -- Misc -----------------------------------------------------------------

    def add_comment(self, msg):
        """Inline comment."""
        self._add_line(f"# {msg}")

    def add_textmsg(self, msg):
        """Display message on teach pendant log."""
        self._add_line(f'textmsg("{msg}")')

    def add_popup(self, msg, title="Animaquina", blocking=True):
        """Show a popup dialog on the teach pendant.
        If blocking=True, program waits until operator presses OK."""
        if blocking:
            self._add_line(f'popup("{msg}", "{title}", blocking=True)')
        else:
            self._add_line(f'popup("{msg}", "{title}", blocking=False)')

    def set_variable(self, name, value):
        """Assign a variable (auto-formats bool/int/float)."""
        if isinstance(value, bool):
            val_str = "True" if value else "False"
        elif isinstance(value, (int, float)):
            val_str = str(value)
        else:
            val_str = f'"{value}"'
        self._add_line(f"{name} = {val_str}")

    # -- Output ---------------------------------------------------------------

    def end_program(self):
        """Finalise the program string."""
        self._program = (
            f"def {self.program_name}():\n"
            f"{''.join(self._body_lines)}"
            "end\n"
            f"{self.program_name}()\n"
        )

    def get_program(self) -> str:
        """Return the full program text (call end_program() first)."""
        return self._program

    def save_program(self, filename="generated_program.script"):
        """Write the program to a file."""
        with open(filename, "w") as f:
            f.write(self._program)
