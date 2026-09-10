# Copyright (C) 2026 Luis Arturo Pacheco
# SPDX-License-Identifier: GPL-3.0-or-later
class KRL:
    def __init__(self, program_name="kuka_program"):
        self.program = ""
        self.program_name = program_name
        # Sections are accumulated as line lists and joined once in
        # end_program(); str += per move is quadratic on large toolpaths.
        self._declaration_lines = []
        self._motion_lines = []
        self.start_program()

    @property
    def declaration_section(self):
        return "".join(self._declaration_lines)

    @property
    def motion_section(self):
        return "".join(self._motion_lines)

    def start_program(self):
        """Initializes the KRL program with default settings"""
        self._declaration_lines.append(
            "&ACCESS RVP\n"
            "&REL 1\n"
            "&PARAM TEMPLATE = C:\\KRC\\Roboter\\Template\\vorgabe\n"
            "&PARAM EDITMASK = *\n"
            f"DEF {self.program_name}()\n\n"
            ";FOLD INI\n"
            ";FOLD BASISTECH INI\n"
            "GLOBAL INTERRUPT DECL 3 WHEN $STOPMESS==TRUE DO IR_STOPM()\n"
            "INTERRUPT ON 3\n"
            "BAS(#INITMOV, 0)\n"
            ";ENDFOLD (BASISTECH INI)\n"
            ";ENDFOLD (INI)\n\n"
        )

    def set_tool(self, tool_no=1, mass=0.5, cog=[0, 0, 0]):
        """Defines a tool in the declaration section"""
        self._declaration_lines.append(
            f"DECL TOOLDATA TOOL_{tool_no} = {{M {mass}, CM {{X {cog[0]}, Y {cog[1]}, Z {cog[2]}}}}}\n"
        )

    def set_base(self, base_no=1, pos=[0, 0, 0], orient=[0, 0, 0]):
        """Defines a base in the declaration section"""
        self._declaration_lines.append(
            f"DECL FRAME BASE_{base_no} = {{X {pos[0]}, Y {pos[1]}, Z {pos[2]}, A {orient[0]}, B {orient[1]}, C {orient[2]}}}\n"
        )

    def set_speed(self, speed=15, acceleration=100):
        """Sets motion speed in the declaration section"""
        self._declaration_lines.append(f"DECL REAL VEL = {speed}\n")
        self._declaration_lines.append(f"DECL REAL ACC = {acceleration}\n")

    def set_start_parameters(self, base=0, tool=0, speed=15, acc=100, lspeed = 0.05, advance = 3, pos=None):
        """Defines the start position in the declaration section"""
        if pos is None:
            pos = {"A1": 5, "A2": -90, "A3": 100, "A4": 5, "A5": -10, "A6": -5, "E1": 0, "E2": 0, "E3": 0, "E4": 0}

        self._declaration_lines.append(
            f";FOLD STARTPOSITION - BASE IS {base}, TOOL IS {tool}, SPEED IS {speed}%, POSITION IS "
            f"A1 {pos['A1']}, A2 {pos['A2']}, A3 {pos['A3']}, A4 {pos['A4']}, A5 {pos['A5']}, A6 {pos['A6']}, "
            f"E1 {pos['E1']}, E2 {pos['E2']}, E3 {pos['E3']}, E4 {pos['E4']}\n"
            "$BWDSTART = FALSE\n"
            f"PDAT_ACT = {{VEL {speed}, ACC {acc}, APO_DIST 50}}\n"
            f"FDAT_ACT = {{TOOL_NO {tool}, BASE_NO {base}, IPO_FRAME #BASE}}\n"
            f"BAS(#PTP_PARAMS, {speed})\n"
            ";ENDFOLD\n\n"
            ";FOLD LIN SPEED IS 0.05 m/sec, INTERPOLATION SETTINGS IN FOLD\n"
            f"$VEL.CP={lspeed:.4f}\n"
            f"$ADVANCE={advance}\n"
            ";ENDFOLD\n"
        )

        self._motion_lines.append(f"PTP {{A1 {pos['A1']}, A2 {pos['A2']}, A3 {pos['A3']}, A4 {pos['A4']}, A5 {pos['A5']}, A6 {pos['A6']}, E1 {pos['E1']}, E2 {pos['E2']}, E3 {pos['E3']}, E4 {pos['E4']}}}\n")

    def set_linear_speed(self, speed=0.05):
        """Sets the linear speed (TCP speed in m/sec)"""
        self._motion_lines.append(f"$VEL.CP={speed:.4f}\n")

    def set_advance(self, advance=3):
        """Sets the advance parameter (motion look-ahead)"""
        self._motion_lines.append(f"$ADVANCE={advance}\n")

    def add_move_ptp(self, *args, joint=False):
        """
        Adds a PTP movement.

        Parameters:
        - If `joint=True`, expects joint angles: A1, A2, A3, A4, A5, A6 (E1-E4 optional)
        - If `joint=False`, expects Cartesian coordinates: X, Y, Z, A, B, C
        """

        if joint:
            # Handle Joint Position PTP
            if len(args) < 6:
                raise ValueError("Joint PTP movement requires at least 6 joint angles (A1-A6).")
            
            A1, A2, A3, A4, A5, A6 = args[:6]
            extra_axes = ", ".join([f"E{i+1} {args[i+6]:.4f}" for i in range(len(args) - 6)]) if len(args) > 6 else ""
            self._motion_lines.append(f"PTP {{A1 {A1:.4f}, A2 {A2:.4f}, A3 {A3:.4f}, A4 {A4:.4f}, A5 {A5:.4f}, A6 {A6:.4f}{', ' + extra_axes if extra_axes else ''}}}\n")

        else:
            # Handle Cartesian Position PTP
            if len(args) < 3:
                raise ValueError("Cartesian PTP movement requires at least X, Y, Z coordinates.")
            
            x, y, z = args[:3]
            a, b, c = (args[3:] if len(args) >= 6 else (0, 90, 0))
            self._motion_lines.append(f"PTP {{X {x:.4f}, Y {y:.4f}, Z {z:.4f}, A {a:.4f}, B {b:.4f}, C {c:.4f}}}\n")
    def add_linear_move(self, x, y, z, a=0, b=90, c=0, approximate=True):
        """Adds a linear move (optionally approximated)."""
        suffix = " C_DIS" if approximate else ""
        self._motion_lines.append(f"LIN {{X {x:.4f}, Y {y:.4f}, Z {z:.4f}, A {a:.4f}, B {b:.4f}, C {c:.4f}}}{suffix}\n")

    def set_digital_output(self, output_no, value):
        """Sets a digital output (TRUE for ON, FALSE for OFF)"""
        value_str = "TRUE" if value else "FALSE"  # Convert to KRL-compatible format
        self._motion_lines.append(f"$OUT[{output_no}] = {value_str}\n")

    def set_analog_output(self, output_no, value):
        """Sets an analog output (e.g., adjust voltage or current)"""
        self._motion_lines.append(f"$ANOUT[{output_no}] = {value}\n")

    def wait_for_digital_input(self, input_no, value):
        """Waits for a digital input to reach a specified value"""
        self._motion_lines.append(f"WAIT FOR $IN[{input_no}] == {value}\n")

    def wait_time(self, time_sec):
        """Waits for a specific time in seconds"""
        self._motion_lines.append(f"WAIT SEC {time_sec}\n")

    def add_comment(self, msg):
        """Adds a comment inside the KRL program"""
        self._motion_lines.append(f"; {msg}\n")

    def add_print_statement(self, msg):
        """Adds a print statement to the teach pendant (equivalent to TPWrite)"""
        self._motion_lines.append(f"TPWrite \"{msg}\"\n")

    def end_program(self):
        """Ends the KRL program"""
        self.program = "".join(self._declaration_lines) + "".join(self._motion_lines) + "END\n"

    def save_program(self, filename="generated_module.src"):
        """Saves the KRL program to a file"""
        with open(filename, 'w') as file:
            file.write(self.program)
            
    def change_variable(self, variable_name, variable_value, wait=True):
        """
        Adds a variable update command to the KRL program.
        If wait is True, uses TRIGGER WHEN PATH for synchronized updates.
        If wait is False, directly assigns the value.

        Parameters:
        variable_name (str): Name of the KRL variable to update.
        variable_value (int, float, or bool): New value to assign to the variable.
        wait (bool): If True, uses TRIGGER for synchronized updates. If False, direct assignment.
        """
        if isinstance(variable_value, bool):
            variable_value = "TRUE" if variable_value else "FALSE"
        elif isinstance(variable_value, (int, float)):
            variable_value = str(variable_value)
        else:
            raise ValueError("Unsupported value type for KRL variable.")

        if wait:
            self._motion_lines.append(
                f"TRIGGER WHEN DISTANCE = 0 DELAY = 0 DO {variable_name} = {variable_value}\n"
            )
        else:
            self._motion_lines.append(f"{variable_name} = {variable_value}\n")
