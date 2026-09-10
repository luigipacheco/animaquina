# Copyright (C) 2026 Luis Arturo Pacheco
# SPDX-License-Identifier: GPL-3.0-or-later
import socket
import time
import struct

class VariableError(Exception):
    """A variable-level failure: the named variable could not be read or written.

    Distinct from a transport failure. The socket is still healthy - typically
    the variable is simply not declared in $CONFIG.DAT. Callers should report it
    and carry on, never reconnect.
    """


class KUKA:
    # Minimum seconds between reconnect attempts. read()/write() transparently
    # reconnect after any failure, and both are called from the poll thread at
    # the slot poll rate. Without a floor here, one dead link turns into a
    # reconnect storm: a fresh TCP connection per failed read, tens per second.
    # On Windows every one of those leaves a TIME_WAIT entry for ~4 minutes, so
    # the machine runs out of ephemeral ports and *all* networking stalls - the
    # same class of system-wide symptom as the UR stopScript() leak.
    # The C3 Bridge also accepts only a handful of concurrent connections.
    RECONNECT_MIN_INTERVAL = 0.5

    def __init__(self, TCP_IP):
        self.TCP_IP = TCP_IP
        self.TCP_PORT = 7000
        self.client = None
        self.connected = False
        self._last_connect_attempt = 0.0
        self.connect()

    def connect(self):
        """Establish connection to the KUKA robot"""
        # Never replace a socket without closing it, and never hammer the
        # bridge: both turn a transient fault into a resource leak.
        now = time.monotonic()
        since = now - self._last_connect_attempt
        if self._last_connect_attempt and since < self.RECONNECT_MIN_INTERVAL:
            raise Exception(
                f"Reconnect throttled ({since:.2f}s < {self.RECONNECT_MIN_INTERVAL}s)"
            )
        self._last_connect_attempt = now

        # Close any previous socket first. The old code assigned over
        # self.client, orphaning a still-open fd on every reconnect.
        if self.client is not None:
            try:
                self.client.close()
            except Exception:
                pass
            self.client = None

        try:
            # Create new socket
            self.client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.client.settimeout(2.0)  # 2 second timeout

            # Connect to robot
            self.client.connect((self.TCP_IP, self.TCP_PORT))
            self.connected = True
            return True

        except Exception as e:
            self.connected = False
            if self.client:
                try:
                    self.client.close()
                except:
                    pass
                self.client = None
            raise Exception(f"Connection failed: {str(e)}")

    def disconnect(self):
        """Close the connection to the KUKA robot"""
        if self.client:
            try:
                self.client.close()
            except:
                pass
            self.client = None
        self.connected = False

    def send(self, var, val, msgID=0):
        """
        Send a message to the KUKA robot
        Message format:
        - msg ID in HEX (2 bytes)
        - msg length in HEX (2 bytes)
        - read (0) or write (1) (1 byte)
        - variable name length in HEX (2 bytes)
        - variable name in ASCII
        - variable value length in HEX (2 bytes)
        - variable value in ASCII
        """
        if not self.connected:
            raise Exception("Not connected to robot")

        try:
            msg = bytearray()
            temp = bytearray()
            
            if val != "":
                val = str(val)
                msg.append((len(val) & 0xff00) >> 8)  # MSB of variable value length
                msg.append((len(val) & 0x00ff))       # LSB of variable value length
                msg.extend(map(ord, val))             # Variable value in ASCII
            
            temp.append(bool(val))                    # Read (0) or Write (1)
            temp.append(((len(var)) & 0xff00) >> 8)   # MSB of variable name length
            temp.append((len(var)) & 0x00ff)          # LSB of variable name length
            temp.extend(map(ord, var))                # Variable name in ASCII
            
            msg = temp + msg
            del temp[:]
            
            temp.append((msgID & 0xff00) >> 8)        # MSB of message ID
            temp.append(msgID & 0x00ff)               # LSB of message ID
            temp.append((len(msg) & 0xff00) >> 8)     # MSB of message length
            temp.append((len(msg) & 0x00ff))          # LSB of message length
            
            msg = temp + msg
            
            # Send message and get response.
            # sendall, not send: send may write only part of the buffer, which
            # desynchronises the stream for every later transaction.
            self.client.sendall(msg)

            # Read the reply framed by its own length header instead of a single
            # recv(1024). A response split across TCP segments used to come back
            # truncated, fail to parse, and mark the link dead - which then
            # triggered a reconnect. Under the poll rates this driver runs at
            # that was the main trigger of the reconnect storm.
            header = self._recv_exact(4)          # msg ID (2) + payload length (2)
            payload_len = (header[2] << 8) | header[3]
            payload = self._recv_exact(payload_len) if payload_len else b""
            response = header + payload

            if not response:
                raise Exception("No response from robot")

            return response

        except socket.timeout:
            raise Exception("Timeout while communicating with robot")
        except socket.error as e:
            self.connected = False
            raise Exception(f"Socket error: {str(e)}")
        except Exception as e:
            raise Exception(f"Error sending message: {str(e)}")

    def __get_var(self, msg):
        """
        Parse response from KUKA robot
        Response format:
        - msg ID in HEX (2 bytes)
        - msg length in HEX (2 bytes)
        - read (0) or write (1) (1 byte)
        - variable value length in HEX (2 bytes)
        - variable value in ASCII
        """
        try:
            if len(msg) < 7:
                raise Exception("Invalid response length")
                
            lsb = int(msg[5])
            msb = int(msg[6])
            lenValue = (lsb << 8 | msb)
            
            if len(msg) < 7 + lenValue:
                raise Exception("Response too short for value length")
                
            return str(msg[7:7+lenValue], 'utf-8')
            
        except Exception as e:
            raise Exception(f"Error parsing response: {str(e)}")

    def read(self, var, msgID=0):
        """Read a variable from the KUKA robot.

        A variable-level failure (name not declared in $CONFIG.DAT, unparseable
        reply) raises VariableError and leaves the connection alone. Only a real
        transport fault marks the link dead - send() does that itself.

        This distinction matters a lot. The old code set connected = False on
        *any* exception, so reading one undeclared variable poisoned a perfectly
        healthy socket: the next read reconnected, and since the poll thread
        re-reads the same variable every cycle, the link was torn down and
        rebuilt at the poll rate indefinitely.
        """
        if not self.connected:
            self.connect()

        try:
            return self.__get_var(self.send(var, "", msgID))
        except socket.error as e:
            self.connected = False
            raise Exception(f"Error reading variable: {str(e)}")
        except Exception as e:
            if not self.connected:
                # send() already classified this as a transport fault.
                raise Exception(f"Error reading variable: {str(e)}")
            raise VariableError(f"Error reading variable {var}: {str(e)}")

    def write(self, var, val, msgID=0):
        """Write a value to a variable on the KUKA robot.

        Same error split as read(): a bad variable or value does not invalidate
        the connection.
        """
        if not self.connected:
            self.connect()

        if val == "":
            raise VariableError("Value cannot be empty")

        try:
            return self.__get_var(self.send(var, val, msgID))
        except socket.error as e:
            self.connected = False
            raise Exception(f"Error writing variable: {str(e)}")
        except Exception as e:
            if not self.connected:
                raise Exception(f"Error writing variable: {str(e)}")
            raise VariableError(f"Error writing variable {var}: {str(e)}")

    def _recv_exact(self, n):
        """Read exactly n bytes from socket, retrying on partial reads."""
        buf = b""
        while len(buf) < n:
            chunk = self.client.recv(n - len(buf))
            if not chunk:
                raise Exception("Connection closed while reading")
            buf += chunk
        return buf

    def create_header(self, tag_id, message_type, payload_length):
        # Message Length = 1 (message type byte) + payload
        # Per C3 Bridge spec: excludes Tag ID (2) and Message Length (2) fields
        message_length = 1 + payload_length
        return struct.pack(">HHB", tag_id, message_length, message_type)

    def send_message(self, message):
        self.client.sendall(message)

    def receive_response(self):
        header_data = self._recv_exact(5)
        tag_id, message_length, message_type = struct.unpack(">HHB", header_data)
        # message_length = 1 (type, already read) + payload + 3 (footer)
        # payload bytes = message_length - 1 - 3 = message_length - 4
        payload_length = message_length - 4
        payload_data = self._recv_exact(payload_length) if payload_length > 0 else b""
        footer_data = self._recv_exact(3)
        error_code, success_flag = struct.unpack(">HB", footer_data)
        return tag_id, payload_data, {"error_code": error_code, "success": bool(success_flag)}

    def upload_file(self, local_file_path, remote_file_name, copy_flags=0):
        """
        Upload a file to the KUKA robot controller via C3 Bridge.
        Uses the Write File Content flow (msg #29): Begin → Data chunks → End.
        Matches the C# reference (c3sharp SyncFileStream).

        Requires C3 Bridge server on the robot (NOT plain KukavarProxy).

        Args:
            local_file_path (str): Path to the local file to upload
            remote_file_name (str): Name/path on the robot controller
            copy_flags (int): CopyFlag bitmask for WriteEnd (0 = None)

        Returns:
            str: Empty string on success, error message on failure.
        """
        if not self.connected:
            return "Not connected to robot"

        try:
            with open(local_file_path, 'rb') as f:
                file_content = f.read()
                file_size = len(file_content)

            tag_id = 1

            # Step 1: FileIoBegin — allocate buffer of file_size
            payload_begin = struct.pack(">BI", 1, file_size)
            header_begin = self.create_header(tag_id, 29, len(payload_begin))
            self.send_message(header_begin + payload_begin)
            _, _, footer_resp = self.receive_response()
            if not footer_resp or not footer_resp['success']:
                return f"FileIoBegin failed (error_code={footer_resp.get('error_code') if footer_resp else '?'}). Is C3 Bridge running? (KukavarProxy alone does not support file operations)"

            # Step 2: FileIoData — write chunks
            chunk_size = 4096
            offset = 0
            while offset < file_size:
                chunk = file_content[offset:offset + chunk_size]
                payload_data = struct.pack(">BII", 2, offset, len(chunk)) + chunk
                header_data = self.create_header(tag_id + 1, 29, len(payload_data))
                self.send_message(header_data + payload_data)
                _, _, footer_resp = self.receive_response()
                if not footer_resp or not footer_resp['success']:
                    return f"FileIoData failed at offset {offset} (error_code={footer_resp.get('error_code') if footer_resp else '?'})"
                offset += len(chunk)

            # Step 3: FileIoEnd — flush buffer to file on disk
            file_name_encoded = remote_file_name.encode('utf-16-le')
            payload_final = struct.pack(">BIH", 4, copy_flags, len(remote_file_name)) + file_name_encoded
            header_final = self.create_header(tag_id + 2, 29, len(payload_final))
            self.send_message(header_final + payload_final)
            _, _, footer_resp = self.receive_response()
            if not footer_resp or not footer_resp['success']:
                return f"FileIoEnd failed for '{remote_file_name}' (error_code={footer_resp.get('error_code') if footer_resp else '?'})"

            print(f"File '{local_file_path}' uploaded as '{remote_file_name}' successfully.")
            return ""

        except socket.timeout:
            return "Timeout during upload — is C3 Bridge running? (KukavarProxy alone does not support file operations)"
        except Exception as e:
            return f"Upload error: {str(e)}"

    def select_program(self, program_name):
        """
        Select a program on the KUKA robot controller via C3 Bridge.
        Message #10 ProgramControl, Subtype II (command=5 Select).

        Args:
            program_name (str): Name/path of the program to select

        Returns:
            bool: True if selection was successful, False otherwise
        """
        if not self.connected:
            raise Exception("Not connected to robot")

        try:
            # Payload: command(1) + interpreter(2) + LN(2) + name(LN*2) + LP(2) + params(LP*2) + force(1)
            # Try ROBOT first, then SUBMIT as fallback (controller setups differ).
            last_err = ""
            for interpreter in (1, 0):
                tag_id = 1
                name_encoded = program_name.encode('utf-16-le')
                payload = struct.pack(">BH", 5, interpreter)          # command=Select
                payload += struct.pack(">H", len(program_name))       # LN (char count)
                payload += name_encoded                                # name in UTF-16-LE
                payload += struct.pack(">H", 0)                        # LP = 0 (no parameters)
                payload += struct.pack(">B", 0)                        # force = False
                header = self.create_header(tag_id, 10, len(payload))
                self.send_message(header + payload)
                _, _, footer_resp = self.receive_response()
                if footer_resp and footer_resp['success']:
                    print(f"Program '{program_name}' selected successfully (interp={interpreter}).")
                    return ""
                last_err = (
                    f"Select failed for '{program_name}' (interp={interpreter}, "
                    f"error_code={footer_resp.get('error_code') if footer_resp else '?'})"
                )
            return f"{last_err}. Is C3 Bridge running?"

        except socket.timeout:
            return "Timeout selecting program — is C3 Bridge running?"
        except Exception as e:
            return f"Select error: {str(e)}"

    def _program_control_simple(self, command, command_name="Command", interpreter=1):
        """
        Send a ProgramControl Subtype I command via C3 Bridge (message type 10).
        Subtype I payload: command(1B) + interpreter(2B).
        Used for Reset(1), Start(2), Stop(3), Cancel(4).

        Args:
            command (int): 1=Reset, 2=Start, 3=Stop, 4=Cancel
            command_name (str): Human-readable name for error messages
            interpreter (int): 0=Submit interpreter, 1=Robot interpreter

        Returns:
            str: Empty string on success, error message on failure.
        """
        if not self.connected:
            return "Not connected to robot"

        try:
            # Subtype I: command(1B) + interpreter(2B)
            # Try requested interpreter first, then fallback to the other.
            order = (interpreter, 0 if interpreter == 1 else 1)
            last_err = ""
            for interp in order:
                tag_id = 1
                payload = struct.pack(">BH", command, interp)
                header = self.create_header(tag_id, 10, len(payload))
                self.send_message(header + payload)
                _, _, footer_resp = self.receive_response()
                if footer_resp and footer_resp['success']:
                    return ""
                last_err = (
                    f"{command_name} failed (interp={interp}, "
                    f"error_code={footer_resp.get('error_code') if footer_resp else '?'})"
                )
            return f"{last_err}. Is C3 Bridge running?"

        except socket.timeout:
            return f"Timeout during {command_name} — is C3 Bridge running?"
        except Exception as e:
            return f"{command_name} error: {str(e)}"

    def start_program(self):
        """Start the currently selected program. Message #10 Subtype I, command=2."""
        return self._program_control_simple(2, "Start")

    def stop_program(self):
        """Stop the running program. Message #10 Subtype I, command=3."""
        return self._program_control_simple(3, "Stop")

    def cancel_program(self):
        """Cancel the running program. Message #10 Subtype I, command=4."""
        return self._program_control_simple(4, "Cancel")

    def reset_program(self):
        """Reset the program pointer to start. Message #10 Subtype I, command=1."""
        return self._program_control_simple(1, "Reset")

    def __del__(self):
        """Cleanup when object is destroyed"""
        self.disconnect()
