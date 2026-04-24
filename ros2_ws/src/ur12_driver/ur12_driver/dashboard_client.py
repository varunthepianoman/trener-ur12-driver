"""
Dashboard Server client — TCP port 29999.
Handles robot controller lifecycle: power, brakes, program play/pause/stop/load.
"""

import socket
import time
import logging

logger = logging.getLogger(__name__)

DASHBOARD_PORT = 29999
TIMEOUT = 5.0


class DashboardClient:
    def __init__(self, host: str = "localhost"):
        self.host = host
        self._sock: socket.socket | None = None

    # ------------------------------------------------------------------
    # Connection
    # ------------------------------------------------------------------

    def connect(self) -> bool:
        try:
            self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self._sock.settimeout(TIMEOUT)
            self._sock.connect((self.host, DASHBOARD_PORT))
            # Read greeting: "Connected: Universal Robots Dashboard Server\n"
            greeting = self._recv_line()
            logger.info(f"Dashboard connected: {greeting.strip()}")
            return True
        except Exception as e:
            logger.error(f"Dashboard connect failed: {e}")
            self._sock = None
            return False

    def disconnect(self):
        if self._sock:
            try:
                self._send("quit")
            except Exception:
                pass
            self._sock.close()
            self._sock = None

    # ------------------------------------------------------------------
    # Lifecycle commands (spec §2)
    # ------------------------------------------------------------------

    def power_on(self) -> tuple[bool, str]:
        return self._cmd("power on")

    def power_off(self) -> tuple[bool, str]:
        return self._cmd("power off")

    def brake_release(self) -> tuple[bool, str]:
        return self._cmd("brake release")

    def load_program(self, program_path: str) -> tuple[bool, str]:
        return self._cmd(f"load {program_path}")

    def play(self) -> tuple[bool, str]:
        return self._cmd("play")

    def pause(self) -> tuple[bool, str]:
        return self._cmd("pause")

    def stop(self) -> tuple[bool, str]:
        return self._cmd("stop")

    def get_robot_mode(self) -> str:
        ok, reply = self._cmd("robotmode")
        return reply

    def get_program_state(self) -> str:
        ok, reply = self._cmd("programstate")
        return reply

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _cmd(self, command: str) -> tuple[bool, str]:
        if not self._sock:
            if not self.connect():
                return False, "Not connected"
        try:
            self._send(command)
            reply = self._recv_line().strip()
            logger.debug(f"Dashboard '{command}' → '{reply}'")
            # Most success replies start with a capital letter and contain no "Failed"
            ok = "failed" not in reply.lower() and "error" not in reply.lower()
            return ok, reply
        except Exception as e:
            logger.error(f"Dashboard command '{command}' failed: {e}")
            self._sock = None
            return False, str(e)

    def _send(self, text: str):
        self._sock.sendall((text + "\n").encode("utf-8"))

    def _recv_line(self) -> str:
        buf = b""
        while b"\n" not in buf:
            chunk = self._sock.recv(1024)
            if not chunk:
                raise ConnectionError("Dashboard server closed connection")
            buf += chunk
        return buf.decode("utf-8", errors="replace")
