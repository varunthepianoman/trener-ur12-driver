"""
Minimal RTDE (Real-Time Data Exchange) client — TCP port 30004.
Reads actual_q (joint positions) and actual_qd (joint velocities) at 125 Hz.

RTDE packet wire format (big-endian):
  uint16  size   — total byte length including this header
  uint8   type   — packet type code
  bytes   payload

Relevant type codes:
  86 ('V')  RTDE_REQUEST_PROTOCOL_VERSION
  79 ('O')  RTDE_CONTROL_PACKAGE_SETUP_OUTPUTS
  83 ('S')  RTDE_CONTROL_PACKAGE_START
  85 ('U')  RTDE_DATA_PACKAGE
"""

import socket
import struct
import threading
import logging
from dataclasses import dataclass, field

logger = logging.getLogger(__name__)

RTDE_PORT = 30004
PROTOCOL_VERSION = 2

# Packet type codes
TYPE_VERSION   = 86  # 'V'
TYPE_SETUP_OUT = 79  # 'O'
TYPE_START     = 83  # 'S'
TYPE_DATA      = 85  # 'U'

# Variables to subscribe to (6 doubles each)
OUTPUT_VARS = "actual_q,actual_qd"
HEADER = struct.Struct(">HB")  # size (uint16), type (uint8)


@dataclass
class RobotState:
    joint_positions: list[float] = field(default_factory=lambda: [0.0] * 6)
    joint_velocities: list[float] = field(default_factory=lambda: [0.0] * 6)


class RtdeClient:
    def __init__(self, host: str = "localhost", frequency: float = 125.0):
        self.host = host
        self.frequency = frequency
        self._sock: socket.socket | None = None
        self._state = RobotState()
        self._lock = threading.Lock()
        self._thread: threading.Thread | None = None
        self._running = False
        self._recipe_id: int = 0

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def start(self) -> bool:
        """Connect, handshake, and begin reading in a background thread."""
        if not self._connect():
            return False
        if not self._request_protocol_version():
            return False
        if not self._setup_outputs():
            return False
        if not self._send_start():
            return False
        self._running = True
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()
        logger.info("RTDE client started")
        return True

    def stop(self):
        self._running = False
        if self._sock:
            self._sock.close()
            self._sock = None

    def get_state(self) -> RobotState:
        with self._lock:
            return RobotState(
                joint_positions=list(self._state.joint_positions),
                joint_velocities=list(self._state.joint_velocities),
            )

    # ------------------------------------------------------------------
    # RTDE handshake
    # ------------------------------------------------------------------

    def _connect(self) -> bool:
        try:
            self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self._sock.settimeout(5.0)
            self._sock.connect((self.host, RTDE_PORT))
            logger.info(f"RTDE connected to {self.host}:{RTDE_PORT}")
            return True
        except Exception as e:
            logger.error(f"RTDE connect failed: {e}")
            return False

    def _request_protocol_version(self) -> bool:
        payload = struct.pack(">H", PROTOCOL_VERSION)
        self._send_packet(TYPE_VERSION, payload)
        ptype, data = self._recv_packet()
        if ptype != TYPE_VERSION:
            logger.error(f"Expected protocol version reply, got type {ptype}")
            return False
        accepted = struct.unpack_from(">?", data)[0]
        if not accepted:
            logger.error("RTDE protocol version not accepted")
            return False
        logger.debug("RTDE protocol v2 accepted")
        return True

    def _setup_outputs(self) -> bool:
        # payload = frequency (double) + variable names (utf-8)
        freq_bytes = struct.pack(">d", self.frequency)
        vars_bytes = OUTPUT_VARS.encode("utf-8")
        self._send_packet(TYPE_SETUP_OUT, freq_bytes + vars_bytes)
        ptype, data = self._recv_packet()
        if ptype != TYPE_SETUP_OUT:
            logger.error(f"Expected setup outputs reply, got type {ptype}")
            return False
        # Response: recipe id (uint8) + variable types (comma-separated string)
        self._recipe_id = data[0]
        types_str = data[1:].decode("utf-8", errors="replace")
        logger.info(f"RTDE output recipe id={self._recipe_id}, types={types_str}")
        if "NOT_FOUND" in types_str:
            logger.error("One or more RTDE variables not found")
            return False
        return True

    def _send_start(self) -> bool:
        self._send_packet(TYPE_START, b"")
        ptype, data = self._recv_packet()
        if ptype != TYPE_START:
            logger.error(f"Expected start reply, got type {ptype}")
            return False
        accepted = struct.unpack_from(">?", data)[0]
        if not accepted:
            logger.error("RTDE start not accepted")
            return False
        logger.debug("RTDE streaming started")
        return True

    # ------------------------------------------------------------------
    # Read loop
    # ------------------------------------------------------------------

    def _read_loop(self):
        self._sock.settimeout(2.0)
        while self._running:
            try:
                ptype, data = self._recv_packet()
                if ptype == TYPE_DATA:
                    self._parse_data(data)
            except socket.timeout:
                continue
            except Exception as e:
                if self._running:
                    logger.error(f"RTDE read error: {e}")
                break

    def _parse_data(self, data: bytes):
        # Data packet: uint8 recipe_id, then 6+6 doubles (actual_q, actual_qd)
        if len(data) < 1 + 96:  # 1 recipe byte + 12 doubles * 8 bytes
            return
        offset = 1  # skip recipe id byte
        positions = list(struct.unpack_from(">6d", data, offset))
        offset += 48
        velocities = list(struct.unpack_from(">6d", data, offset))
        with self._lock:
            self._state.joint_positions = positions
            self._state.joint_velocities = velocities

    # ------------------------------------------------------------------
    # Wire helpers
    # ------------------------------------------------------------------

    def _send_packet(self, ptype: int, payload: bytes):
        size = HEADER.size + len(payload)
        packet = HEADER.pack(size, ptype) + payload
        self._sock.sendall(packet)

    def _recv_packet(self) -> tuple[int, bytes]:
        header_bytes = self._recv_exact(HEADER.size)
        size, ptype = HEADER.unpack(header_bytes)
        payload_len = size - HEADER.size
        payload = self._recv_exact(payload_len) if payload_len > 0 else b""
        return ptype, payload

    def _recv_exact(self, n: int) -> bytes:
        buf = b""
        while len(buf) < n:
            chunk = self._sock.recv(n - len(buf))
            if not chunk:
                raise ConnectionError("RTDE socket closed")
            buf += chunk
        return buf
