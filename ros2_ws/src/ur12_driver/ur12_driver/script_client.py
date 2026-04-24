"""
Secondary Script Interface client — TCP port 30002.
Sends URScript programs to the robot for immediate execution.

Note: Scripts sent here run *alongside* the loaded dashboard program.
For PoC we send self-contained scripts that include the function definition
and a call — no dependency on a pre-loaded program.
"""

import socket
import logging

logger = logging.getLogger(__name__)

SCRIPT_PORT = 30002
TIMEOUT = 5.0


class ScriptClient:
    def __init__(self, host: str = "localhost"):
        self.host = host

    def send_script(self, script: str) -> bool:
        """Open a fresh connection, send the URScript text, close."""
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(TIMEOUT)
            sock.connect((self.host, SCRIPT_PORT))
            payload = script.encode("utf-8")
            sock.sendall(payload)
            sock.close()
            logger.debug(f"URScript sent ({len(payload)} bytes)")
            return True
        except Exception as e:
            logger.error(f"ScriptClient send failed: {e}")
            return False

    # ------------------------------------------------------------------
    # Pre-built motion commands (spec §2)
    # ------------------------------------------------------------------

    def move_home(self) -> bool:
        script = (
            "def move_home():\n"
            "  movej([0, -1.5707, 1.5707, 0, 0, 0], a=1.4, v=1.05)\n"
            "end\n"
            "move_home()\n"
        )
        return self.send_script(script)

    def move_first_joint(self, joint_val: float) -> bool:
        """Move joint 0 to an absolute position (radians)."""
        script = (
            "def move_first_joint(joint_val):\n"
            "  curr_joint = get_actual_joint_positions()\n"
            f"  curr_joint[0] = curr_joint[0] + joint_val\n"
            "  movej(curr_joint, a=1.4, v=1.05)\n"
            "end\n"
            f"move_first_joint({joint_val})\n"
        )
        return self.send_script(script)
