"""
UR12 Driver Node — the main ROS2 node.

Services (std_srvs/Trigger):
  /ur/power_on
  /ur/power_off
  /ur/brake_release
  /ur/play
  /ur/pause
  /ur/resume      (alias for play — UR dashboard uses 'play' to resume)
  /ur/stop
  /ur/move_home

Service (ur12_driver_msgs/MoveFirstJoint):
  /ur/move_first_joint

Topic (sensor_msgs/JointState):
  /joint_states   @ 125 Hz
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

from ur12_driver_msgs.srv import MoveFirstJoint

from ur12_driver.dashboard_client import DashboardClient
from ur12_driver.script_client import ScriptClient
from ur12_driver.rtde_client import RtdeClient

JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]


class UrDriverNode(Node):
    def __init__(self):
        super().__init__("ur_driver_node")

        # Parameters
        self.declare_parameter("robot_host", "host.docker.internal")
        self.declare_parameter("publish_rate", 125.0)
        self.declare_parameter("program_path", "/programs/ur12_program.script")

        host = self.get_parameter("robot_host").get_parameter_value().string_value
        rate = self.get_parameter("publish_rate").get_parameter_value().double_value
        self._program_path = self.get_parameter("program_path").get_parameter_value().string_value

        self.get_logger().info(f"Connecting to UR at {host}")

        # Clients
        self._dashboard = DashboardClient(host)
        self._script = ScriptClient(host)
        self._rtde = RtdeClient(host, frequency=rate)

        # Connect dashboard eagerly; RTDE started after
        self._dashboard.connect()

        # Start RTDE streaming
        if not self._rtde.start():
            self.get_logger().warn("RTDE not available — joint states will be zeros")

        # Publisher
        self._js_pub = self.create_publisher(JointState, "/joint_states", 10)
        self._js_timer = self.create_timer(1.0 / rate, self._publish_joint_states)

        # Lifecycle services
        self._make_trigger("/ur/power_on",     self._dashboard.power_on)
        self._make_trigger("/ur/power_off",    self._dashboard.power_off)
        self._make_trigger("/ur/brake_release",self._dashboard.brake_release)
        self._make_trigger("/ur/play",         self._dashboard.play)
        self._make_trigger("/ur/pause",        self._dashboard.pause)
        self._make_trigger("/ur/resume",       self._dashboard.play)   # resume = play
        self._make_trigger("/ur/stop",         self._dashboard.stop)

        # Motion services
        self._make_trigger("/ur/move_home", self._handle_move_home)
        self.create_service(
            MoveFirstJoint, "/ur/move_first_joint", self._handle_move_first_joint
        )

        self.get_logger().info("UR12 driver node ready")

    # ------------------------------------------------------------------
    # Joint state publisher
    # ------------------------------------------------------------------

    def _publish_joint_states(self):
        state = self._rtde.get_state()
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINT_NAMES
        msg.position = state.joint_positions
        msg.velocity = state.joint_velocities
        msg.effort = []
        self._js_pub.publish(msg)

    # ------------------------------------------------------------------
    # Service handlers
    # ------------------------------------------------------------------

    def _handle_move_home(self):
        ok = self._script.move_home()
        return ok, "move_home sent" if ok else "ScriptClient failed"

    def _handle_move_first_joint(self, request, response):
        ok = self._script.move_first_joint(request.joint_val)
        response.success = ok
        response.message = (
            f"move_first_joint({request.joint_val:.4f}) sent"
            if ok else "ScriptClient failed"
        )
        return response

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _make_trigger(self, name: str, fn):
        """Create a Trigger service whose handler calls fn() → (bool, str)."""
        def handler(_, response):
            ok, msg = fn()
            response.success = ok
            response.message = msg
            return response

        self.create_service(Trigger, name, handler)

    def destroy_node(self):
        self._rtde.stop()
        self._dashboard.disconnect()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = UrDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
