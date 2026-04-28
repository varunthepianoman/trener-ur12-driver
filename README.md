# UR12 Driver

A ROS2 Jazzy C++ driver for the Universal Robots UR12 E-series robot, validated against URSim in Docker.

## Quick Start

```bash
# Start URSim + driver
docker compose up

# On first launch (or after ursim container restart): confirm safety in PolyScope
# Open http://localhost:6080 → Installation → Safety → Unlock → Apply
```

## Driving the Robot

Once the driver is active, use ROS2 CLI from inside the container:

```bash
docker compose exec ur12_driver bash
source /opt/ros/jazzy/setup.bash && source /ros2_ws/install/setup.bash

# Lifecycle
ros2 service call /ur/power_on std_srvs/srv/Trigger
ros2 service call /ur/brake_release std_srvs/srv/Trigger

# Motion (with live feedback)
ros2 action send_goal /ur/move_home ur12_driver_msgs/action/MoveHome '{}' --feedback
ros2 action send_goal /ur/move_first_joint ur12_driver_msgs/action/MoveFirstJoint '{joint_val: 1.0}' --feedback

# Joint state stream
ros2 topic echo /joint_states
```

Or run the automated demo:

```bash
docker compose exec ur12_driver bash /ros2_ws/demo.sh
```

## Architecture

- **`DashboardClient`** (port 29999) — text protocol: power on/off, brake release, play/pause/stop
- **`ScriptClient`** (port 30002) — sends URScript snippets for `move_home` and `move_first_joint`
- **`RtdeClient`** (port 30004) — binary protocol: streams 6-DOF joint positions + velocities at 125 Hz
- **`UrDriverNode`** — `rclcpp_lifecycle::LifecycleNode` wiring the three clients into ROS2 services, action servers, and a `/joint_states` publisher
- **`Ur12HardwareInterface`** — `hardware_interface::SystemInterface` plugin for ros2_control integration

## Services & Actions

| Interface | Type | Description |
|-----------|------|-------------|
| `/ur/power_on` | Service (Trigger) | Power on robot |
| `/ur/power_off` | Service (Trigger) | Power off robot |
| `/ur/brake_release` | Service (Trigger) | Release brakes |
| `/ur/play` | Service (Trigger) | Start loaded program |
| `/ur/pause` | Service (Trigger) | Pause program |
| `/ur/resume` | Service (Trigger) | Resume program |
| `/ur/stop` | Service (Trigger) | Stop program |
| `/ur/move_home` | Action (MoveHome) | Move to `[0, -π/2, π/2, 0, 0, 0]` with feedback |
| `/ur/move_first_joint` | Action (MoveFirstJoint) | Move joint 0 to target (rad) with feedback |
| `/joint_states` | Topic (JointState) | 6-DOF positions + velocities at 125 Hz |

## Running Tests

```bash
docker compose run --rm ur12_driver colcon test --packages-select ur12_driver
```

## Production Path

This PoC targets URSim. Production extensions would include:
- External Control URCap + 500 Hz reverse interface (replacing `ScriptClient`)
- MoveIt2 integration via ros2_control `JointTrajectoryController`
- Hardware-in-the-loop CI with URSim on GitHub Actions
