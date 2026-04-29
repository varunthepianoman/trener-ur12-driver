# UR12 Driver

A ROS2 Jazzy C++ driver for the Universal Robots UR12 E-series robot, validated against URSim in Docker.

## Quick Start

```bash
# Start URSim + driver. Use --build any time you've edited driver source —
# ros2_ws/ is COPY'd into the image, not volume-mounted.
docker compose up --build -d

# On first launch (or after the ursim container is recreated): confirm safety
# in PolyScope. Open http://localhost:6080 → Installation → Safety → Unlock
# → Apply. State persists across driver restarts.
```

## Driving the Robot

Once the driver is active, use ROS2 CLI from inside the container:

```bash
docker compose exec ur12_driver bash
source /opt/ros/jazzy/setup.bash && source /ros2_ws/install/setup.bash

# Lifecycle
ros2 service call /ur/power_on        std_srvs/srv/Trigger
ros2 service call /ur/brake_release   std_srvs/srv/Trigger

# Motion (with live feedback)
ros2 action send_goal /ur/move_home          ur12_driver_msgs/action/MoveHome        '{}' --feedback
ros2 action send_goal /ur/move_first_joint   ur12_driver_msgs/action/MoveFirstJoint  '{joint_val: 1.0}' --feedback
ros2 action send_goal /ur/move_joints        ur12_driver_msgs/action/MoveJoints \
  '{joint_positions: [0.5, -1.5707, 1.5707, 0.0, 0.0, 0.0]}' --feedback

# Joint state stream
ros2 topic echo /joint_states
```

Or run the automated demo:

```bash
docker compose exec ur12_driver bash /ros2_ws/demo.sh
```

## Architecture

- **`DashboardClient`** (port 29999) — text protocol: power on/off, brake release, play/pause/stop. Mutex-serialised so cancel handlers and service calls don't race on the socket.
- **`ScriptClient`** (port 30002) — sends URScript programs. Two patterns: file-loaded defs (`move_home`, `move_first_joint`) and inlined-in-code programs (`move_joints`).
- **`RtdeClient`** (port 30004) — binary protocol: streams 6-DOF joint positions + velocities at 125 Hz.
- **`UrDriverNode`** — `rclcpp_lifecycle::LifecycleNode` wiring the three clients into ROS2 services, action servers, and a `/joint_states` publisher.
- **`Ur12HardwareInterface`** — `hardware_interface::SystemInterface` plugin for ros2_control. State path (`read()`) is wired through to RTDE; command path (`write()`) is a no-op stub pending the reverse interface.

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
| `/ur/get_robot_mode` | Service (GetRobotMode) | Returns parsed `mode` (e.g. `RUNNING`, `IDLE`, `POWER_OFF`) plus the raw dashboard reply |
| `/ur/move_home` | Action (MoveHome) | Move to `[0, -π/2, π/2, 0, 0, 0]` with feedback |
| `/ur/move_first_joint` | Action (MoveFirstJoint) | Offset joint 0 by `joint_val` (rad) with feedback |
| `/ur/move_joints` | Action (MoveJoints) | Move all 6 joints to `joint_positions[6]` with feedback |
| `/joint_states` | Topic (JointState) | 6-DOF positions + velocities at 125 Hz |

## Parameters

| Name | Default | Description |
|------|---------|-------------|
| `robot_host` | `host.docker.internal` | UR / URSim host. Compose sets this to `ursim`. |
| `publish_rate` | `125.0` | RTDE rate (Hz). |
| `program_path` | `/programs/ur12_program.script` | URScript defs file loaded at construction. Compose overrides to `/ros2_ws/commands.script`. |
| `successive_goals` | `false` | If true, concurrent motion goals serialise on a single mutex (queue semantics). If false, the secondary interface's native replace semantics apply — the last script sent wins. Toggle at runtime: `ros2 param set /ur_driver_node successive_goals true`. |

## Tests

Unit tests (no live robot needed):

```bash
docker compose exec ur12_driver bash -c \
  "cd /ros2_ws && source install/setup.bash && \
   colcon test --packages-select ur12_driver --event-handlers console_direct+"
```

19 tests across three suites: Dashboard wire format (7), RTDE protocol parsing + handshake against a mock server (7), URScript wire format (5).

Stress tests (require URSim running, robot powered on):

```bash
docker compose exec ur12_driver bash /ros2_ws/stress_test.sh
```

Six scenarios: rapid-goal bisect, six-goal preemption, `successive_goals` queue, action-cancel mid-motion, 15s rate stability, lifecycle deactivate→activate.

## Production Path

This PoC targets URSim with the secondary-interface architecture. Production extensions in priority order:

1. **External Control URCap + 500 Hz reverse interface** — replaces `ScriptClient` with a streaming setpoint path. Required for any controller bandwidth above ~10 Hz, including MoveIt2.
2. **MoveIt2 integration** via the ros2_control `JointTrajectoryController` (the controller is loaded but inactive in the current scaffold).
3. **Speed scaling, protective-stop recovery, force/torque** — deployment hardening.
4. **Hardware-in-the-loop CI** — URSim in GitHub Actions on every PR.

See `DEVLOG.md` for the build narrative and the empirical finding on URSim secondary-interface semantics that motivates (1).
