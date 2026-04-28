# UR12 Driver — Weekly Plan

Take-home technical interview for Trener Robotics (RSE role).
Target: ROS2 C++ driver for a UR12 E-series robot, validated against URSim.

---

## Spec Requirements

1. **Robot controller lifecycle** — power on/off, brake release, play/pause/resume/stop via ROS2 services
2. **Remote procedure execution** — `move_home` and `move_first_joint(joint_val)` via ROS2
3. **Joint state publishing** — stream 6-DOF positions + velocities at up to 125Hz

---

## Day-by-Day Plan

| Day | Goal | Status |
|-----|------|--------|
| 1 | C++ driver PoC, working E2E against URSim | ✅ Done |
| 2 | Action servers for motion, lifecycle node wrapping | ✅ Done |
| 3 | ros2_control hardware interface scaffold + RTDE wired in | ✅ Done |
| 4 | Unit tests (mock socket) + integration tests with URSim | ✅ Done |
| 5 | Polish, demo script, stress-test | 🔲 |
| 6–7 | Presentation slides, architecture diagrams, talking points | 🔲 |

---

## Day 5 Detail — Polish, Demo Script, Stress-Test

### 5A — Polish
Clean up rough edges before the demo:

- **Remove debug noise**: strip `fprintf(stderr, ...)` hex-dump output from `RtdeClient` (or gate behind a `verbose` parameter). Fine for debugging, embarrassing in a demo.
- **Suppress compiler warnings**: fix any remaining `-Wunused`, `-Wpedantic` issues flagged at build time.
- **README**: write a concise top-level `README.md` covering: what it is, how to run (`docker compose up`), how to drive the robot (service/action CLI commands), architecture overview in 5 bullet points.
- **`docker compose` UX**: add a `healthcheck` to the `ur12_driver` service so `docker compose ps` shows `healthy` once the node is active, not just `running`.

### 5B — Demo Script
A single shell script (`demo.sh`) that drives the robot end-to-end and prints annotated output. Audience should be able to follow along without knowing ROS2.

Steps the script will execute (each with a printed header):
1. `power_on` → wait for "Powering on"
2. `brake_release` → wait for "Brake releasing"
3. Sleep 5 s (robot arms up)
4. Show current joint positions from `/joint_states`
5. `move_home` action → stream feedback until SUCCEEDED
6. Show joint positions at home
7. `move_first_joint 1.0` action → stream feedback until SUCCEEDED
8. Show joint positions (j0 ≈ 1.0 rad)
9. `move_first_joint -1.0` action → return to home
10. `power_off`

The script should be runnable with: `docker compose exec ur12_driver bash /ros2_ws/demo.sh`

### 5C — Stress Tests
Validate the driver holds up under conditions the reviewer might probe:

| Scenario | What to check | Pass criterion |
|----------|--------------|----------------|
| **Rapid successive goals** | Send 10 `move_first_joint` goals back-to-back with 0.1 rad increments | All succeed or are cleanly preempted; no crash or deadlock |
| **Goal cancellation** | Send a 1.5 rad goal, cancel after 500 ms | `CANCELED` result, robot stops within 1 s |
| **RTDE reconnect** | Kill and restart the `ursim` container mid-stream | Driver reconnects automatically, `/joint_states` resumes within 10 s |
| **Long-running stability** | Let the driver idle at 125 Hz for 60 s | No memory growth, no dropped connection |
| **Lifecycle transitions** | `deactivate` → `activate` via `ros2 lifecycle set` | Services/actions come back cleanly, RTDE reconnects |

Results to be documented in DEVLOG Day 5 entry.

---

## Day 2 Detail — Action Servers + Lifecycle Node

### Action servers
Convert `/ur/move_home` and `/ur/move_first_joint` from simple services to **rclcpp_action** action servers:
- Report progress feedback while motion is executing (poll RTDE joint positions)
- Support cancellation (send a stop command to dashboard)
- Define a `MotionFeedback.action` message

### Lifecycle node
Wrap `UrDriverNode` as `rclcpp_lifecycle::LifecycleNode`:
- `configure` — create RTDE/dashboard/script clients, connect
- `activate` — start RTDE streaming, advertise publisher + services
- `deactivate` — stop streaming
- `cleanup` / `shutdown` — close sockets

---

## Day 3 Detail — ros2_control

Scaffold a `hardware_interface::SystemInterface` plugin:
- `on_init` — load params (host, rate)
- `read` — pull joint positions + velocities from `RtdeClient::get_state()`
- `write` — send velocity/position commands (scaffold only; real trajectory execution is future work)
- Register with `ros2_control` via `pluginlib`
- Wire up a `JointStatePublisher` from the hardware interface state

---

## Day 4 Detail — Tests

### Unit tests (GTest, no hardware)
- `DashboardClient`: mock TCP server, verify command strings sent + response parsing
- `RtdeClient`: craft raw RTDE packets, verify handshake logic + `parse_data` byte layout

### Integration tests
- Spin up URSim in CI (or assume it's running), call power_on → brake_release → move_home, assert joint states change

---

## Production Architecture Notes
(For presentation — what this PoC would grow into)

- Replace `ScriptClient` with **External Control URCap** + reverse interface at 500Hz
- Use **ros2_control** trajectory controllers + **MoveIt2** for planning
- Promote to **lifecycle nodes** for clean robot bringup/teardown
- Add **action servers** with proper feedback + preemption
- RTDE recipe expanded: add `target_q`, `actual_TCP_pose`, `robot_mode`, `safety_mode`
- CI: URSim in Docker on GitHub Actions, colcon test gated on green

---

## Stretch Goals / Future Extensions

### Safety & Reliability
- **Watchdog timer**: if the ROS2 node stops sending heartbeats (e.g. crashes, hangs), the robot stops automatically. Implemented as a periodic RTDE write or URScript `watchdog()` call; timeout triggers a protective stop.
- **Protective stop recovery**: detect `safetymode: PROTECTIVE_STOP` via RTDE `safety_status_bits`, auto-unlock via dashboard `unlock protective stop`, re-home, and resume — with configurable retry limits.
- **E-stop monitoring**: subscribe to `safety_status_bits` and publish a `/ur/estop` latched topic; trigger lifecycle `deactivate` automatically on E-stop.
- **Connection watchdog**: if RTDE stream goes silent for >N packets, declare robot lost, deactivate lifecycle node, and alert via diagnostics.

### Motion & Control
- **Full ros2_control integration**: implement `write()` to forward joint position/velocity commands from a `JointTrajectoryController` via the reverse interface, enabling closed-loop trajectory execution at 500Hz.
- **MoveIt2 integration**: expose a `move_group` interface so motion can be planned in Cartesian or joint space with collision checking.
- **Cartesian motion**: add a `move_tcp` action server that accepts a `geometry_msgs/PoseStamped` goal and sends a `movel`/`movep` URScript.
- **Speed scaling**: expose a `/ur/set_speed_fraction` service mapping to dashboard `speed n` command; integrate with ros2_control's speed scaling interface.
- **Force/torque streaming**: add `actual_TCP_force` (VECTOR6D) to the RTDE recipe and publish on `/ur/wrench` as `geometry_msgs/WrenchStamped`.

### Observability & Diagnostics
- **ROS2 diagnostics**: publish `diagnostic_msgs/DiagnosticArray` with robot mode, safety mode, joint temperatures, and RTDE packet loss rate.
- **RTDE expanded recipe**: stream `robot_mode`, `safety_mode`, `joint_temperatures`, `actual_TCP_pose`, `target_q`, `speed_scaling` — feed into diagnostics and a richer joint state.
- **TF2 publisher**: compute and broadcast forward kinematics transforms (`base_link` → `tool0`) from joint positions, enabling rviz visualisation without a separate robot_state_publisher.
- **Parameter server integration**: expose RTDE frequency, joint names, and script paths as ROS2 parameters with `rcl_interfaces` dynamic reconfigure support.

### Developer Experience
- **ur_rtde replacement path**: the custom RTDE client covers the PoC; production path is the `ur_rtde` C++ library (maintained by UR), which handles edge cases (RTDE v1 fallback, protective stop mid-stream, large recipes). Document the migration path.
- **Hardware-in-the-loop CI**: GitHub Actions workflow spins up `universalrobots/ursim_e-series`, runs the full integration test suite, posts pass/fail on each PR.
- **Simulation flag**: `--ros-args -p simulation:=true` skips real TCP connections and plays back a recorded RTDE trace, enabling offline development and CI without URSim.
- **AI/control loop decoupling**: clean interface boundary between the low-level driver (this package) and a high-level AI planner node — driver exposes only standard ROS2 interfaces (`/joint_states`, action servers, `/tf`), planner sends goals, no URScript awareness required at the AI layer.
