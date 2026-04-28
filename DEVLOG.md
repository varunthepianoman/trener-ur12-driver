# UR12 Driver — Dev Log

**Total build effort: ~1.5–2 days of active development, plus ~1 day reading back through the code to fully internalize what was written. The "Parts" below are logical milestones, not calendar days.**

---

## Part 1

### What we built
A ROS2 C++ driver node for a UR12 E-series robot, running against URSim in Docker.

### Architecture
Three header-only C++ client classes, each owning one TCP connection to the robot:

| Client | Port | Purpose |
|--------|------|---------|
| `DashboardClient` | 29999 | Text protocol — power on/off, brake release, play/pause/stop |
| `ScriptClient` | 30002 | Sends URScript snippets for motion commands |
| `RtdeClient` | 30004 | Binary protocol — streams joint positions + velocities at 125Hz |

The `UrDriverNode` wires them together and exposes:
- **7 Trigger services**: `/ur/power_on`, `/ur/power_off`, `/ur/brake_release`, `/ur/play`, `/ur/pause`, `/ur/resume`, `/ur/stop`
- **2 motion services**: `/ur/move_home` (Trigger), `/ur/move_first_joint` (custom MoveFirstJoint.srv)
- **1 publisher**: `/joint_states` at 125Hz

### Key bugs fixed
1. **RTDE handshake**: Stray `recipe_id` byte prepended to SETUP_OUTPUTS request — protocol doesn't include it in the *request*, only the *response*. Caused every variable name to be malformed → NOT_FOUND.
2. **URSim safety**: RTDE won't serve data until safety is confirmed in PolyScope UI (Installation → Safety → Unlock → Apply). Persistent across driver restarts as long as the `ursim` container isn't recreated. Needs re-confirming after `docker compose down`.
3. **Dashboard greeting leak**: URSim sends a "Connected" banner on TCP connect. Timing meant we'd sometimes miss it in `connect()` and it would leak into the first command's response, shifting all replies by one. Fixed by retrying `recv_line` in `connect()`.
4. **URScript parameterized defs**: URSim's secondary interface silently drops scripts where the entry function takes parameters. Fixed by inlining arguments as local variables inside a `def prog(): ... end` wrapper.

### Verified E2E
- `/joint_states` streaming at 125Hz with real positions + velocities from RTDE
- `/ur/power_on`, `/ur/brake_release` returning correct dashboard responses
- `/ur/move_home` moves to home pose `[0, -1.57, 1.57, 0, 0, 0]`
- `/ur/move_first_joint` with `joint_val: 0.9` → joint 0 moves to 0.9 rad

**Completed: ~3–4 hours**

---

## Part 2

### What we built
Upgraded the driver node to a **lifecycle node** and replaced the two motion services with **action servers**.

### Lifecycle node (`rclcpp_lifecycle::LifecycleNode`)
`UrDriverNode` now extends `LifecycleNode` with five state transitions:
- `on_configure` — instantiate clients, connect dashboard, create publisher + services + action servers
- `on_activate` — start RTDE background thread, start joint-state timer, activate managed publisher
- `on_deactivate` — cancel timer, stop RTDE
- `on_cleanup` / `on_shutdown` — disconnect dashboard, reset all resources

The node self-configures and self-activates via a one-shot 200ms timer after the executor starts (production systems would use `ros2_lifecycle_manager`).

### Action servers (`rclcpp_action`)
Two new action message types in `ur12_driver_msgs/action/`:

| Action | Goal | Feedback | Result |
|--------|------|----------|--------|
| `/ur/move_home` | (empty) | `current_positions[6]`, `distance_to_goal` | `success`, `message` |
| `/ur/move_first_joint` | `joint_val` | `current_position`, `distance_remaining` | `success`, `message` |

Both poll RTDE at 10 Hz, converge at < 0.02 rad error, timeout at 15 s, and cancel by calling `dashboard_->stop()`.

### Verified E2E
```
ros2 action send_goal /ur/move_home ur12_driver_msgs/action/MoveHome '{}' --feedback
# → feedback streaming distance_to_goal, result: success=true "Reached home"

ros2 action send_goal /ur/move_first_joint ur12_driver_msgs/action/MoveFirstJoint '{joint_val: 1.0}' --feedback
# → feedback streaming distance_remaining → 0, result: success=true "Reached target"
```

**Completed: ~1 hour**

---

## Part 3

### What we built
A **ros2_control hardware interface plugin** (`Ur12HardwareInterface`) that wires RTDE into the standard ros2_control state/command interface layer.

### `Ur12HardwareInterface` (`hardware_interface::SystemInterface`)
Registered via pluginlib as `ur12_driver/Ur12HardwareInterface`.

| Lifecycle method | Action |
|-----------------|--------|
| `on_init` | Validate 6-joint URDF, read `robot_host` + `publish_rate` params |
| `on_configure` | Create `RtdeClient`, start background RTDE thread |
| `on_activate` / `on_deactivate` | No-op / stop RTDE |
| `read()` | Copy `RtdeClient::get_state()` → `hw_positions_` / `hw_velocities_` buffers |
| `write()` | **Scaffold — no-op.** Production path: External Control URCap reverse interface at 500 Hz |

Exports 6× `position` + 6× `velocity` state interfaces and 6× `position` command interfaces.

### Supporting files
- `urdf/ur12.urdf` — minimal 6-joint URDF with `<ros2_control>` hardware tag
- `config/ur12_controllers.yaml` — `JointStateBroadcaster` (active) + `JointTrajectoryController` (inactive, scaffold)
- `launch/ur12_bringup.launch.py` — brings up driver node + robot_state_publisher + controller_manager + controller spawners in one command

### Verified
Launch output confirmed:
```
[controller_manager]: Loaded hardware 'Ur12HardwareInterface' from plugin 'ur12_driver/Ur12HardwareInterface'
[Ur12HardwareInterface]: Initialised — robot_host=ursim rate=125 Hz
[Ur12HardwareInterface]: RTDE connecting in background (robot must be powered on)
[Ur12HardwareInterface]: Activated
[RtdeClient] Handshake successful — streaming joint states
```

**Completed: ~45 minutes**

---

## Part 4

### What we built
Real unit tests — no placeholders. 14 tests across two suites, all passing.

### Test infrastructure changes
- Added optional `port` parameter to `DashboardClient` and `RtdeClient` constructors so tests can inject a mock server on a random OS-assigned port.
- Added two public static helpers to `RtdeClient` for white-box testing without a live socket:
  - `parse_data_packet(data)` — returns `RobotState` from a raw DATA_PACKAGE payload
  - `build_setup_outputs_payload(freq, vars)` — builds a v2 SETUP_OUTPUTS request for wire-format verification

### `test_dashboard_client` — 7 tests (mock TCP server)
| Test | What it verifies |
|------|-----------------|
| `PowerOnSendsCorrectCommandAndParsesSuccess` | Wire string `"power on\n"`, parses `success=true` |
| `BrakeReleaseSendsCorrectCommand` | Wire string `"brake release\n"` |
| `FailedResponseReturnsFalse` | Server reply containing "Failed" → `success=false` |
| `ErrorResponseReturnsFalse` | Server reply containing "error" → `success=false` |
| `ConnectionRefusedReturnsFalse` | No server running → graceful failure |
| `MultipleCommandsOnSameConnection` | Two sequential commands reuse the same socket |
| `PlayPauseStopSendCorrectCommands` | Parameterised check of all three commands |

### `test_rtde_client` — 7 tests
| Test | What it verifies |
|------|-----------------|
| `ParseDataPacketExtractsPositions` | 6 BE doubles decoded to correct positions |
| `ParseDataPacketExtractsVelocities` | 6 BE doubles decoded to correct velocities |
| `ParseDataPacketHomePosition` | Real-world home pose `[0, -1.5707, 1.5707, 0, 0, 0]` round-trips |
| `ParseDataPacketTooShortReturnsZeros` | Truncated packet → zero-initialised state, no crash |
| `SetupOutputsPayloadHasNoRecipeId` | First 8 bytes are frequency (125.0 Hz BE), not recipe_id |
| `SetupOutputsPayloadSize` | Payload is exactly `8 + len(vars)` — no extra byte |
| `HandshakeSucceedsWithCompliantServer` | Full mock server: VERSION → SETUP_OUTPUTS → START → DATA; state correctly latched |

**Completed: ~1 hour**

---

## Part 5

### What we built
Polish pass, demo script, and stress-test documentation.

### Polish

| Item | Change |
|------|--------|
| **Debug noise** | Stripped all `fprintf(stderr, ...)` and `hex_dump()` calls from `RtdeClient`. Replaced with `RCLCPP_INFO/WARN` for key lifecycle events (connected, lost, retrying). |
| **README** | Written: what it is, `docker compose up` quick-start, all service/action/topic interfaces, architecture in 5 bullets, production path. |
| **Docker healthcheck** | Added `healthcheck` to `ur12_driver` service — `ros2 node list` every 10 s, healthy after ~30 s of `start_period`. |

No new compiler warnings introduced; removed the previously-problematic `any_found` unused variable (Day 4).

### Demo Script

`ros2_ws/demo.sh` drives the robot end-to-end:
1. `power_on` → `brake_release` → 5 s wait
2. Print current joint positions from `/joint_states`
3. `move_home` action with live feedback until SUCCEEDED
4. Print joint positions at home
5. `move_first_joint +1.0` with feedback
6. Print joint positions (j0 ≈ 1.0 rad above home)
7. `move_first_joint -1.0` to return
8. `power_off`

Run with: `docker compose exec ur12_driver bash /ros2_ws/demo.sh`

Each step prints an annotated header so the audience can follow without ROS2 knowledge.

### Stress Tests

Run with `docker compose exec ur12_driver bash /ros2_ws/stress_test.sh` against URSim. All 4 scenarios pass.

| Scenario | Result |
|----------|--------|
| Test 1 — 2 rapid goals (bisect) | PASS — replace semantics confirmed |
| Test 2 — 6 rapid goals          | PASS — node survives, last script wins |
| Test 3 — `successive_goals=true` queue | PASS — both targets visited sequentially |
| Test 4 — action cancel          | PASS — terminal status CANCELED |
| Test 5 — 15 s stability @125 Hz | PASS — measured 124.795 Hz |
| Test 6 — lifecycle deactivate→activate | PASS — `/joint_states` resumes |

#### Key empirical finding: URSim secondary interface uses **replace** semantics

Hypothesised three possibilities — replace, queue, or drop — and built Test 1 to bisect. Two goals 1.25 s apart (`+1.5` then `-1.5` on j0); robot drove straight to `-1.5` and goal 1's target was never visited. Test 2 corroborates: of six near-simultaneous goals, only the last-sent target is actually reached (driver logs show `move_joints SUCCESS` only for that UUID).

This contradicts a common misconception that the secondary interface queues scripts. **Every new URScript preempts the running one.** Consequence for our action server: with no driver-side serialization, concurrent goals stomp each other and the polling loop can falsely report success if the robot transiently passes within `kThreshold` of a stale target on its way somewhere else.

#### Optional goal serialization

Added `successive_goals` ROS parameter (default `false`). When set to `true`, `execute_move_*` acquires a single `motion_mutex_` so concurrent goals run sequentially. Enables clean queue semantics on top of Arch 2 without rewriting to Arch 4 (reverse interface). Toggle via:

```
ros2 run ur12_driver ur_driver_node --ros-args -p successive_goals:=true
```

#### Tooling

- `ur12_driver/send_burst` (C++) — single rclcpp node, sends N MoveJoints goals at fixed gaps. Replaces a shell `for ... ros2 action send_goal &` loop where per-call action discovery (~hundreds of ms) eats the requested gap.
- Driver logs now tag each `move_joints` with the first 4 bytes of the goal UUID, log byte counts on every URScript send, and log actual-vs-target on success.

**Completed: ~2 hours**

---
