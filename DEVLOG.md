# UR12 Driver — Dev Log

---

## End of Day 1

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

---
