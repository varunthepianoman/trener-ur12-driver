#!/bin/bash
# UR12 End-to-End Demo
# Run with: docker compose exec ur12_driver bash /ros2_ws/demo.sh
set -e

source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash

header() { echo; echo "════════════════════════════════════════"; echo "  $1"; echo "════════════════════════════════════════"; }

trigger() {
  ros2 service call "$1" std_srvs/srv/Trigger 2>&1 | grep -E "success|message" || true
}

# Poll dashboard robotmode until it contains the expected string (timeout 30s)
wait_for_mode() {
  local expected="$1"
  local elapsed=0
  echo -n "  Waiting for robot mode '$expected'..."
  while true; do
    local mode
    mode=$(ros2 service call /ur/get_robot_mode std_srvs/srv/Trigger 2>/dev/null \
           | grep "message" | head -1 || true)
    if echo "$mode" | grep -qi "$expected"; then
      echo " OK"
      return 0
    fi
    sleep 2
    elapsed=$((elapsed + 2))
    if [ "$elapsed" -ge 30 ]; then
      echo " TIMEOUT waiting for $expected"
      return 1
    fi
    echo -n "."
  done
}

run_action() {
  local server="$1" type="$2" goal="$3"
  local output
  output=$(ros2 action send_goal "$server" "$type" "$goal" --feedback 2>/dev/null)
  echo "$output" | grep "distance_to_goal" | tail -5
  if echo "$output" | grep -q "success: true"; then
    echo "    success: true"
  else
    echo "    success: false — action did not succeed"
    exit 1
  fi
}

show_joints() {
  local positions
  # Strip only the YAML list marker "- " at the start of each line, preserving negative signs
  positions=$(ros2 topic echo --once /joint_states 2>/dev/null \
              | grep -A7 'position:' | grep -E '^\s*-\s+-?[0-9]' | sed 's/^\s*-\s//')
  local i=0
  while IFS= read -r val; do
    echo "  joint[$i] = $val rad"
    ((i++)) || true
  done <<< "$positions"
}

wait_for_rtde() {
  echo -n "  Waiting for RTDE joint state stream..."
  for _ in $(seq 1 15); do
    local pos
    pos=$(ros2 topic echo --once /joint_states 2>/dev/null \
          | grep -A7 'position:' | grep -E '^\s*-\s+-?[0-9]' | head -1)
    if [ -n "$pos" ]; then echo " OK"; return 0; fi
    sleep 1
    echo -n "."
  done
  echo " TIMEOUT"
}

# ─────────────────────────────────────────────────────────────────────────────

header "Step 1 — Power On"
trigger /ur/power_on
wait_for_mode "IDLE"

header "Step 2 — Brake Release"
trigger /ur/brake_release
wait_for_mode "RUNNING"

header "Step 3 — Current Joint Positions"
wait_for_rtde
show_joints

header "Step 4 — Move Away from Home First (to verify move_home actually runs)"
run_action /ur/move_joints ur12_driver_msgs/action/MoveJoints \
  '{joint_positions: [0.3, -1.5707, 1.5707, 0.0, 0.0, 0.0]}'

header "Step 5 — Move to Home [0, -π/2, π/2, 0, 0, 0]"
run_action /ur/move_home ur12_driver_msgs/action/MoveHome '{}'

header "Step 6 — Joint Positions at Home"
show_joints

header "Step 7 — Move All Joints to [0.5, -1.0, 1.0, -0.5, 0.5, 0.0]"
run_action /ur/move_joints ur12_driver_msgs/action/MoveJoints \
  '{joint_positions: [0.5, -1.0, 1.0, -0.5, 0.5, 0.0]}'

header "Step 8 — Joint Positions After Move"
show_joints

header "Step 9 — Return to Home"
run_action /ur/move_home ur12_driver_msgs/action/MoveHome '{}'

header "Step 10 — Power Off"
trigger /ur/power_off

header "Demo Complete"
echo "All steps finished successfully."
echo
