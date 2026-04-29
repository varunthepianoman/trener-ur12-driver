#!/bin/bash
# UR12 Stress Test Suite
# Run with: docker compose exec ur12_driver bash /ros2_ws/stress_test.sh
set -e

source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash

PASS=0
FAIL=0

header() { echo; echo "════════════════════════════════════════"; echo "  $1"; echo "════════════════════════════════════════"; }

pass() { echo "  [PASS] $1"; PASS=$((PASS + 1)); }
fail() { echo "  [FAIL] $1"; FAIL=$((FAIL + 1)); }

trigger() {
  ros2 service call "$1" std_srvs/srv/Trigger 2>&1 | grep -E "success|message" || true
}

wait_for_mode() {
  local expected="$1"
  local elapsed=0
  echo -n "  Waiting for robot mode '$expected'..."
  while true; do
    # ros2 service call prints the response as a Python-repr one-liner:
    #   response:
    #   ur12_driver_msgs.srv.GetRobotMode_Response(success=True, mode='IDLE', raw='...')
    # We extract the value of the mode= field.
    local mode
    mode=$(ros2 service call /ur/get_robot_mode \
              ur12_driver_msgs/srv/GetRobotMode 2>/dev/null \
           | grep -oE "mode='[^']*'" | head -1 | sed -E "s/mode='(.*)'/\1/" || true)
    if [ -n "$mode" ] && echo "$mode" | grep -qi "$expected"; then echo " OK ($mode)"; return 0; fi
    sleep 2; elapsed=$((elapsed + 2))
    if [ "$elapsed" -ge 30 ]; then echo " TIMEOUT (last mode: ${mode:-<unparseable>})"; return 1; fi
    echo -n "."
  done
}

wait_for_rtde() {
  echo -n "  Waiting for RTDE stream..."
  for _ in $(seq 1 15); do
    local pos
    pos=$(ros2 topic echo --once /joint_states 2>/dev/null \
          | grep -A7 'position:' | grep -E '^\s*-\s+-?[0-9]' | head -1)
    if [ -n "$pos" ]; then echo " OK"; return 0; fi
    sleep 1; echo -n "."
  done
  echo " TIMEOUT"; return 1
}

get_joint0() {
  ros2 topic echo --once /joint_states 2>/dev/null \
    | grep -A7 'position:' | grep -E '^\s*-\s+-?[0-9]' | head -1 | sed 's/^\s*-\s//'
}

# ─────────────────────────────────────────────────────────────────────────────
# Setup: power on
# ─────────────────────────────────────────────────────────────────────────────
header "Setup — Power On & Brake Release"
trigger /ur/power_on
wait_for_mode "IDLE"
trigger /ur/brake_release
wait_for_mode "RUNNING"
wait_for_rtde

# Move to known home first
echo "  Homing robot..."
ros2 action send_goal /ur/move_home ur12_driver_msgs/action/MoveHome '{}' \
  2>/dev/null | grep -E "success" | head -1

# ─────────────────────────────────────────────────────────────────────────────
# Test 1: Two rapid goals (bisect)
# ─────────────────────────────────────────────────────────────────────────────
header "Test 1 — Two Rapid Goals (bisect URSim secondary-interface behavior)"
echo "  Sends exactly 2 goals 1.25s apart via send_burst (single rclcpp node,"
echo "  so the gap is honored — shell 'ros2 action send_goal &' eats the gap"
echo "  on per-call action discovery)."
echo "  Expectation under Architecture 2 (replace semantics): goal 2 wins → j0 ≈ -1.5"

J0_BEFORE=$(get_joint0)
echo "  joint[0] before: $J0_BEFORE"

ros2 run ur12_driver send_burst 1.25 6 \
  " 1.5,-1.5707,1.5707,0.0,0.0,0.0" \
  "-1.5,-1.5707,1.5707,0.0,0.0,0.0"

J0_AFTER=$(get_joint0)
echo "  joint[0] after : $J0_AFTER"
echo "  → +1.5 = goal 1 won (queue/drop)   -1.5 = goal 2 won (replace)   ~0 = neither"

if awk "BEGIN{exit !($J0_AFTER <= -1.4)}"; then
  pass "Replace semantics confirmed (j0 final=$J0_AFTER, expected ≈ -1.5)"
else
  fail "Robot did not settle at goal 2 (j0 final=$J0_AFTER)"
fi

# Re-home before continuing
ros2 action send_goal /ur/move_home ur12_driver_msgs/action/MoveHome '{}' \
  2>/dev/null | grep -E "success" | head -1 || true

# ─────────────────────────────────────────────────────────────────────────────
# Test 2: Rapid successive goals
# ─────────────────────────────────────────────────────────────────────────────
header "Test 2 — Rapid Successive Goals (6 move_joints calls, 1.25s apart, large moves)"
echo "  Sending 6 goals 1.25s apart via send_burst with large joint swings —"
echo "  robot should be mid-motion (past the movej accel ramp) when each arrives."
echo "  Last script wins under Arch 2."

T1_CRASHED=false
ros2 run ur12_driver send_burst 1.25 6 \
  " 1.5,-1.0,1.0, 0.5, 0.5,0.0" \
  "-1.5,-2.0,2.0,-0.5,-0.5,0.0" \
  " 1.2,-0.8,0.8, 0.4, 0.4,0.0" \
  "-1.2,-1.8,1.8,-0.4,-0.4,0.0" \
  " 0.8,-1.2,1.2, 0.3, 0.3,0.0" \
  " 0.0,-1.5707,1.5707,0.0,0.0,0.0"

# Verify node still alive
if ros2 node list 2>/dev/null | grep -q "ur_driver_node"; then
  pass "Node still alive after 10 rapid goals"
else
  fail "Node crashed or unresponsive after rapid goals"
  T1_CRASHED=true
fi

if [ "$T1_CRASHED" = false ]; then
  # Re-home before next test
  ros2 action send_goal /ur/move_home ur12_driver_msgs/action/MoveHome '{}' \
    2>/dev/null | grep -E "success" | head -1 || true
fi

# ─────────────────────────────────────────────────────────────────────────────
# Test 3: successive_goals=true (queue semantics on top of Arch 2)
# ─────────────────────────────────────────────────────────────────────────────
header "Test 3 — successive_goals=true (queue concurrent goals via motion_mutex)"
echo "  Flips successive_goals on and fires the same 2 goals as Test 1."
echo "  Expectation: robot visits goal 1 (j0≈+1.5) FIRST, then goal 2 (j0≈-1.5)."
echo "  send_burst's joint_states summary shows j0 max ≥ 1.4 if both targets visited."

ros2 param set /ur_driver_node successive_goals true >/dev/null

T2_LOG=$(mktemp)
ros2 run ur12_driver send_burst 0.75 10 \
  " 1.5,-1.5707,1.5707,0.0,0.0,0.0" \
  "-1.5,-1.5707,1.5707,0.0,0.0,0.0" | tee "$T2_LOG"

J0_MAX=$(grep -E "^\s+j0:" "$T2_LOG" | grep -oE "max=[+-][0-9.]+" | sed 's/max=//')
J0_AFTER=$(get_joint0)
rm -f "$T2_LOG"

echo "  j0 max during run: $J0_MAX"
echo "  j0 final         : $J0_AFTER"

if [ -n "$J0_MAX" ] && awk "BEGIN{exit !($J0_MAX >= 1.4)}" \
   && awk "BEGIN{exit !($J0_AFTER <= -1.4)}"; then
  pass "successive_goals=true: robot visited goal 1 (j0 max=$J0_MAX) then goal 2 (j0=$J0_AFTER)"
else
  fail "successive_goals=true did not produce queue semantics (j0_max=$J0_MAX, j0_final=$J0_AFTER)"
fi

ros2 param set /ur_driver_node successive_goals false >/dev/null

# Re-home
ros2 action send_goal /ur/move_home ur12_driver_msgs/action/MoveHome '{}' \
  2>/dev/null | grep -E "success" | head -1 || true

# ─────────────────────────────────────────────────────────────────────────────
# Test 4: Action-level cancel
# ─────────────────────────────────────────────────────────────────────────────
header "Test 4 — Action Cancel (handle_move_joints_cancel + dashboard stop)"
echo "  Sends a long goal, waits 1.5s mid-motion, calls async_cancel_goal."
echo "  Expectation: terminal status CANCELED, robot stopped short of target."

T25_LOG=$(mktemp)
if ros2 run ur12_driver cancel_goal 1.5 "1.5,-1.5707,1.5707,0.0,0.0,0.0" | tee "$T25_LOG"; then
  if grep -q "terminal status: CANCELED" "$T25_LOG"; then
    pass "Goal terminated as CANCELED via action cancel"
  else
    fail "Cancel returned but status was not CANCELED"
  fi
else
  fail "cancel_goal binary returned non-zero"
fi
rm -f "$T25_LOG"

# Re-home
ros2 action send_goal /ur/move_home ur12_driver_msgs/action/MoveHome '{}' \
  2>/dev/null | grep -E "success" | head -1 || true

# ─────────────────────────────────────────────────────────────────────────────
# Test 5: Long-running stability at 125 Hz
# ─────────────────────────────────────────────────────────────────────────────
header "Test 5 — 15s Stability at 125 Hz"
echo "  Monitoring /joint_states for 15s..."

HZ_LOG=$(mktemp)
ros2 topic hz /joint_states --window 100 > "$HZ_LOG" 2>&1 &
HZ_PID=$!
sleep 17
kill $HZ_PID 2>/dev/null || true
wait $HZ_PID 2>/dev/null || true

AVG_HZ=$(grep "average rate" "$HZ_LOG" | tail -1 | grep -oE '[0-9]+\.[0-9]+' | head -1)
rm -f "$HZ_LOG"
echo "  Average rate: ${AVG_HZ:-unknown} Hz"

if [ -n "$AVG_HZ" ] && awk "BEGIN{exit !($AVG_HZ >= 100)}"; then
  pass "Sustained >= 100 Hz over 60s (got ${AVG_HZ} Hz)"
else
  fail "Rate below 100 Hz or unmeasurable (got ${AVG_HZ:-unknown} Hz)"
fi

# ─────────────────────────────────────────────────────────────────────────────
# Test 6: Lifecycle deactivate → activate
# ─────────────────────────────────────────────────────────────────────────────
header "Test 6 — Lifecycle Deactivate → Activate"
echo "  Deactivating node..."
ros2 lifecycle set /ur_driver_node deactivate 2>/dev/null || true
sleep 2

echo "  Activating node..."
ros2 lifecycle set /ur_driver_node activate 2>/dev/null || true
sleep 3

echo "  Checking /joint_states resumes..."
if wait_for_rtde; then
  pass "joint_states resumed after lifecycle deactivate→activate"
else
  fail "joint_states did not resume after lifecycle transition"
fi

# ─────────────────────────────────────────────────────────────────────────────
# Summary
# ─────────────────────────────────────────────────────────────────────────────
header "Stress Test Summary"
echo "  PASSED: $PASS"
echo "  FAILED: $FAIL"
echo

# Teardown
trigger /ur/power_off

if [ "$FAIL" -eq 0 ]; then
  echo "All stress tests passed."
  exit 0
else
  echo "Some tests failed — see output above."
  exit 1
fi
