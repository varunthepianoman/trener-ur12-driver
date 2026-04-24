#!/bin/bash
set -e
source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash

# Wait for URSim TCP servers to be ready before starting the driver
ROBOT_HOST="${ROBOT_HOST:-ursim}"
for PORT in 29999 30004; do
  echo "Waiting for URSim at ${ROBOT_HOST}:${PORT}..."
  until nc -z "${ROBOT_HOST}" "${PORT}" 2>/dev/null; do
    sleep 1
  done
done
echo "URSim is up — starting driver"

exec "$@"
