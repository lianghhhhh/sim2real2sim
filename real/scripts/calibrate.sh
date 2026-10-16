#!/bin/bash
# Interactive calibration wizard. Runs the wildbot_calibration package in a
# one-shot `docker run` container on the host network, like the compose
# services, so it can talk to kros_car + wildbot_imu (which must already be
# running, e.g. via start_all.sh or kros_car.sh + imu.sh).
#
# Usage:
#   ./scripts/calibrate.sh                                # full wizard (run_all_calibrations)
#   ./scripts/calibrate.sh imu_axis_sanity                # single step
#
# Available executables (see wildbot_calibration/setup.py):
#   run_all_calibrations, imu_axis_sanity, imu_static_calibration,
#   wheel_linear_calibration, wheel_angular_calibration, imu_accel_two_face
#
# Results land in <repo>/.ros/calibration_results.yaml on the host (git-ignored).

set -e

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ENV_FILE="$REPO_ROOT/docker/compose/.env"
IMAGE="kros-car-new:latest"

EXECUTABLE="${1:-run_all_calibrations}"

mkdir -p "$REPO_ROOT/.ros"

exec docker run --rm -it \
    --env-file "$ENV_FILE" \
    --network host \
    --ipc host \
    -v "$REPO_ROOT/.ros:/root/.ros" \
    "$IMAGE" \
    ros2 run wildbot_calibration "$EXECUTABLE"
