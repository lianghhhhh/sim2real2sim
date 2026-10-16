#!/bin/bash
# Chooses which controller drives the wheels. Only one can be active.
#
#   ./scripts/wheel_controller.sh joint_command   # wheel speeds [rad/s] on /joint_command
#                                                 # (data collection, MPC; same as the simulation)
#   ./scripts/wheel_controller.sh cmd_vel         # twist on /base_controller/cmd_vel
#                                                 # (teleop, mapping, calibration; the default)
#
# kros_car must already be running (start_all.sh or kros_car.sh).

set -e

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ENV_FILE="$REPO_ROOT/docker/compose/.env"
IMAGE="kros-car-new:latest"

case "${1:-}" in
    joint_command) ACTIVATE=wheel_velocity_controller; DEACTIVATE=base_controller ;;
    cmd_vel)       ACTIVATE=base_controller;           DEACTIVATE=wheel_velocity_controller ;;
    *) echo "Usage: $0 joint_command|cmd_vel" >&2; exit 1 ;;
esac

# The image has no `ros2 control` CLI, so the controller manager service is
# called directly. strictness 2 = STRICT: nothing changes if either step fails.
exec docker run --rm \
    --env-file "$ENV_FILE" \
    --network host \
    --ipc host \
    "$IMAGE" \
    ros2 service call /controller_manager/switch_controller \
        controller_manager_msgs/srv/SwitchController \
        "{activate_controllers: [$ACTIVATE], deactivate_controllers: [$DEACTIVATE], strictness: 2}"
