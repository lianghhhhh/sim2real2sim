#!/bin/bash
# A shell for running sim2real2sim's ROS packages (../src) against the real car.
#
#   ./scripts/sim2real2sim.sh      # first call: starts the container and builds
#   ./scripts/sim2real2sim.sh      # later calls: another shell in the same container
#
# It uses the car's own image (kros-car-new, ROS 2 Jazzy) with host networking
# and the car's ROS_DOMAIN_ID, so it sees the car's topics. ../src is mounted at
# /s2r/src, so a map saved under /s2r/src/car_loc_lidar/maps stays in the repo.
# The build output lives in the container and is gone when the first shell exits.
#
# PACKAGES selects what to build (default: car_teleop car_loc_lidar).
set -e

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
S2R_ROOT="$(cd "$REPO_ROOT/.." && pwd)"
NAME=s2r_real
PACKAGES="${PACKAGES:-car_teleop car_loc_lidar}"

if docker ps --format '{{.Names}}' | grep -qx "$NAME"; then
    exec docker exec -it "$NAME" bash --rcfile /s2r/.bashrc
fi

mkdir -p "$S2R_ROOT/car_run_data"

# The rc file builds once (in the first shell) and sources the result.
read -r -d '' RC <<EOF || true
source /opt/ros/\$ROS_DISTRO/setup.bash
cd /s2r
if [ ! -f /s2r/install/setup.bash ]; then
    colcon build --symlink-install --packages-select $PACKAGES
fi
source /s2r/install/setup.bash
alias r='cd /s2r && colcon build --symlink-install --packages-select $PACKAGES && source install/setup.bash'
PS1='(s2r_real) \w\$ '
EOF

exec docker run -it --rm \
    --name "$NAME" \
    --network host \
    --ipc host \
    --env-file "$REPO_ROOT/docker/compose/.env" \
    -v "$S2R_ROOT/src:/s2r/src" \
    -v "$S2R_ROOT/scripts:/s2r/scripts" \
    -v "$S2R_ROOT/car_run_data:/s2r/car_run_data" \
    -w /s2r \
    --entrypoint bash \
    kros-car-new:latest \
    -c "printf '%s\n' \"\$1\" > /s2r/.bashrc && exec bash --rcfile /s2r/.bashrc" _ "$RC"
