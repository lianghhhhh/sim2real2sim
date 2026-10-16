#!/bin/bash
# Everything the car needs for sim2real data collection: wheels + arm, lidar,
# IMU and the Foxglove bridge. No EKF: sim2real2sim does its own localization
# and uses /odom for ground truth.
source "$(dirname "${BASH_SOURCE[0]}")/utils.sh"
main \
    "$COMPOSE_DIR/docker-compose_kros_car.yml" \
    "$COMPOSE_DIR/docker-compose_oradarlidar.yml" \
    "$COMPOSE_DIR/docker-compose_imu.yml" \
    "$COMPOSE_DIR/docker-compose_foxglove_bridge.yml"
