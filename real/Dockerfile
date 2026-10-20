# The car's driver image with the packages in workspaces/src rebuilt in place.
#
# The base image already has the apt dependencies (ros2_control, laser_filters,
# foxglove_bridge, ...) and a built /workspaces whose entrypoint sources
# /workspaces/install. Only the sources kept in this folder are replaced.
ARG BASE_IMAGE=registry.screamtrumpet.csie.ncku.edu.tw/kros_images/kros-wildbot-driver-image:latest
FROM ${BASE_IMAGE}

SHELL ["/bin/bash", "-c"]
WORKDIR /workspaces

# colcon package names; oradar_lidar is the package in src/oradar_ros.
ARG PACKAGES="oradar_lidar wildbot_imu wildbot_calibration kros_car_description kros_car"

# car_loc_lidar (../src, run through scripts/sim2real2sim.sh) needs scipy for
# its distance field. Kept above the COPY so source changes do not redo it.
RUN apt-get update && \
    apt-get install -y --no-install-recommends python3-scipy && \
    rm -rf /var/lib/apt/lists/*

COPY workspaces/src /tmp/src

# Each source directory is replaced as a whole, and the old build output is
# dropped, so a file removed here does not linger in the image.
RUN for dir in /tmp/src/*/; do rm -rf "src/$(basename "${dir}")"; done && \
    cp -r /tmp/src/. src/ && \
    rm -rf /tmp/src && \
    for pkg in ${PACKAGES}; do rm -rf "build/${pkg}" "install/${pkg}"; done && \
    source /opt/ros/${ROS_DISTRO}/setup.bash && \
    colcon build --symlink-install \
        --packages-select ${PACKAGES} \
        --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
