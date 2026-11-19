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

# ---- MPC toolchain for mlp_control_pkg / sensor_mpc_pkg / nav_mpc_pkg ----
# Same versions as the simulation image (repo root Dockerfile): acados commit,
# casadi 3.7.2, l4casadi v2.0.0, torch 2.13.0. Differences:
#   - torch is the CPU build: the MPC runs l4casadi with device='cpu', and the
#     car has no environment cameras, so nothing here needs CUDA.
#   - Ubuntu 24.04 marks the system Python as externally managed (PEP 668), so
#     pip needs PIP_BREAK_SYSTEM_PACKAGES. Packages go to /usr/local.
#   - numpy stays at the apt 1.26.4 (cv_bridge) and setuptools at the apt 68.x
#     (colcon --symlink-install); pip.constraints keeps pip from upgrading them.
#   - sklearn is not needed: the scalers are .npz.
# Nav2 (planner, smac, costmap, map_server) is already in the base image.
ARG THREADS=8
ARG ACADOS_COMMIT=374f57234eec1daf517a34c461f25f07faf9f6be
ARG TERA_VERSION=0.2.0
ARG TORCH_VERSION=2.13.0
ARG L4CASADI_BRANCH=v2.0.0-release
ENV PIP_BREAK_SYSTEM_PACKAGES=1 \
    PIP_NO_CACHE_DIR=1

RUN apt-get update && \
    apt-get install -y --no-install-recommends cmake libblas-dev liblapack-dev && \
    rm -rf /var/lib/apt/lists/* && \
    python3 -c "import numpy, setuptools; print(f'numpy=={numpy.__version__}\nsetuptools=={setuptools.__version__}')" \
        > /etc/pip.constraints && \
    cat /etc/pip.constraints
ENV PIP_CONSTRAINT=/etc/pip.constraints

RUN git clone https://github.com/acados/acados.git /acados && \
    cd /acados && git checkout ${ACADOS_COMMIT} && \
    git submodule update --init --recursive && \
    mkdir -p build && cd build && \
    cmake .. -DACADOS_WITH_OPENMP=ON && \
    make install -j${THREADS} && \
    mkdir -p /acados/bin && \
    curl -fsSL -o /acados/bin/t_renderer \
        https://github.com/acados/tera_renderer/releases/download/v${TERA_VERSION}/t_renderer-v${TERA_VERSION}-linux-amd64 && \
    chmod +x /acados/bin/t_renderer
ENV ACADOS_SOURCE_DIR=/acados
ENV LD_LIBRARY_PATH=/acados/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}

# torch 2.13 lists setuptools>=77 as a dependency, which the constraint forbids
# (torch itself imports fine with 68). So torch goes in with --no-deps and its
# other requirements are installed from its own metadata, minus setuptools.
RUN pip install --no-deps "torch==${TORCH_VERSION}+cpu" --index-url https://download.pytorch.org/whl/cpu && \
    python3 -c "from importlib.metadata import requires; from packaging.requirements import Requirement; \
rs = [Requirement(r) for r in requires('torch')]; \
print('\n'.join(str(r) for r in rs if r.name != 'setuptools' and (r.marker is None or r.marker.evaluate({'extra': ''}))))" \
        > /tmp/torch-reqs.txt && \
    cat /tmp/torch-reqs.txt && \
    pip install -r /tmp/torch-reqs.txt && rm /tmp/torch-reqs.txt && \
    pip install casadi==3.7.2 && \
    pip install "scikit-build>=0.17" "ninja>=1.11" && \
    git clone --depth 1 --branch ${L4CASADI_BRANCH} \
        https://github.com/Tim-Salzmann/l4casadi.git /tmp/l4casadi && \
    CMAKE_BUILD_PARALLEL_LEVEL=${THREADS} \
        pip install --no-deps --no-build-isolation /tmp/l4casadi && \
    pip install jinja2 && \
    pip install /acados/interfaces/acados_template && \
    rm -rf /tmp/l4casadi && \
    python3 -c "import torch, casadi, l4casadi, acados_template, numpy, setuptools; \
print('torch', torch.__version__, 'casadi', casadi.__version__, 'numpy', numpy.__version__, 'setuptools', setuptools.__version__)"

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
