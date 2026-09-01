FROM registry.screamtrumpet.csie.ncku.edu.tw/ocarpan/pros_base_image
ENV ROS_DISTRO=humble

WORKDIR /workspaces

RUN apt-get update && \
    apt-get install -y ros-$ROS_DISTRO-vision-opencv \
                       build-essential \
                       cmake \
                       axel && \
    apt install -y ros-$ROS_DISTRO-moveit && \
    apt install -y ros-$ROS_DISTRO-image-transport-plugins && \
    apt install -y ros-$ROS_DISTRO-rqt-image-view && \
    apt install -y ros-$ROS_DISTRO-robot-localization && \
    apt install -y ros-$ROS_DISTRO-slam-toolbox && \
    apt install -y ros-$ROS_DISTRO-pointcloud-to-laserscan && \
    rm -rf /var/lib/apt/lists/* && apt-get clean

# rf2o_laser_odometry has no apt binary for ROS2 — build from source
RUN mkdir -p /workspaces/src && \
    git clone -b ros2 https://github.com/MAPIRlab/rf2o_laser_odometry.git /workspaces/src/rf2o_laser_odometry
   
# Step 2: Upgrade pip and install core build tools
RUN pip install --upgrade pip "setuptools<70.0.0" wheel Cython
RUN pip install --no-cache-dir numpy
# Add the --no-build-isolation flag to the pynvjpeg install
# RUN pip install --no-build-isolation pynvjpeg
# Step 3: Install heavy dependencies (Cached separately to save time)
RUN pip install --no-cache-dir torch torchvision
RUN pip install --no-cache-dir ultralytics opencv-contrib-python==4.6.0.66 norfair prometheus_client pytest
RUN pip install --no-cache-dir ncnn
RUN pip install --no-cache-dir onnx
RUN pip install --no-cache-dir onnxruntime-gpu[cuda,cudnn]
RUN pip install --no-cache-dir PyQt5


# Step 4: Isolate the problematic package
# If this still fails, your base image is missing CUDA development headers (nvcc, libnvjpeg-dev)



# Step 5: Setup ROS and Colcon
RUN pip install --force-reinstall "setuptools<70.0.0" && \
    colcon mixin update && \
    colcon metadata update && \
    rosdep install -q -y -r --from-paths src --ignore-src && \
    source /opt/ros/humble/setup.bash && colcon build --mixin release && \
    source ./install/setup.bash && \
    rm -rf /var/lib/apt/lists/* && apt-get clean
# Step 6: Build the ROS workspace
# (Assuming your source code is already copied into /workspaces/src somewhere before this)
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --mixin release"

ENTRYPOINT [ "/ros_entrypoint.bash" ]
CMD ["/bin/bash", "-l"]