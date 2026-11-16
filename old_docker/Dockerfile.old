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
    apt install -y ros-$ROS_DISTRO-foxglove-bridge && \
    rm -rf /var/lib/apt/lists/* && apt-get clean

# rf2o_laser_odometry has no apt binary for ROS2 — build from source
RUN mkdir -p /workspaces/src && \
    git clone -b ros2 https://github.com/MAPIRlab/rf2o_laser_odometry.git /workspaces/src/rf2o_laser_odometry
   
# ROS humble's cv_bridge (ros-humble-vision-opencv) and opencv-contrib-python 4.6
# are compiled against the NumPy 1.x C ABI. Any pip install below that drags in
# numpy 2.x breaks them at import time with "_ARRAY_API not found".
# onnx -> ml_dtypes>=0.5.4 is the one that does it: ml_dtypes 0.6 requires numpy>=2.
# An image-wide constraint keeps every pip install on the 1.x line (numpy 1.26.4).
RUN echo "numpy<2" > /etc/pip-constraint.txt
ENV PIP_CONSTRAINT=/etc/pip-constraint.txt

# Step 2: Upgrade pip and install core build tools
RUN pip install --upgrade pip "setuptools<70.0.0" wheel Cython
RUN pip install --no-cache-dir "numpy<2"
# Add the --no-build-isolation flag to the pynvjpeg install
# RUN pip install --no-build-isolation pynvjpeg
# Step 3: Install heavy dependencies (Cached separately to save time)
RUN pip install --no-cache-dir torch torchvision
RUN pip install --no-cache-dir ultralytics opencv-contrib-python==4.6.0.66 norfair prometheus_client pytest
RUN pip install --no-cache-dir ncnn
RUN pip install --no-cache-dir onnx
RUN pip install --no-cache-dir onnxruntime-gpu[cuda,cudnn]
RUN pip install --no-cache-dir PyQt5

# ultralytics and ncnn pull in opencv-python, which owns the same cv2/ directory
# as the pinned opencv-contrib-python — whichever lands last wins. Reinstall the
# contrib build alone so there is exactly one cv2.
RUN pip uninstall -y opencv-python opencv-contrib-python && \
    pip install --no-cache-dir opencv-contrib-python==4.6.0.66


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