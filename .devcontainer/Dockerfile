FROM nvcr.io/nvidia/l4t-jetpack:r35.3.1

LABEL maintainer="srinath.bhamidipati@research.iiit.ac.in"

ENV DEBIAN_FRONTEND=noninteractive \
    TZ=Asia/Kolkata \
    LANG=en_US.UTF-8 \
    LANGUAGE=en_US:en \
    LC_ALL=en_US.UTF-8

# --- Create non-root user ---
RUN useradd -m -s /bin/bash container_user && \
    echo "container_user ALL=(ALL) NOPASSWD: ALL" >> /etc/sudoers

# --- Add ROS Noetic APT source and key ---
RUN apt-get update && apt-get install -y curl gnupg2 lsb-release && \
    echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros-latest.list && \
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -

# --- Fix expired NVIDIA ROS GPG key ---
RUN apt-key del F42ED6FBAB17C654 || true && \
    apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-keys F42ED6FBAB17C654

# --- Install system, ROS, and build dependencies ---
RUN apt-get update && apt-get install -y --no-install-recommends \
    libxrandr-dev libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev \
    libssl-dev libusb-1.0-0-dev libudev-dev pkg-config \
    libgtk-3-dev libgl1-mesa-glx libpsl-dev \
    libeigen3-dev \
    python3-pip python3-catkin-tools python3-rospkg python3-rosdep \
    cmake git wget curl build-essential \
    ros-noetic-catkin \
    ros-noetic-roslaunch \
    ros-noetic-rospy \
    ros-noetic-message-generation \
    ros-noetic-std-msgs \
    ros-noetic-sensor-msgs \
    ros-noetic-nav-msgs \
    ros-noetic-image-transport \
    ros-noetic-nodelet \
    ros-noetic-nodelet-core \
    ros-noetic-pluginlib \
    ros-noetic-class-loader \
    ros-noetic-cv-bridge \
    ros-noetic-image-proc \
    ros-noetic-camera-info-manager \
    ros-noetic-tf \
    ros-noetic-tf2 \
    ros-noetic-tf2-ros \
    ros-noetic-diagnostic-updater \
    ros-noetic-ddynamic-reconfigure \
    ros-noetic-visualization-msgs \
    && rm -rf /var/lib/apt/lists/*




# --- Install PyTorch with CUDA (Jetson wheel) ---
RUN python3 -m pip install --upgrade pip && \
    python3 -m pip install --no-cache-dir numpy==1.24.4 && \
    wget https://developer.download.nvidia.com/compute/redist/jp/v511/pytorch/torch-2.0.0+nv23.05-cp38-cp38-linux_aarch64.whl && \
    python3 -m pip install torch-2.0.0+nv23.05-cp38-cp38-linux_aarch64.whl && \
    rm torch-2.0.0+nv23.05-cp38-cp38-linux_aarch64.whl

# --- Initialize rosdep (safe, even if not used directly) ---
RUN rosdep init || true && rosdep update

# --- Build and install librealsense ---
USER container_user
WORKDIR /home/container_user

RUN git config --global http.postBuffer 524288000 && \
    git clone --depth 1 --branch v2.56.2 https://github.com/IntelRealSense/librealsense.git && \
    cd librealsense && mkdir build && cd build && \
    cmake .. \
        -DCMAKE_BUILD_TYPE=Release \
        -DBUILD_WITH_CUDA=ON \
        -DBUILD_TOOLS=OFF \
        -DBUILD_EXAMPLES=OFF \
        -DBUILD_GRAPHICAL_EXAMPLES=OFF && \
    make -j2

USER root
RUN cd /home/container_user/librealsense/build && make install && \
    cp /home/container_user/librealsense/config/99-realsense-libusb.rules /etc/udev/rules.d/ && \
    ldconfig

# --- Clone and build RealSense ROS wrapper ---
USER container_user
WORKDIR /home/container_user
RUN mkdir -p catkin_ws/src && cd catkin_ws/src && \
    git clone -b ros1-legacy https://github.com/IntelRealSense/realsense-ros.git

WORKDIR /home/container_user/catkin_ws
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make -DCATKIN_ENABLE_TESTING=OFF -DCMAKE_BUILD_TYPE=Release"

# --- Source ROS and workspace setup for user ---
USER root
RUN echo "source /opt/ros/noetic/setup.bash" >> /home/container_user/.bashrc 

# FROM ghcr.io/srindot/noetic_realsense:dev1

# LABEL maintainer="srinath.bhamidipati@research.iiit.ac.in"

USER root

# Install dependencies for torchvision build (best for Jetson)
RUN apt-get update && apt-get install -y --no-install-recommends \
    libjpeg-dev zlib1g-dev libpng-dev libavcodec-dev libavformat-dev libswscale-dev \
    && rm -rf /var/lib/apt/lists/*

# Install Python packages required for YOLO
RUN python3 -m pip install --upgrade --no-cache-dir pip \
    && python3 -m pip install --no-cache-dir --upgrade numpy wheel setuptools \
    && python3 -m pip install --no-cache-dir opencv-python>=4.5.1 \
    && python3 -m pip install --no-cache-dir ultralytics

RUN python3 -m pip install --upgrade pip setuptools importlib-metadata wheel


# Install torchvision from source for ARM/Jetson compatibility (match torch==2.0.0)
RUN git clone --branch v0.15.2 https://github.com/pytorch/vision.git \
    && cd vision \
    && python3 setup.py install \
    && cd .. && rm -rf vision

# --- Final user and working directory ---
USER container_user
WORKDIR /home/container_user
