FROM dustynv/ros:noetic-pytorch-l4t-r35.3.1

LABEL maintainer="srinath.bhamidipati@research.iiit.ac.in"

ENV DEBIAN_FRONTEND=noninteractive \
    TZ=Asia/Kolkata \
    LANG=en_US.UTF-8 \
    LANGUAGE=en_US:en \
    LC_ALL=en_US.UTF-8

# --- Create non-root user ---
RUN useradd -m -s /bin/bash container_user && \
    echo "container_user ALL=(ALL) NOPASSWD: ALL" >> /etc/sudoers

# --- Fix expired ROS GPG key ---
RUN apt-key del F42ED6FBAB17C654 || true && \
    apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-keys F42ED6FBAB17C654

# --- Install system dependencies for librealsense ---
RUN apt-get update && apt-get install -y --no-install-recommends \
    libxrandr-dev \
    libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev \
    libssl-dev libusb-1.0-0-dev libudev-dev pkg-config \
    libgtk-3-dev libgl1-mesa-glx \
    cmake git wget curl build-essential && \
    rm -rf /var/lib/apt/lists/*

# --- Clone and build librealsense (RealSense SDK v2.56.2) ---
USER container_user
WORKDIR /home/container_user

RUN git config --global http.postBuffer 524288000 && \
    git clone --depth 1 --branch v2.56.2 https://github.com/IntelRealSense/librealsense.git && \
    cd librealsense && mkdir build && cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_WITH_CUDA=ON && \
    make -j2

# --- Install librealsense system-wide ---
USER root
RUN cd /home/container_user/librealsense/build && make install && \
    cp /home/container_user/librealsense/config/99-realsense-libusb.rules /etc/udev/rules.d/ && \
    ldconfig

# NOTE: udevadm control --reload-rules && udevadm trigger should be run at runtime instead

# --- Install RealSense ROS wrapper ---
RUN apt-get update && \
    apt-get install -y --no-install-recommends ros-noetic-realsense2-camera && \
    rm -rf /var/lib/apt/lists/*

# --- Install additional ROS packages from rosPkgs.list ---
COPY rosPkgs.list /tmp/rosPkgs.list
RUN apt-get update && \
    xargs -a /tmp/rosPkgs.list apt-get install -y --no-install-recommends && \
    rm -rf /var/lib/apt/lists/*

# --- Install Python dependencies for YOLO (Ultralytics) ---
COPY yolo-requirements.txt /tmp/yolo-requirements.txt
RUN pip3 install --upgrade pip && \
    pip3 install --no-cache-dir -r /tmp/yolo-requirements.txt

# --- ROS setup for container_user ---
RUN echo "source /opt/ros/noetic/setup.bash" >> /home/container_user/.bashrc

# --- Final working directory and user ---
USER container_user
WORKDIR /home/container_user
