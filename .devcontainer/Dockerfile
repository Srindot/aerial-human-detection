FROM nvcr.io/nvidia/l4t-pytorch:r35.3.1-pth2.0-py3

LABEL maintainer="srinath.bhamidipati@research.iiit.ac.in"

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Asia/Kolkata
ENV LANG=en_US.UTF-8 \
    LANGUAGE=en_US:en \
    LC_ALL=en_US.UTF-8

# --- System dependencies ---
RUN apt-get update && apt-get install -y --no-install-recommends \
    tzdata locales curl sudo gnupg2 lsb-release software-properties-common \
    build-essential cmake git wget \
    libssl-dev libusb-1.0-0-dev pkg-config \
    libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev libgtk-3-dev \
    v4l-utils udev bc \
    libjpeg-dev libpng-dev \
    python3-dev python3-pip python3-yaml python3-zmq \
    && ln -fs /usr/share/zoneinfo/$TZ /etc/localtime \
    && dpkg-reconfigure --frontend noninteractive tzdata \
    && locale-gen en_US.UTF-8 && update-locale LANG=en_US.UTF-8 \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# --- ROS Noetic base ---
RUN echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros-latest.list && \
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
    apt-get update && apt-get install -y ros-noetic-ros-base

# --- Create user ---
RUN useradd -m -s /bin/bash container_user && \
    echo "container_user ALL=(ALL) NOPASSWD: ALL" >> /etc/sudoers

USER container_user
WORKDIR /home/container_user

# --- Librealsense ---
RUN git config --global http.postBuffer 524288000 && \
    git clone --branch v2.56.2 https://github.com/IntelRealSense/librealsense.git && \
    cd librealsense && mkdir build && cd build && \
    cmake .. && make -j$(nproc)

USER root
RUN cd /home/container_user/librealsense/build && make install && \
    cp /home/container_user/librealsense/config/99-realsense-libusb.rules /etc/udev/rules.d/ && \
    udevadm control --reload-rules && udevadm trigger

# --- ROS Packages ---
COPY rosPkgs.list /tmp/rosPkgs.list
RUN apt-get update && \
    xargs -a /tmp/rosPkgs.list apt-get install -y --no-install-recommends && \
    rm -rf /var/lib/apt/lists/*

# --- Python dependencies: Ultralytics YOLO + others ---
COPY yolo-requirements.txt /tmp/yolo-requirements.txt
RUN pip3 install --upgrade pip && \
    pip3 install --no-cache-dir -r /tmp/yolo-requirements.txt

# --- ROS setup in bashrc ---
RUN echo "source /opt/ros/noetic/setup.bash" >> /home/container_user/.bashrc

# --- Final Setup ---
WORKDIR /home/container_user
USER container_user
