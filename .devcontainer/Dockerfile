FROM nvcr.io/nvidia/l4t-pytorch:r35.4.1-pth2.2-py3

LABEL maintainer="srinath.bhamidipati@research.iiit.ac.in"

ENV TZ=Asia/Kolkata
ENV LANG=en_US.UTF-8 \
    LANGUAGE=en_US:en \
    LC_ALL=en_US.UTF-8 \
    DEBIAN_FRONTEND=noninteractive

# --- System setup ---
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        tzdata locales curl ca-certificates lsb-release software-properties-common \
        build-essential git wget gnupg sudo cmake \
        libssl-dev libusb-1.0-0-dev pkg-config \
        libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev libgtk-3-dev \
        v4l-utils udev bc libjpeg-dev libpng-dev \
        python3 python3-pip python3-dev python3-yaml python3-zmq && \
    ln -fs /usr/share/zoneinfo/$TZ /etc/localtime && \
    dpkg-reconfigure --frontend noninteractive tzdata && \
    locale-gen en_US.UTF-8 && update-locale LANG=en_US.UTF-8 && \
    rm -rf /var/lib/apt/lists/*

# --- ROS Noetic ---
RUN echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list && \
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
    apt-get update && \
    apt-get install -y ros-noetic-ros-base && \
    rm -rf /var/lib/apt/lists/*

# --- Create user ---
RUN useradd -m -s /bin/bash container_user && \
    echo "container_user ALL=(ALL) NOPASSWD: ALL" >> /etc/sudoers

USER container_user
WORKDIR /home/container_user

# --- Clone and Build librealsense ---
RUN git config --global http.postBuffer 524288000 && \
    git clone --branch v2.56.2 https://github.com/IntelRealSense/librealsense.git

WORKDIR /home/container_user/librealsense
RUN mkdir -p build && cd build && cmake ../ && make clean && make -j$(nproc)

USER root
RUN cd /home/container_user/librealsense/build && make install

# --- Install ROS packages ---
COPY rosPkgs.list /tmp/rosPkgs.list
RUN apt-get update && \
    apt-get install -y --no-install-recommends $(cat /tmp/rosPkgs.list) && \
    rm -rf /var/lib/apt/lists/*

# --- Install Python packages into system Python ---
COPY yolo-requirements.txt /tmp/yolo-requirements.txt

RUN pip3 install --upgrade pip && \
    pip3 install -r /tmp/yolo-requirements.txt
