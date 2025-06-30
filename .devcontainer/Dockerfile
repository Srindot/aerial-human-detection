FROM ubuntu:20.04

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
        build-essential python3-pip python3-dev python3-venv python3-setuptools \
        git wget gnupg sudo cmake libssl-dev libusb-1.0-0-dev pkg-config \
        libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev libgtk-3-dev \
        v4l-utils udev bc \
        libjpeg-dev libpng-dev && \
    ln -fs /usr/share/zoneinfo/$TZ /etc/localtime && \
    dpkg-reconfigure --frontend noninteractive tzdata && \
    locale-gen en_US.UTF-8 && update-locale LANG=en_US.UTF-8 && \
    rm -rf /var/lib/apt/lists/*

# --- CUDA env vars only (Jetson friendly) ---
ENV CUDA_VERSION=11.4
ENV PATH=/usr/local/cuda/bin:$PATH
ENV LD_LIBRARY_PATH=/usr/local/cuda/lib64

# --- CMake install ---
ARG CMAKE_VERSION=3.27.8
RUN curl -sSL https://github.com/Kitware/CMake/releases/download/v${CMAKE_VERSION}/cmake-${CMAKE_VERSION}-linux-aarch64.sh -o cmake.sh && \
    chmod +x cmake.sh && ./cmake.sh --skip-license --prefix=/usr/local && rm cmake.sh

# --- ROS Noetic installation ---
RUN echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list && \
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
    apt-get update && \
    apt-get install -y ros-noetic-ros-base && \
    rm -rf /var/lib/apt/lists/*

# --- Non-root user ---
RUN useradd -m -s /bin/bash container_user && \
    echo "container_user ALL=(ALL) NOPASSWD: ALL" >> /etc/sudoers && \
    chown -R container_user:container_user /home/container_user /opt/ros

# --- Build librealsense ---
USER container_user
WORKDIR /home/container_user
RUN git config --global http.postBuffer 524288000 && \
    git clone --branch v2.56.2 https://github.com/IntelRealSense/librealsense.git

WORKDIR /home/container_user/librealsense
RUN mkdir -p build && cd build && \
    cmake ../ && \
    make clean && make -j$(nproc)

# --- Install librealsense ---
USER root
RUN cd /home/container_user/librealsense/build && make install

# --- Install ROS dependencies from list ---
COPY --chown=container_user:container_user rosPkgs.list /tmp/rosPkgs.list
RUN apt-get update && \
    apt-get install -y --no-install-recommends $(cat /tmp/rosPkgs.list) && \
    rm -rf /var/lib/apt/lists/*

# --- Python dependencies ---
USER container_user
WORKDIR /home/container_user
RUN pip3 install --upgrade pip && \
    pip3 install \
        dill \
        numpy \
        opencv-python \
        torch \
        torchvision \
        ultralytics \
        rospkg \
        matplotlib \
        netifaces

# --- ROS sourcing ---
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

CMD ["/bin/bash"]
