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

USER container_user
WORKDIR /home/container_user

# --- Clone and build librealsense (RealSense SDK v2.56.2) ---
RUN git config --global http.postBuffer 524288000 && \
    git clone --depth 1 --branch v2.56.2 https://github.com/IntelRealSense/librealsense.git && \
    cd librealsense && mkdir build && cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release && \
    make -j$(nproc)

# --- Install librealsense system-wide ---
USER root
RUN cd /home/container_user/librealsense/build && make install && \
    cp /home/container_user/librealsense/config/99-realsense-libusb.rules /etc/udev/rules.d/ && \
    udevadm control --reload-rules && udevadm trigger && \
    ldconfig

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
