FROM ubuntu:20.04

LABEL maintainer="srinath.bhamidipati@research.iiit.ac.in"

ENV TZ=Asia/Kolkata
ENV LANG=en_US.UTF-8 \
    LANGUAGE=en_US:en \
    LC_ALL=en_US.UTF-8 \
    DEBIAN_FRONTEND=noninteractive \
    CONDA_DIR=/opt/conda \
    PATH=/opt/conda/bin:$PATH

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

# --- Clone librealsense ---
USER container_user
WORKDIR /home/container_user
RUN git config --global http.postBuffer 524288000 && \
    git clone --branch v2.56.2 https://github.com/IntelRealSense/librealsense.git

# --- Build librealsense ---
WORKDIR /home/container_user/librealsense
RUN mkdir -p build && cd build && cmake ../ && make clean && make -j$(nproc)

# --- Install librealsense ---
USER root
RUN cd /home/container_user/librealsense/build && make install

# --- Install ROS packages ---
COPY --chown=container_user:container_user rosPkgs.list /tmp/rosPkgs.list
RUN apt-get update && \
    apt-get install -y --no-install-recommends $(cat /tmp/rosPkgs.list) && \
    rm -rf /var/lib/apt/lists/*

# --- Install Miniforge (Conda) ---
ARG TARGETARCH
RUN if [ "$TARGETARCH" = "arm64" ]; then \
        curl -sL https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-Linux-aarch64.sh -o miniforge.sh ; \
    else \
        curl -sL https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-Linux-x86_64.sh -o miniforge.sh ; \
    fi && \
    bash miniforge.sh -b -p $CONDA_DIR && \
    rm miniforge.sh && \
    conda clean -afy

# --- Conda environment ---
COPY --chown=container_user:container_user environment.yml /tmp/environment.yml
RUN conda env create -f /tmp/environment.yml && \
    conda clean -afy

# --- Setup bashrc (no auto conda activation) ---
RUN echo "source /opt/conda/etc/profile.d/conda.sh" >> /home/container_user/.bashrc && \
    echo "source /opt/ros/noetic/setup.bash" >> /home/container_user/.bashrc && \
    chown container_user:container_user /home/container_user/.bashrc

# --- Final setup ---
USER container_user
WORKDIR /home/container_user

CMD ["/bin/bash"]
