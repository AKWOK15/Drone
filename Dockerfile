FROM arm64v8/ros:humble-ros-base
# Approves all downloads
ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y \
    curl \
    git \
    wget \
    build-essential \
    pkg-config \
    python3-pip \
    python3-dev \
    python3-colcon-common-extensions \
    python3-setuptools \
    ros-humble-mavros \
    ros-humble-mavros-msgs \
    ros-humble-mavros-extras \
    ros-humble-geographic-msgs \
    ros-humble-launch \
    ros-humble-launch-ros \
    libusb-1.0-0-dev \
    udev \
    ninja-build \
    libsystemd-dev \
    libgtest-dev \
    gcc \
    g++ \
    systemd \
    minicom \
    vim \
    ffmpeg \
    libsm6 \
    libxext6 \
    vcstool \
    default-jre \
    lsb-release \
    gnupg \
    && rm -rf /var/lib/apt/lists/*

# Install Gazebo Harmonic
RUN curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null && \
    apt-get update && \
    apt-get install -y gz-harmonic && \
    rm -rf /var/lib/apt/lists/*

ENV GZ_VERSION=harmonic

COPY ros2_ws/ /root/Drone/ros2_ws/
COPY run.sh /root/Drone/
COPY run_mavrouter.sh /root/Drone/

WORKDIR /root/Drone/ros2_ws

# Install ArduPilot for Gazebo Simulation
RUN vcs import --recursive --input https://raw.githubusercontent.com/ArduPilot/ardupilot/master/Tools/ros2/ros2.repos src

RUN apt-get update && \
    rosdep update && \
    . /opt/ros/humble/setup.bash && \
    rosdep install --from-paths src --ignore-src -r -y

RUN git clone --recurse-submodules --branch v4.7.0 https://github.com/ardupilot/Micro-XRCE-DDS-Gen.git && \
    cd Micro-XRCE-DDS-Gen && \
    ./gradlew assemble

ENV PATH="/root/Drone/ros2_ws/Micro-XRCE-DDS-Gen/scripts:${PATH}"

RUN . /opt/ros/humble/setup.bash && \
    colcon build --packages-up-to ardupilot_dds_tests --event-handlers=console_cohesion+

# Install SITL
RUN cd src/ardupilot && ./Tools/environment_install/install-prereqs-ubuntu.sh -y

RUN . /opt/ros/humble/setup.bash && \
    colcon build --packages-up-to ardupilot_sitl

# Install ArduPilot <-> Gazebo bridge packages
RUN vcs import --input https://raw.githubusercontent.com/ArduPilot/ardupilot_gz/main/ros2_gz.repos --recursive src

RUN wget https://raw.githubusercontent.com/osrf/osrf-rosdep/master/gz/00-gazebo.list -O /etc/ros/rosdep/sources.list.d/00-gazebo.list && \
    rosdep update

RUN apt-get update && \
    . /opt/ros/humble/setup.bash && \
    rosdep install --from-paths src --ignore-src -r -y

RUN . /opt/ros/humble/setup.bash && \
    colcon build --packages-up-to ardupilot_gz_bringup

# weights for OAK-D pipelines (paths referenced by perception_py)
COPY gun_model/ /root/Drone/gun_model/
COPY person_model/ /root/Drone/person_model/

# mavlink-router config (run_mavrouter.sh expects ~/Drone/config/mavlink-router/main.conf)
COPY config/ /root/Drone/config/

# Install GeographicLib datasets (required by MAVROS)
RUN /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh

RUN pip3 install "meson==1.1.1"
RUN pip3 install "depthai==2.29.0.0"
RUN pip3 install "numpy==2.1.3"
RUN pip3 install "opencv-python==4.10.0.84"
RUN pip3 install "ultralytics==8.4.66"
RUN pip3 install "inference-sdk==1.3.0"

RUN cd src/mavlink-router && \
    meson setup build --wipe && \
    ninja -C build && \
    ninja -C build install

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source /root/Drone/ros2_ws/install/setup.bash" >> /root/.bashrc

WORKDIR /root/Drone
ENV SHELL=/bin/bash
# When creating image, need to give it some default comand
CMD ["/bin/bash"]
