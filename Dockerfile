FROM ros:humble as base
SHELL ["/bin/bash", "-c"]
ARG DEBIAN_FRONTEND=noninteractive
ARG USER_UID=1000
ARG USER_GID=1000
ARG USER_NAME=aidan

RUN useradd -o -m -u ${USER_UID} -G sudo,dialout,video,plugdev ${USER_NAME}

RUN echo "${USER_NAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USER_NAME} \
    && chmod 0440 /etc/sudoers.d/${USER_NAME}

RUN apt update && apt install -y \
    curl \
    git \
    wget \
    build-essential \
    pkg-config \
    python3-pip \
    python3-dev \
    python3-colcon-common-extensions \
    python3-setuptools \
    python3-vcstool \
    python3-matplotlib \
    python3-lxml \
    python3-pygame \
    python3-wxgtk4.0 \ 
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
    default-jre \
    lsb-release \
    gnupg \
    socat \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install "meson==1.1.1"
RUN pip3 install "depthai==2.29.0.0"
RUN pip3 install  "numpy==2.1.3"
RUN pip3 install "opencv-python==4.10.0.84"
RUN pip3 install "ultralytics==8.4.66"
RUN pip3 install  "inference-sdk==1.3.0"
RUN pip3 install -U MAVProxy
RUN pip3 install future
ENV GZ_VERSION=harmonic

COPY ros2_ws/ /root/Drone/ros2_ws/
COPY run.sh /root/Drone/
COPY run_mavrouter.sh /root/Drone/
COPY gun_model/ /root/Drone/gun_model/
COPY person_model/ /root/Drone/person_model/
COPY config/ /root/Drone/config/

WORKDIR /root/Drone/ros2_ws

RUN vcs import --recursive --input https://raw.githubusercontent.com/ArduPilot/ardupilot/master/Tools/ros2/ros2.repos src

RUN apt update && \
    rosdep update && \
    . /opt/ros/humble/setup.bash && \
    rosdep install --from-paths src --ignore-src -r -y

RUN git clone --recurse-submodules --branch v4.7.0 https://github.com/ardupilot/Micro-XRCE-DDS-Gen.git && \
    cd Micro-XRCE-DDS-Gen && \
    ./gradlew assemble --no-daemon

ENV PATH="/root/Drone/ros2_ws/Micro-XRCE-DDS-Gen/scripts:${PATH}"

RUN pip3 install pexpect
RUN . /opt/ros/humble/setup.bash && \
    colcon build --packages-up-to ardupilot_dds_tests --event-handlers=console_cohesion+

RUN chown -R ${USER_NAME}:${USER_NAME} /root/Drone/ros2_ws/src/ardupilot && \
    chmod o+x /root
USER ${USER_NAME}
ENV USER=root
RUN cd /root/Drone/ros2_ws/src/ardupilot && ./Tools/environment_install/install-prereqs-ubuntu.sh -y
USER root

RUN git config --global --add safe.directory '*' && \
    colcon build --packages-up-to ardupilot_sitl
RUN . /opt/ros/humble/setup.bash

RUN vcs import --input https://raw.githubusercontent.com/ArduPilot/ardupilot_gz/main/ros2_gz.repos --recursive src

RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

RUN apt update

RUN wget https://raw.githubusercontent.com/osrf/osrf-rosdep/master/gz/00-gazebo.list \
        -O /etc/ros/rosdep/sources.list.d/00-gazebo.list && \
   rosdep update

RUN . /opt/ros/humble/setup.bash && \
    apt update && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    colcon build --packages-up-to ardupilot_gz_bringup

RUN /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh

RUN cd src && \
    git clone https://github.com/mavlink-router/mavlink-router.git

RUN cd src/mavlink-router && \
    git submodule update --init --recursive && \
    meson setup build --wipe && \
    ninja -C build && \
    ninja -C build install

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source /root/Drone/ros2_ws/install/setup.bash" >> /root/.bashrc && \
    echo "export GZ_VERSION=harmonic" >> /root/.bashrc && \
    echo 'export PATH="$PATH:$HOME/.local/bin"' >> /root/.bashrc
WORKDIR /root/Drone
ENV SHELL=/bin/bash
CMD ["/bin/bash"]
