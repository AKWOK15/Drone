FROM osrf/ros:humble-desktop

# Use bash for all RUN steps so `source` works as expected
SHELL ["/bin/bash", "-c"]

RUN apt-get update \
    && apt-get install -y \
    nano \
    vim \
    git \
    curl \
    wget \
    lsb-release \
    gnupg \
    sudo \
    build-essential \
    pkg-config \
    python3-pip \
    python3-dev \
    python3-matplotlib \
    python3-lxml \
    python3-pygame \
    python3-wxgtk4.0 \
    ros-humble-mavros \
    ros-humble-mavros-msgs \
    ros-humble-mavros-extras \
    ros-humble-geographic-msgs \
    libusb-1.0-0-dev \
    udev \
    ninja-build \
    libsystemd-dev \
    libgtest-dev \
    gcc \
    g++ \
    systemd \
    minicom \
    ffmpeg \
    libsm6 \
    libxext6 \
    socat \
    screen \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install "meson==1.1.1" \
    && pip3 install "depthai==2.29.0.0" \
    && pip3 install "numpy==2.1.3" \
    && pip3 install "opencv-python==4.10.0.84" \
    && pip3 install "ultralytics==8.4.66" \
    && pip3 install "inference-sdk==1.3.0"

# Create a non-root user
ARG USERNAME=Drone
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME \
  && groupadd -f video \
  && groupadd -f render \
  && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
  && usermod -aG video,render $USERNAME \
  && mkdir /home/$USERNAME/.config && chown $USER_UID:$USER_GID /home/$USERNAME/.config

# Set up sudo
RUN echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
  && chmod 0440 /etc/sudoers.d/$USERNAME
ENV GZ_VERSION=harmonic

# Install gz-harmonic
RUN curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null \
    && apt-get update \
    && apt-get install -y gz-harmonic

USER $USERNAME
WORKDIR /home/$USERNAME

RUN sudo apt install default-jre \
    && sudo apt-get install gitk git-gui \
    && sudo apt-get install gcc-arm-none-eabi -y

RUN cd ~/ \
    && git clone --recurse-submodules https://github.com/ardupilot/Micro-XRCE-DDS-Gen.git
ENV PATH="/home/${USERNAME}/Micro-XRCE-DDS-Gen/scripts:${PATH}"
RUN cd ~/Micro-XRCE-DDS-Gen \
    && ./gradlew assemble

RUN cd ~/ \
    && git clone https://github.com/ArduPilot/ardupilot.git \
    && cd ardupilot \
    && git submodule update --init --recursive \
    && git submodule init \
    && git submodule update \
    && git status

RUN cd ~/ardupilot \
    # clean objects produced by build
    && ./waf distclean \
    && ./waf distclean \
    && ./waf configure --board MatekH743

RUN mkdir -p ~/ws/src

COPY extra.repos /home/${USERNAME}/ws/extra.repos

RUN source /opt/ros/humble/setup.bash \
    && cd ~/ws/ \
    && vcs import --recursive --input https://raw.githubusercontent.com/Jagadeesh-pradhani/ROS2_ardupilot_Iris_docker/main/extra.repos src \
    && sudo apt update \
    && rosdep update \
    && rosdep install -y --from-paths src --ignore-src

RUN source /opt/ros/humble/setup.bash \
    && cd ~/ws \
    && colcon build

RUN mkdir -p ~/ros2_ws/src \
    && cd ~/ros2_ws

RUN source /opt/ros/humble/setup.bash \
    && cd ~/ros2_ws/ \
    && vcs import --recursive --input https://raw.githubusercontent.com/Jagadeesh-pradhani/ROS2_ardupilot_Iris_docker/main/ros2.repos src \
    && sudo apt update \
    && rosdep update \
    && rosdep install -y --from-paths src --ignore-src

RUN sudo pip3 install pexpect
RUN  source /opt/ros/humble/setup.bash \
    && source ~/ws/install/setup.bash \
    && cd ~/ros2_ws \
    && colcon build --packages-up-to ardupilot_dds_tests --event-handlers console_direct+

ARG USER=$USERNAME
RUN cd ~/ardupilot \
    && sudo apt-get install -y python3-pip \
    && sudo pip3 install future \
    && sudo chmod +x Tools/environment_install/install-prereqs-ubuntu.sh \
    && Tools/environment_install/install-prereqs-ubuntu.sh -y \
    && sudo apt-get install -y python3-pexpect \
    && ./waf clean \
    && ./waf configure --board sitl \
    && ./waf copter -v

RUN cd ~/ardupilot/Tools/autotest \
    && sudo pip3 install MAVProxy \
    && sudo pip3 install MAVProxy[joystick]

RUN source /opt/ros/humble/setup.bash \
    && source ~/ws/install/setup.bash \
    && cd ~/ros2_ws/ \
    && colcon build --packages-up-to ardupilot_sitl

RUN source /opt/ros/humble/setup.bash \
    && cd ~/ros2_ws \
    && vcs import --input https://raw.githubusercontent.com/Jagadeesh-pradhani/ROS2_ardupilot_Iris_docker/main/ros2_gz.repos --recursive src \
    && sudo apt update \
    && rosdep update \
    && rosdep install -y --from-paths src --ignore-src -r

RUN source /opt/ros/humble/setup.bash \
    && source ~/ws/install/setup.bash \
    && cd ~/ros2_ws \
    && colcon build --packages-up-to ardupilot_gz_bringup

RUN source /opt/ros/humble/setup.bash \
    && source ~/ws/install/setup.bash \
    && cd ~/ros2_ws/src/ \
    && git clone https://github.com/ArduPilot/ardupilot_ros.git \
    && cd ~/ros2_ws/ \
    && rosdep install --from-paths src --ignore-src -r -y --skip-keys gazebo-ros-pkgs \
    && colcon build --packages-up-to ardupilot_ros --parallel-workers 12

COPY ros2_ws/src/ /home/${USERNAME}/ros2_ws/src/
COPY run.sh /home/${USERNAME}/
COPY run_mavrouter.sh /home/${USERNAME}/
COPY config/ /home/${USERNAME}/config/

RUN sudo /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh

RUN cd ros2_ws/src && \
    git clone https://github.com/mavlink-router/mavlink-router.git
RUN cd ros2_ws/src/mavlink-router && \
    git submodule update --init --recursive && \
    meson setup build --wipe && \
    ninja -C build && \
    sudo ninja -C build install

COPY entrypoint.sh /entrypoint.sh
COPY bashrc /home/${USERNAME}/.bashrc

# Source ROS and built packages
ENTRYPOINT ["/bin/bash", "/entrypoint.sh"]
CMD ["bash"]
