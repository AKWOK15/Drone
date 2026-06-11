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
    && rm -rf /var/lib/apt/lists/*

# Install GeographicLib datasets (required by MAVROS)
RUN /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh
RUN pip3 install \
    "meson==1.1.1"






COPY ros2_ws/ /root/Drone/ros2_ws/
COPY run.sh /usr/local/bin/
COPY run_mavrouter.sh /usr/local/bin/

# Model weights for OAK-D pipelines (paths referenced by perception_py)
COPY gun_model/ /root/Drone/gun_model/
COPY person_model/ /root/Drone/person_model/

# mavlink-router config (run_mavrouter.sh expects ~/Drone/config/mavlink-router/main.conf)
COPY config/ /root/Drone/config/

RUN cd /root/Drone/ros2_ws/src/mavlink-router && \
    meson setup build --wipe && \
    ninja -C build && \
    ninja -C build install

RUN pip3 install \
    "depthai==2.29.0.0" \
    "numpy==2.1.3" \
    "opencv-python==4.10.0.84" \
    "ultralytics==8.4.66" \
    "inference-sdk==1.3.0" 


RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

WORKDIR /root/Drone
ENV SHELL /bin/bash
# When creating image, need to give it some default comand
CMD ["/bin/bash"]












