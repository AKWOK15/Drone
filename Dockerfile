FROM arm64v8/ros:humble-ros-base                                                                                                                           
                                                                                                                                          
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
    meson \
    ninja-build \
    libsystemd-dev \
    libgtest-dev \
    && rm -rf /var/lib/apt/lists/*

# Install GeographicLib datasets (required by MAVROS)
RUN /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh

RUN pip3 install \
    "depthai==2.29.0.0" \
    numpy \
    inference-sdk && \
    pip3 install ultralytics && \
    pip3 install "opencv-python==4.5.4.60" --force-reinstall
WORKDIR /ros2_ws

COPY . /ros2_ws
COPY run.sh /usr/local/bin/
COPY run_mavrouter.sh /usr/local/bin/

RUN cd /ros2_ws/src/mavlink-router && \
    meson setup build --wipe && \
    ninja -C build && \
    ninja -C build install

RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install"

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc

CMD ["/bin/bash"]












