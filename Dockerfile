FROM osrf/ros:humble-desktop-full

RUN apt-get update && apt-get install -y \
    libpcl-dev \
    iproute2 \
    libapr1-dev \
    python3-ament-package \
    python3-colcon-common-extensions \
    python3-pip \
    libeigen3-dev \
    libboost-all-dev \
    libaprutil1-dev \
    git \
    cmake \
    iputils-ping \
    build-essential \
    wget \
    gcc \
    ros-humble-rosidl-typesupport-c \
    ros-humble-rosidl-default-generators \
    ros-humble-ament-cmake-auto \
    ros-humble-slam-toolbox \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    && rm -rf /var/lib/apt/lists/*


WORKDIR /root/ws_livox/src
COPY ./livox_ros_driver2 ./livox_ros_driver2

WORKDIR /root
COPY ./Livox_SDK2 ./Livox_SDK2

WORKDIR /root/ws_fast_lio/src
COPY ./FAST_LIO ./FAST_LIO

WORKDIR /root/ws_integration/src
COPY ./fast_lio_nav2_integration ./fast_lio_nav2_integration

# build Livox SDK
WORKDIR /root/Livox_SDK2
RUN mkdir build && cd build && cmake .. && make -j && make install

# build livox_ros_driver2
WORKDIR /root/ws_livox/src/livox_ros_driver2
RUN chmod +x build.sh
RUN /bin/bash -c "source /opt/ros/humble/setup.sh && \
    ./build.sh humble"

# build FAST_LIO
WORKDIR /root/ws_fast_lio
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    source /root/ws_livox/install/setup.bash && \
    rosdep install --from-paths src --ignore-src -y && \
    colcon build --symlink-install && \
    . ./install/setup.bash"

WORKDIR /root/ws_pcl_to_ls/src
RUN git clone https://github.com/ros-perception/pointcloud_to_laserscan.git -b humble

WORKDIR /root/ws_pcl_to_ls
RUN bash -c ". /opt/ros/humble/setup.bash && colcon build --symlink-install"
RUN echo "source /root/ws_pcl_to_ls/install/setup.bash" >> /root/.bashrc

WORKDIR /root/ws_integration
RUN bash -c ". /opt/ros/humble/setup.bash && colcon build --symlink-install"


WORKDIR /root
COPY ./docker-entrypoint.sh /root/docker-entrypoint.sh
RUN chmod +x /root/docker-entrypoint.sh

ENTRYPOINT [ "/root/docker-entrypoint.sh" ]
CMD ["/bin/bash"]
