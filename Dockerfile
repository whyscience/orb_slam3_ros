FROM amd64/ros:humble-perception-jammy

ARG DEBIAN_FRONTEND=noninteractive
ARG ROS_DISTRO=humble

#
# install ORBSLAM3 ROS package
#

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        software-properties-common \
        git \
        build-essential \
        cmake \
        libeigen3-dev \
        # ros-${ROS_DISTRO}-hector-trajectory-server \
        ros-dev-tools \
        libopencv-dev && \
    rm -rf /var/lib/apt/lists/* && \
    apt-get clean

WORKDIR /root

RUN git clone https://github.com/stevenlovegrove/Pangolin.git && \
    cd Pangolin && \
    mkdir build && cd build && \
    cmake .. && \
    make -j && \
    make install

RUN mkdir -p ws_orb_slam3/src && \
    cd ws_orb_slam3/src && \
    git clone https://github.com/whyscience/orb_slam3_ros.git && \
    cd .. && \
    #catkin config --extend /opt/ros/${ROS_DISTRO} && \
    source /opt/ros/${ROS_DISTRO}/setup.bash && \
    colcon build --symlink-install

RUN echo "source /root/ws_orb_slam3/install/setup.bash" >> /root/.bashrc

#
# install RealSenseSDK / RealSense ROS wrapper
#

RUN apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
RUN add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -sc) main"

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        libssl-dev \
        libudev-dev \
        libusb-1.0-0-dev \
        librealsense2-dev \
        librealsense2-utils \
        ros-${ROS_DISTRO}-realsense2-camera &&  \
    rm -rf /var/lib/apt/lists/* && \
    apt-get clean
