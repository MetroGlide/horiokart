#########################################################################################
# Base image
#########################################################################################
FROM tiryoh/ros-desktop-vnc:melodic
#FROM ros:melodic-ros-core


#########################################################################################
# Install tools
#########################################################################################
RUN apt-get update && apt-get install -y curl git cmake \
    && apt-get autoremove --purge \
    && rm -rf /var/cache/apt/*


#########################################################################################
# Install basic ROS tools
#########################################################################################
RUN apt-get update && apt-get install -y \
    python-rosdep python-rosinstall python-rosinstall-generator \
    python-wstool build-essential python-catkin-tools \
    && apt-get autoremove --purge \
    && rm -rf /var/cache/apt/* \
    rosdep init && rosdep update


#########################################################################################
# Install python packages
#########################################################################################
RUN apt-get update && apt-get install -y python3-pip \
    && apt-get autoremove --purge \
    && rm -rf /var/cache/apt/*

WORKDIR /app
COPY ./docker/requirements.txt /app/docker/requirements.txt
RUN pip3 install -r /app/docker/requirements.txt


#########################################################################################
# Build ROS custom packages
#########################################################################################
#WORKDIR /app/org
#COPY . /app/org
#
WORKDIR /root/catkin_ws/src
RUN /bin/bash -c "ln -s /app/horiokart ./"
RUN git clone -b melodic-devel https://github.com/ros-drivers/nmea_navsat_driver.git

WORKDIR /root/catkin_ws
#RUN rosdep update && rosdep install -i -y --from-paths src/nmea_navsat_driver
RUN /bin/bash -c "source /opt/ros/melodic/setup.bash; catkin_make"


#########################################################################################
# Runtime
#########################################################################################
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES utility,graphics