# syntax=docker/dockerfile:1

FROM osrf/ros:humble-desktop

ARG USERNAME=humble
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN groupadd --gid $USER_GID $USERNAME \
&& useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
&& apt-get update \
&& apt-get install -y sudo \
&& echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
&& chmod 0440 /etc/sudoers.d/$USERNAME

# Grant user privileges
RUN usermod -aG dialout $USERNAME \
    && usermod -aG tty $USERNAME

USER $USERNAME

# Update packages
RUN sudo apt update \
    && sudo apt upgrade -y

RUN sudo apt install -y \
    git
 
# Rosdep update
RUN rosdep update
 
# Source the ROS setup file
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
 
WORKDIR /home/$USERNAME/drone_ws/
 
# Clone and build px4_msgs and MicroXRCE_Agent
RUN git clone --branch release/1.14 https://github.com/PX4/px4_msgs.git --recursive src/px4_msgs \
    && /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash; colcon build --packages-select px4_msgs" \
    && git clone --branch ros2 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git /tmp/micro-xrce-dds-agent \
    && cd /tmp/micro-xrce-dds-agent \
    && mkdir build && cd build \
    && cmake .. \
    && make -j $(nproc) \
    && sudo make install

# Source local_setup folder 
RUN echo "source /home/$USERNAME/drone_ws/install/local_setup.bash" >> ~/.bashrc
 
RUN sudo ldconfig /usr/local/lib \
&& sudo rm -rf /tmp/*

# docker run ... --device=/dev/ttyUSB0 ...