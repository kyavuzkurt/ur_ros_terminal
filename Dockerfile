FROM osrf/ros:noetic-desktop-full

# Arguments for user creation
ARG USER=user
ARG DEBIAN_FRONTEND=noninteractive

# Install required ROS packages and system dependencies
RUN apt-get update && \
    apt-get install -y ros-noetic-moveit \
                       ros-noetic-ros-controllers \
                       ros-noetic-gazebo-ros-control \
                       ros-noetic-rosserial \
                       ros-noetic-rosserial-arduino \
                       ros-noetic-roboticsgroup-upatras-gazebo-plugins \
                       ros-noetic-actionlib-tools \
                       terminator \
                       python3-pip \
                       git \
                       ros-noetic-ur-robot-driver \
                       ros-noetic-ur-calibration \
                       ros-noetic-universal-robots\
                       sudo && \

    rm -rf /var/lib/apt/lists/*


# Install Python packages
RUN pip install flask flask-ask-sdk ask-sdk

# Add the user, set as sudoer with no password required
RUN useradd -ms /bin/bash $USER && \
    usermod -aG sudo $USER && \
    usermod -aG dialout $USER &&\
    echo "$USER ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# Switch to user's home directory
WORKDIR /home/${USER}

# Set the user to default in the container
USER ${USER}

