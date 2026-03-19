# docker/Dockerfile
ARG ROS_DISTRO=humble
FROM osrf/ros:$ROS_DISTRO-desktop-full

# Disable interactive prompts during package installation
ARG DEBIAN_FRONTEND=noninteractive

# Install apt-get dependencies
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
      libeigen3-dev \
      build-essential \
      iputils-ping \
      cmake \
      git \
      doxygen \
      python3-colcon-common-extensions \
      python3-pip \
      pkg-config \
      wget \
      ros-$ROS_DISTRO-foxglove-bridge && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

# Install python dependencies
RUN pip3 install roboticstoolbox-python

# Re-enable interactive prompts
ENV DEBIAN_FRONTEND=dialog

# Set locale (i.e. standard for displaying language/region-specific characters)
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

# Workdir for builds
WORKDIR /impedance_control

# FastDDS set-up
COPY fastdds_client_profile.xml /etc/fastdds_client_profile.xml
ENV FASTRTPS_DEFAULT_PROFILES_FILE=/etc/fastdds_client_profile.xml
ENV ROS_LOCALHOST_ONLY=0

# Expose the default Foxglove port
EXPOSE 8765