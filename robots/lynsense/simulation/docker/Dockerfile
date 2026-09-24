FROM cyberbotics/webots@sha256:f0023e30daf38b172e4e6ad24ed345909bcd9551df34d63d824e121a7cebf099

ENV DEBIAN_FRONTEND=noninteractive ROS_DISTRO=humble
RUN apt-get update && apt-get install -y --no-install-recommends \
      curl gnupg lsb-release software-properties-common ca-certificates \
    && install -d /usr/share/keyrings \
    && curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc \
       -o /usr/share/keyrings/ros-archive-keyring.asc \
    && echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.asc] http://packages.ros.org/ros2/ubuntu jammy main" \
       > /etc/apt/sources.list.d/ros2.list \
    && apt-get update && apt-get install -y --no-install-recommends \
      ros-humble-ros-base \
      ros-humble-rosidl-default-generators \
      ros-humble-rosidl-default-runtime \
      ros-humble-py-trees \
      ros-humble-py-trees-ros \
      python3-colcon-common-extensions \
      python3-yaml \
      ffmpeg \
      xvfb \
    && rm -rf /var/lib/apt/lists/* \
    && useradd --uid 1000 --create-home --shell /bin/bash lynsense \
    && mkdir -p /workspace/ws/src /workspace/artifacts \
    && chown -R lynsense:lynsense /workspace

USER lynsense
WORKDIR /home/lynsense
ENV HOME=/home/lynsense
