FROM ros:humble-ros-base
SHELL ["/bin/bash","-c"]
ENV DEBIAN_FRONTEND=noninteractive ROS_DISTRO=humble

# Core tools + RViz + software OpenGL
RUN apt-get update && apt-get install -y --no-install-recommends \
      python3-pip \
      python3-colcon-common-extensions \
      ros-humble-rviz2 \
      mesa-utils \
      libgl1-mesa-dri \
      libgl1-mesa-glx \
  && rm -rf /var/lib/apt/lists/*

# Python deps for benchmarks / plotting
RUN pip3 install --no-cache-dir \
      numpy \
      matplotlib \
      pandas

WORKDIR /ws

# Copy workspace content
COPY src /ws/src
COPY maps /ws/maps
COPY scripts /ws/scripts

# Make all helper scripts executable
RUN chmod +x /ws/scripts/*.sh

# Build the ROS 2 workspace
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && colcon build --merge-install

# Container entrypoint
COPY scripts/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]

