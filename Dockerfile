FROM ros:humble-ros-base
SHELL ["/bin/bash","-c"]
ENV DEBIAN_FRONTEND=noninteractive ROS_DISTRO=humble

# Build tools (colcon) + numpy if your code uses it
RUN apt-get update && apt-get install -y --no-install-recommends \
      python3-pip python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/* \
 && pip3 install --no-cache-dir numpy

# Workspace
WORKDIR /ws

# Copy only source (faster rebuilds; avoid dragging build artifacts)
COPY src /ws/src

# Build the workspace
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && colcon build --merge-install

# Auto-source on container start
COPY scripts/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]


