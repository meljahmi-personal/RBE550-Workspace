FROM ros:humble-ros-base
SHELL ["/bin/bash","-c"]
ENV DEBIAN_FRONTEND=noninteractive ROS_DISTRO=humble

RUN apt-get update && apt-get install -y --no-install-recommends \
      python3-pip python3-colcon-common-extensions \
  && rm -rf /var/lib/apt/lists/* \
  && pip3 install --no-cache-dir numpy

WORKDIR /ws

# Copy sources and maps into container
COPY src /ws/src
COPY maps /ws/maps

RUN source /opt/ros/${ROS_DISTRO}/setup.bash && colcon build --merge-install

COPY scripts/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]

