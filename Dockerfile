FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=humble \
    RMW_IMPLEMENTATION=rmw_fastrtps_cpp

RUN apt-get update && apt-get install -y \
      python3-pip python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*
RUN pip3 install --no-cache-dir numpy

WORKDIR /ws
# copy ONLY sources; build artifacts are excluded via .dockerignore
COPY . /ws

# in case host had artifacts, ensure a clean build inside the image
RUN rm -rf build install log || true

SHELL ["/bin/bash","-c"]
RUN source /opt/ros/$ROS_DISTRO/setup.bash && colcon build --merge-install

COPY scripts/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
