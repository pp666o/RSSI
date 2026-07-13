FROM osrf/ros:foxy-desktop

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-foxy-rtabmap-ros \
    ros-foxy-rtabmap-viz \
    ros-foxy-rviz2 \
    ros-foxy-tf2-tools \
    ros-foxy-rosbag2 \
    python3-colcon-common-extensions \
    git \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace

CMD ["/bin/bash"]
