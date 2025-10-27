FROM osrf/ros:humble-desktop

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y \
    gazebo \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-xacro \
    ros-humble-turtlebot3-gazebo \
    ros-humble-turtlebot3-msgs \
    ros-humble-turtlebot3-description \
    python3-colcon-common-extensions \
    libgl1-mesa-dri libglx-mesa0 mesa-utils \
    libx11-xcb1 libxrender1 libxrandr2 libxi6 && \
    rm -rf /var/lib/apt/lists/*

ENV TURTLEBOT3_MODEL=waffle

WORKDIR /root/ws
COPY ws ./

RUN rm -rf build install log
RUN bash -lc "source /opt/ros/humble/setup.bash && colcon build --symlink-install"

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
