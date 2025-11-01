FROM ros:humble-ros-base

SHELL ["/bin/bash", "-c"]
WORKDIR /ros2_ws

RUN apt-get update && apt-get install -y \
    python3-pip \
    ros-humble-robot-state-publisher \
    ros-humble-rviz2 \
    ros-humble-xacro \
    && rm -rf /var/lib/apt/lists/*

COPY . /ros2_ws/src/waving_arm

RUN source /opt/ros/humble/setup.bash && \
    colcon build --packages-select waving_arm

CMD ["source /ros2_ws/install/setup.bash && ros2 launch waving_arm arm.launch.py"]