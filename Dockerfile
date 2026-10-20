FROM docker.io/library/ros:lyrical-ros-base

RUN apt-get update && apt-get install -y \
	ros-$ROS_DISTRO-ros2-control \
	ros-$ROS_DISTRO-ros2-controllers \
	&& rm -rf /var/lib/apt/lists/*

RUN rosdep update \
	&& rm -rf /var/lib/apt/lists/*

RUN mkdir /opt/ros/overlay_ws

WORKDIR /opt/ros/overlay_ws
# SHELL ["/bin/sh", "-c"]

COPY ./fairino_description/package.xml ./src/fairino_description/package.xml
COPY ./fairino_hardware/package.xml ./src/fairino_hardware/package.xml
COPY ./fairino_msgs/package.xml ./src/fairino_msgs/package.xml
COPY ./fairino_hardware_v3_9_9/package.xml ./src/fairino_hardware_v3_9_9/package.xml

RUN apt-get update \
	&& . /opt/ros/$ROS_DISTRO/setup.sh \
	&& rosdep install -i --from-path . -y \
	&& rm -rf /var/lib/apt/lists/*

COPY ./fairino_description/ ./src/fairino_description/
COPY ./fairino_hardware/ ./src/fairino_hardware/
COPY ./fairino_msgs/ ./src/fairino_msgs/
COPY ./fairino_hardware_v3_9_9/ ./src/fairino_hardware_v3_9_9/

ARG THREADS=2
RUN . /opt/ros/$ROS_DISTRO/setup.sh \
	&& colcon build --parallel-workers $THREADS \
	&& rm -rf logs
