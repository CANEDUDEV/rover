ARG ROS_DISTRO=jazzy

# Build stage
FROM ros:$ROS_DISTRO-ros-base AS base

WORKDIR /app

COPY rover_py/rover rover
COPY ros-gateway/ros-package .

RUN bash -c 'source "/opt/ros/$ROS_DISTRO/setup.bash" && colcon build'

# Run stage
FROM ros:$ROS_DISTRO-ros-core

RUN apt-get update \
    && apt-get install -y python3-can \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app
COPY --from=base /app .

COPY ros-gateway/entrypoint.sh .
RUN chmod a+x entrypoint.sh

# Create user if it doesn't exist
RUN bash -c 'id -u 1000 &>/dev/null || useradd -m -s /bin/bash ubuntu'

RUN chown -R 1000:1000 /app

USER 1000:1000

ENTRYPOINT ["./entrypoint.sh"]
