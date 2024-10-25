#!/bin/bash
set -eo pipefail

# setup ros2 env
# shellcheck disable=SC1090,SC1091,SC2154
source "/opt/ros/${ROS_DISTRO}/setup.bash"

# setup gateway env
# shellcheck disable=SC1091
source "./install/setup.bash"

ros2 run gateway gateway "$@"
