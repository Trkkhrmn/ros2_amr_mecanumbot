#!/bin/bash
set -e
. /opt/ros/humble/setup.sh
[ -f /ros2_ws/install/setup.sh ] && . /ros2_ws/install/setup.sh
exec "$@"
