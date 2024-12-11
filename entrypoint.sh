#!/bin/bash

# Setup ROS 2 environment
source /opt/ros/humble/setup.bash

# Optionally, source workspace setup if you have any packages
# source /workspace/install/setup.bash

# Execute the command passed to the container (either bash or other commands)
exec "$@"

