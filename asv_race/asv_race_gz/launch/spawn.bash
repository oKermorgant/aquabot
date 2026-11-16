#!/usr/bin/env bash
#!/usr/bin/env bash

# Check that $1 is provided and is an integer
if ! [[ "$1" =~ ^[0-9]+$ ]]; then
    echo "Usage: $0 <ROS_DOMAIN_ID (integer)>" >&2
    exit 1
fi

launch_py="$(realpath $(dirname $0))/spawn_launch.py"

ROS_DOMAIN_ID=$1 ros2 launch $launch_py num:=$1
