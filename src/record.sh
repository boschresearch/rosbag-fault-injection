#!/bin/bash
#
# Copyright (c) 2024 Robert Bosch GmbH and its subsidiaries.
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not  use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#  http://www.apache.org/licenses/LICENSE-2.0
# 
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

__author__ = "Min Hee Jo"
__copyright__ = "Copyright 2024, Robert Bosch GmbH"
__license__ = " Apache"
__version__ = "2.0"
__email__ = minhee.jo@de.bosch.com


# define usage function
usage() {
    echo "Usage: $(basename "$0") [-d value] [-h] [-l str] [-p str] [-y]"
    echo "Options:"
    echo "  -d value    The duration for recording the rosbag (unit: seconds) [Default: 20]"
    echo "  -h          Print manual" 
    echo "  -l str      ROS2 launch file name [Default: all_launch.py]"
    echo "  -p str      ROS2 package name [Default: num_generator]"
    echo "  -y          Auto-confirm the parameters [Default: Deactivated]" 
}

# default values for options
DURATION=20
LAUNCH_FILE="all_launch.py"
PACKAGE="num_generator"

# parameter inputs
while getopts "d:hl:p:y" opt; do
    case $opt in
        d)
            DURATION=$OPTARG
            ;;
        h)
            usage
            exit 1
            ;;
        l)
            LAUNCH_FILE=$OPTARG
            ;;
        p)
            PACKAGE=$OPTARG
            ;;
        y)
            CONFIRM="Y"
            ;;
        \?)
            echo "Invalid option: -$OPTARG" >&2
            usage
            exit 1
            ;;
    esac
done

# remove processed parameters
shift $((OPTIND - 1))

# always execute at working directory
cd "$(dirname "$0")/.."

# bag file directory
BAG_DIR="${PWD}/data/input"

echo "Approximate recording duration: ${DURATION} seconds"
echo "Launch file: ${LAUNCH_FILE}"
echo "Package name: ${PACKAGE}"
echo "Recording the rosbags in $BAG_DIR"

while true; do
    # if not defined
    if [ -z ${CONFIRM} ]; then
        # check the parameter
        read -p "Are the parameters correct? (Y/N) " CONFIRM
    fi

    case "$CONFIRM" in
        [Yy])
            # make directory if not exists
            if [[ ! -e $BAG_DIR ]]; then
                mkdir -p $BAG_DIR
            fi
            cd $BAG_DIR

            # record ros2 bag in the background
            ros2 bag record --max-cache-size 0 -a & sleep 5
            cd -

            # launch ros2 nodes in the background
            ros2 launch $PACKAGE $LAUNCH_FILE & sleep $DURATION

            echo "Stopping the command"

            kill %1  # kill the ROS2 launch process
            kill %2  # kill the ROS2 bag record process

            NODE_LIST=$(ros2 node list)

            for NODE in $NODE_LIST; 
            do
                # remove slash in front of the node name
                # kill running node
                pkill ${NODE:1}
            done

            echo "Rosbag recording terminated"
            break
            ;;
        [Nn])
            echo "Exiting the shell"
            break
            ;;
        *)
            echo "Invalid input: Please enter Y or N"
            read -p "Are the parameters correct? (Y/N) " CONFIRM
            ;;
    esac
done