#!/bin/bash
#
# Copyright (c) 2024 Robert Bosch GmbH and its subsidiaries.
# This program and the accompanying materials are made available under
# the terms of the Bosch Internal Open Source License v4
# which accompanies this distribution, and is available at
# http://bios.intranet.bosch.com/bioslv4.txt
#
# Author        : Min Hee Jo
# Email         : minhee.jo@de.bosch.com
# Usage         : bash inject.sh -h
# Description   : Inject faults to rosbags and plot the results according to user options and config.yaml

# define usage function
usage() {
    echo "Usage: $(basename "$0") [-c str] [-e str] [-h] [-p str] [-n]"
    echo "Options:"
    echo "  -c str      The filename of configuration YAML [Default: config.yaml]"
    echo "          !!! The YAML file should be located in ${PWD}/$(dirname "$0")"
    echo "  -e str      Executable name to re-record the rosbags to see the fault injection effect"
    echo "  -h          Print manual"
    echo "  -p str      ROS2 package name [Default: num_generator]"
    echo "  -n          Disable rosbag play and plotting after the fault injection [Default: Deactivated]"   
}

# default values for options
YAML_FILENAME="config.yaml"
PACKAGE="num_generator"
NO_PLOT=false

# parameter inputs
while getopts "c:e:hp:n" opt; do
    case $opt in
        c)
            YAML_FILENAME=$OPTARG
            ;;
        e)
            EXECUTABLE_NAME=$OPTARG
            ;;
        h)
            usage
            exit 1
            ;;
        p)
            PACKAGE=$OPTARG
            ;;
        n)
            NO_PLOT=true
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

BASE_PATH="${PWD}/$(dirname "$0")"
YAML_PATH="${BASE_PATH}/${YAML_FILENAME}"

# config file does not exist
if [ ! -f "${YAML_PATH}" ]; then
    echo "Config file ${YAML_PATH} do not exist"
    exit 1
fi

echo "Injecting faults as specified in ${YAML_PATH}"

# inject fault
python_result=$(python3 src/fault_injection.py "$YAML_PATH" 2>&1)

if [ $? -ne 0 ]; then
    echo "Injecting fault encourtered an error: Check if the input rosbag exists"
    echo "$python_result"
    exit 1
else
    echo "$python_result"
fi

if [[ "${NO_PLOT}" == "false" ]]; then
    # executable name is necessary
    if [ -z "$EXECUTABLE_NAME" ]; then
        echo "Executable name not given for plotting the result"
        exit 1
    fi

    echo "Playing the fault injected rosbag"

    TEMP_FILES=($(find data/temp/* -type d))

    for FILE in "${TEMP_FILES[@]}"; do
        OUTPUT_FILE=${FILE/"temp"/"output"}

        # record ros2 bag in the background
        ros2 bag record -o $OUTPUT_FILE -a & sleep 5

        # run specific node
        ros2 run $PACKAGE $EXECUTABLE_NAME > /dev/null 2>&1 &
        
        # play fault injected rosbags
        ros2 bag play $FILE &
        wait $!

        echo "Stopping the command"

        pkill $EXECUTABLE_NAME
        pkill ros2
    done

    echo "Plotting the rosbag"

    # plot rosbag
    python_result=$(python3 src/plot.py "$YAML_PATH" 2>&1)

    rm -rf data/temp data/output

    if [ $? -ne 0 ]; then
        echo "Plotting encourtered an error"
        echo "$python_result"
        exit 1
    else
        echo "$python_result"
    fi
fi