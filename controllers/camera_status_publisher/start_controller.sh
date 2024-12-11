#!/bin/bash

# Source ROS2 and set environment variables
source /opt/ros/foxy/setup.bash

# Set Webots home directory (adjust if necessary)
export WEBOTS_HOME="/opt/ros/foxy"

# Add the directory containing libController.so to LD_LIBRARY_PATH
export LD_LIBRARY_PATH=$WEBOTS_HOME/lib/controller:$LD_LIBRARY_PATH

# Define the controller script path
CONTROLLER_SCRIPT="/home/zaid/webot_proj/controllers/camera_status_publisher/camera_status_publisher.py"

# Trap Ctrl+C (SIGINT) or SIGTERM to stop gracefully
trap "echo 'Stopping controller'; exit" SIGINT SIGTERM

# Check if the controller script exists
if [ ! -f "$CONTROLLER_SCRIPT" ]; then
    echo "Error: Controller script '$CONTROLLER_SCRIPT' not found."
    exit 1
fi

# Start the controller
echo "Starting Webots controller..."
python3 "$CONTROLLER_SCRIPT"

# Check if the controller started successfully
if [ $? -ne 0 ]; then
    echo "Error: Failed to start the Webots controller."
    exit 1
fi

echo "Webots controller started successfully."
