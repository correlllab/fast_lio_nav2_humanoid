#!/bin/bash
set -e # Exit immediately if a command exits with a non-zero status

# --- Welcome Message ---
echo "######################################################################"
echo "#                                                                    #"
echo "#      Welcome to Nav2 with FastLIO in Docker for the Unitree H1-2   #"
echo "#                          By:Correll Lab                            #"
echo "#                                                                    #"
echo "#                          steps to run:                             #"
echo "#                                                                    #"
echo "#         ros2 launch livox_ros_driver2 msg_MID360_launch.py         #"
echo "#                                                                    #"
echo "#                     open  1 new terminals and run:                 #"
echo "#                                                                    #"
echo "#              sudo docker exec -it nav2_humble /bin/bash            #"
echo "#                                                                    #"
echo "#                    source ./docker-entrypoint.sh                   #"
echo "#                                                                    #"
echo "#          ros2 launch fast_lio_nav2_integration nav2.launch.py      #"
echo "#                                                                    #"
echo "######################################################################"
echo ""

echo "                   --- Setting up ROS Environment ---"
echo ""
# 1. Source ROS Humble's global setup files

source /opt/ros/humble/setup.bash || { echo "ERROR: Failed to source /opt/ros/humble/setup.bash. Does ROS Humble exist?"; exit 1; }
source /opt/ros/humble/setup.sh || { echo "ERROR: Failed to source /opt/ros/humble/setup.sh. Does ROS Humble exist?"; exit 1; }

# 2. Build the 'ws_livox' workspace (which should contain livox_ros_driver2 and its dependencies)
#    This command should be run from the root of your colcon workspace.

(
  cd /root/ws_livox && \
  colcon build --packages-select livox_ros_driver2 > /dev/null 2>&1 # Redirect both stdout and stderr to /dev/null
) || { echo "ERROR: colcon build of ws_livox failed!"; exit 1; }

# 3. Source the 'ws_livox' workspace setup files

source /root/ws_livox/install/setup.bash || { echo "ERROR: Failed to source /root/ws_livox/install/setup.bash. Is ws_livox built correctly?"; exit 1; }
source /root/ws_livox/install/setup.sh || { echo "ERROR: Failed to source /root/ws_livox/install/setup.sh. Is ws_livox built correctly?"; exit 1; }


# 4. Source the 'ws_fast_lio' workspace setup files
#    This assumes fast_lio is already built and just needs its environment sourced.

source /root/ws_fast_lio/install/setup.bash || { echo "ERROR: Failed to source /root/ws_fast_lio/install/setup.bash. Is ws_fast_lio built correctly?"; exit 1; }

echo "                 --- ROS Environment Setup Complete! ---"

source /root/ws_integration/install/setup.bash || { echo "ERROR: Failed to source /root/ws_integration/install/setup.bash. Is ws_integration built correctly?"; exit 1; }
source /root/ws_integration/install/setup.sh || { echo "ERROR: Failed to source /root/ws_integration/install/setup.sh. Is ws_integration built correctly?"; exit 1; }
# This is the crucial part to prevent the container from exiting immediately:
# Check if the command passed to the entrypoint is "bash" or if no command was passed.
# If so, explicitly run bash interactively to keep the container session alive.
if [ "$1" = "bash" ] || [ -z "$1" ]; then
    exec /bin/bash -i
else
    # Otherwise, execute the command passed to the entrypoint (e.g., 'ros2 launch ...')
    exec "$@"
fi
