#!/bin/bash
set -e

source /opt/ros/humble/setup.bash

# source conda
source /opt/conda/etc/profile.d/conda.sh
conda activate robot_ros

cd /robot_ws
if [ -f install/.colcon_install_layout ] && grep -qx merged install/.colcon_install_layout; then
  colcon build --merge-install --symlink-install
else
  colcon build --symlink-install
fi
# dependency
if [ -d "/robot_ws/src" ]; then 
   source install/setup.bash
fi

# run the codes
# (A) 直接跑控制程式
# ex: python3 src/your_pkg/your_kachaka_controller.py

# (B) 跑Kachaka ROS2 topic/service
# ex: ros2 run kachaka_ros2_bridge ros2_bridge

exec /bin/bash