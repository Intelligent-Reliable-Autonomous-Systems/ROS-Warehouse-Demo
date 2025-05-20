### Installation Guide

1. mkdir ros2_ws/src -p
2. cd ros2_ws
3. git clone git@github.com:Intelligent-Reliable-Autonomous-Systems/ROS-Warehouse-Demo.git src
4. rosdep install --from-paths src --ignore-src -r -y
5. colcon build
6. source install/local_setup.bash 

### Launch simulation
1. ros2 launch warehouse_setup launch_warehouse.py

sudo apt-get install ros-jazzy-ros-gz
sudo apt-get install ros-jazzy-clearpath-simulator

wget https://raw.githubusercontent.com/clearpathrobotics/clearpath_simulator/jazzy/dependencies.repos
vcs import src < dependencies.repos
rosdep install -r --from-paths src -i -y
