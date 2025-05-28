### Installation Guide

1. mkdir ros2_ws/src -p
2. cd ros2_ws
3. git clone git@github.com:Intelligent-Reliable-Autonomous-Systems/ROS-Warehouse-Demo.git src
4. rosdep install --from-paths src --ignore-src -r -y
5. colcon build
6. source install/local_setup.bash 

### Launch simulation
1. ros2 launch warehouse_sim launch_warehouse.py

### To test movement of Jackal
ros2 topic pub /j100_0000/cmd_vel geometry_msgs/msg/TwistStamped \
"{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'base_link'}, twist: {linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}" \
--once

### To test movement of Kinova Arm

ros2 topic pub /joint_trajectory_controller/joint_trajectory trajectory_msgs/JointTrajectory "{
  joint_names: [kinova/joint_1, kinova/joint_2, kinova/joint_3, kinova/joint_4, kinova/joint_5, kinova/joint_6],
  points: [
    { positions: [1.75, 1, -0.5, 1, -1.5, 1.5], time_from_start: { sec: 2 } },
  ]
}"