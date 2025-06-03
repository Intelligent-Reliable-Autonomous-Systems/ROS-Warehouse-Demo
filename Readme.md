### Installation Guide

1. `mkdir ros2_ws/src -p`
2. `cd ros2_ws/src`
3. `git clone git@github.com:Intelligent-Reliable-Autonomous-Systems/ROS-Warehouse-Demo.git`
4. `vcs import < ros2_ws.repos`
5. `cd ..`
6. `rosdep install --from-paths src --ignore-src -r -y`
7. `sudo apt-get install ros-jazzy-ros-gz && sudo apt-get install ros-jazzy-clearpath-simulator`
8. `colcon build --executor sequential --allow-overriding kortex_description nav2_behavior_tree nav2_core nav2_graceful_controller nav2_msgs nav2_util`
9. `source install/local_setup.bash `

### Launch simulation (without apriltag)
- `ros2 launch warehouse_sim launch_warehouse.py`
   
### Launch simulation (with apriltag)
- `ros2 launch warehouse_sim launch_warehouse_with_apriltags.py`

### To test movement of Jackal
```bash
ros2 topic pub /j100_0000/cmd_vel geometry_msgs/msg/TwistStamped "{
  header: {
    stamp: {sec: 0, nanosec: 0}, frame_id: 'base_link'},
    twist: {linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}
    }
}"
```


### To test movement of Kinova Arm

```bash
ros2 topic pub /joint_trajectory_controller/joint_trajectory trajectory_msgs/JointTrajectory "{
  joint_names: [kinova/joint_1, kinova/joint_2, kinova/joint_3, kinova/joint_4, kinova/joint_5, kinova/joint_6],
  points: [
    { positions: [1.75, 1, -0.5, 1, -1.5, 1.5], time_from_start: { sec: 2 } },
  ]
}"
```

### To view data in Rviz2 with Kinova/Jackal
```bash
ros2 run rviz2 rviz2 --ros-args -r /tf:=/kinova/tf
```
```bash
ros2 run rviz2 rviz2 --ros-args -r /tf:=/j100_0000/tf
```

### To echo tf frames
```bash
ros2 run tf2_tools view_frames --ros-args -r /tf:=/kinova/tf
```
```bash
ros2 run tf2_tools view_frames --ros-args -r /tf:=/j100_0000/tf
```
