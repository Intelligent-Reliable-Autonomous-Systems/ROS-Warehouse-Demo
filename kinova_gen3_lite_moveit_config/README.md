# Kinova Pick and Place Package

This ROS2 package implements a pick and place task for the Kinova Gen3 Lite robot arm. The package provides functionality to pick up blocks from specified positions and place them in a storage bin.

## Installation

From your ROS2 workspace,

1. `rosdep install --from-paths src --ignore-src -r -y`
2. `sudo apt-get install ros-jazzy-ros-gz && sudo apt-get install ros-jazzy-clearpath-simulator`
3. `colcon build --executor sequential --allow-overriding kortex_description`
4. `source install/local_setup.bash `

## Usage

1. Launch the warehouse simulation

`ros2 launch warehouse_sim launch_warehouse.py`

2. Run the pick and place node with default parameters:

`ros2 run kinova_pick_place block_pick_place`

Alternatively, the position of the block can be passed as follows

```bash
ros2 run kinova_pick_place block_pick_place --ros-args \
    -p block_x:=9.0 \
    -p block_y:=-0.6 \
    -p block_z:=1.05
```

## Node Structure

The package consists of the following main components:

1. `BlockPickPlace` Node
   - Handles gripper control
   - Manages arm movement
   - Implements pick and place sequence

2. `ArmControl` Class
   - Handles inverse kinematics
   - Manages joint position control

## Pick and Place Sequence

The node executes the following sequence:

1. Opens gripper
2. Moves to approach position
3. Moves to pick position
4. Closes gripper with increased force
5. Waits for secure grip
6. Lifts block with smooth motion
7. Moves to storage bin position
8. Releases block
9. Returns to home position

## Known Issues

One of the fingers in the gripper is limp. It should ideally mimic the other finger being controlled by an actuator, but this fails. Possibly linked to the issue listed in the [official ROS2 Kortex Repo](https://github.com/Kinovarobotics/ros2_kortex/tree/main?tab=readme-ov-file#gazebo-and-mimic-joints)
