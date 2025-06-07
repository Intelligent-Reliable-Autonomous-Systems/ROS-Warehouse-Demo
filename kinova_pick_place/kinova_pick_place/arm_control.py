#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class ArmControl(Node):
    def __init__(self):
        super().__init__('arm_control')
        
        self.joint_trajectory_pub = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10)

    def move_to_joint_positions(self, positions):
        """Move the robot to specified joint positions"""
        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = [
            'kinova/joint_1',
            'kinova/joint_2',
            'kinova/joint_3',
            'kinova/joint_4',
            'kinova/joint_5',
            'kinova/joint_6'
        ]
        
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = 2
        trajectory_msg.points.append(point)
        
        self.joint_trajectory_pub.publish(trajectory_msg)
        self.get_logger().info(f'Moving to joint positions: {positions}')

def main(args=None):
    rclpy.init(args=args)
    node = ArmControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 