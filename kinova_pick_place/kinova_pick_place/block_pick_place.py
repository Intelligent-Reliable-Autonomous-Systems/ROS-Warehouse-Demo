#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import GripperCommand
import time
from .arm_control import ArmControl

class BlockPickPlace(Node):
    def __init__(self):
        super().__init__('block_pick_place')
        
        # Gripper action client
        self.gripper_action_client = ActionClient(
            self,
            GripperCommand,
            '/gen3_lite_2f_gripper_controller/gripper_cmd'
        )
        
        self.arm_control = ArmControl()
        
        # --- Block position parameters ---
        self.block_x = self.declare_parameter('block_x', 9.2).value
        self.block_y = self.declare_parameter('block_y', -0.6).value
        self.block_z = self.declare_parameter('block_z', 1.05).value
        
        self.basket_positions = [
            (9.0, 0.8, 1.15)  
        ]
        self.basket_position = self.basket_positions[0]
        
        self.storage_basket_angles = [-1.5, 1.75, -0.0, 0.9, 0.0, 2.3] 
        
        self.home_position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  
        
        # --- Pick and place parameters ---
        self.pick_safe_offset = 0.10    
        self.pick_intermediate_offset = 0.03  
        self.pick_final_offset = -0.01   
        self.lift_height = 0.15         
        
        self.place_safe_offset = 0.12    
        self.place_final_offset = 0.02   
        self.lift_out_height = 0.15     
        self.pick_orientation = (0.0, -1.57, 0.0)  
        self.place_orientation = (0.0, -1.57, 0.0)  
        
        self.get_logger().info('Waiting for gripper action server...')
        self.gripper_action_client.wait_for_server()
        self.get_logger().info('Gripper action server connected!')
        self.get_logger().info(f'Block position: x={self.block_x}, y={self.block_y}, z={self.block_z}')

    def control_gripper(self, position):
        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = position
        goal_msg.command.max_effort = 50.0
        
        self.get_logger().info(f'Sending gripper command: position={position}')
        
        future = self.gripper_action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Gripper goal rejected!')
            return False
            
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        result = result_future.result()
        if result:
            self.get_logger().info(f'Gripper successfully moved to position: {position}')
            time.sleep(1.0)
            return True
        else:
            self.get_logger().error('Gripper action failed!')
            return False

    def pick_block(self):
        import time
        
        approach_angles = [2.2, 1.6, -0.0, 0.9, -1.05, 2.3]  # Position for opening gripper
        pick_angles = [2.2, 1.73, -0.0, 0.9, -1.05, 2.3]    # Position for closing gripper
        
        self.get_logger().info(f'Step 1: Opening gripper')
        self.control_gripper(0.8)  
        time.sleep(2.0)
        
        self.get_logger().info('Step 2: Moving to approach position')
        self.arm_control.move_to_joint_positions(approach_angles)
        time.sleep(5.0)  
        
        self.get_logger().info('Step 3: Moving to pick position')
        self.arm_control.move_to_joint_positions(pick_angles)
        time.sleep(5.0)  
        
        self.get_logger().info('Step 4: Closing gripper')
        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = 0.0  
        goal_msg.command.max_effort = 100.0  
        future = self.gripper_action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()
        if goal_handle.accepted:
            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future)
            result = result_future.result()
            if result:
                self.get_logger().info('Gripper closed')
        
        self.get_logger().info('Step 5: Waiting for secure grip')
        time.sleep(3.0) 
        
        self.get_logger().info('Step 6: Lift')
        lift_angles = [2.2, 1.6, -0.0, 0.9, -1.05, 2.3] 
        intermediate_angles = [
            [2.22, 1.65, -0.0, 0.9, -1.05, 2.3],  
            [2.21, 1.63, -0.0, 0.9, -1.05, 2.3],  
            lift_angles  
        ]
        
        for i, angles in enumerate(intermediate_angles):
            self.arm_control.move_to_joint_positions(angles)
            time.sleep(3.0) 
        
        self.get_logger().info('Step 7: Moving to storage bin position')
        storage_intermediate_angles = [
            [0.0, 1.75, -0.0, 0.9, 0.0, 2.3],    
            [-0.75, 1.75, -0.0, 0.9, 0.0, 2.3],  
            self.storage_basket_angles           
        ]
        
        for i, angles in enumerate(storage_intermediate_angles):
            self.arm_control.move_to_joint_positions(angles)
            time.sleep(3.0)  
        
        self.get_logger().info('Step 8: Releasing block in storage bin')
        self.control_gripper(0.8)  
        time.sleep(2.0)
        
        self.get_logger().info('Step 9: Moving back to home position')
        return_intermediate_angles = [
            [-0.75, 1.75, -0.0, 0.9, 0.0, 2.3],  
            [0.0, 1.75, -0.0, 0.9, 0.0, 2.3],    
            self.home_position                    
        ]
        
        for i, angles in enumerate(return_intermediate_angles):
            self.arm_control.move_to_joint_positions(angles)
            time.sleep(3.0) 
        
        self.get_logger().info(f'Completed pick and place sequence to storage bin')

def main(args=None):
    rclpy.init(args=args)
    node = BlockPickPlace()
    node.pick_block()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

