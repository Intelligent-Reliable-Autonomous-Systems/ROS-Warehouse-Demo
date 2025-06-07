#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from tf2_ros import TransformListener, Buffer, TransformException
from camera_detect_msgs.srv import GetAprilTagTransform
import rclpy.time

class DetectionManager(Node):
    def __init__(self):
        super().__init__('detection_manager')
        
        # Initialize tf2 buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Service to get transform from kinova/base_link to specific AprilTag
        self.get_transform_service = self.create_service(
            GetAprilTagTransform,
            'get_apriltag_transform',
            self.get_apriltag_transform_callback
        )
        
        self.get_logger().info('Detection Manager initialized - providing AprilTag transform service')
    
    def get_apriltag_transform_callback(self, request, response):
        """Service callback to get transform from kinova/base_link to specific AprilTag"""
        try:
            tag_id = request.tag_id
            tag_frame = f"tag36h11:{tag_id}"
            
            self.get_logger().info(f"Requesting transform for AprilTag {tag_id}")
            
            try:
                # Get transform from kinova/base_link to the specified tag
                transform = self.tf_buffer.lookup_transform(
                    "kinova/base_link",
                    tag_frame,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=2.0)
                )
                
                response.success = True
                response.message = f"Transform found for AprilTag {tag_id}"
                response.transform = transform
                
                # Log the transform info
                trans = transform.transform.translation
                rot = transform.transform.rotation
                self.get_logger().info(
                    f"Tag {tag_id} transform: "
                    f"pos=({trans.x:.3f}, {trans.y:.3f}, {trans.z:.3f}), "
                    f"rot=({rot.x:.3f}, {rot.y:.3f}, {rot.z:.3f}, {rot.w:.3f})"
                )
                
            except TransformException as e:
                response.success = False
                response.message = f"Could not get transform for AprilTag {tag_id}: {str(e)}"
                self.get_logger().warn(response.message)
                
        except Exception as e:
            response.success = False
            response.message = f"Error getting transform: {str(e)}"
            self.get_logger().error(f"Error in get_apriltag_transform_callback: {e}")
            
        return response

def main(args=None):
    rclpy.init(args=args)
    
    detection_manager = DetectionManager()
    
    # Use MultiThreadedExecutor for better performance with tf2
    executor = MultiThreadedExecutor()
    executor.add_node(detection_manager)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        detection_manager.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
