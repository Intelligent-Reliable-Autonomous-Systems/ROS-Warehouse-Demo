import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

class GridNode(Node):
    """Add a transparent grid overlay to the camera feed."""
    def __init__(self):
        super().__init__('grid_overlay')
        self.br = CvBridge()
        self.sub = self.create_subscription(Image, '/camera/image_rect', self.callback, 1)
        self.pub = self.create_publisher(Image, '/camera/image_rect_grid', 1)
        # prepare semi‑transparent grid
        w, h = 640, 480
        self.grid = np.zeros((h, w, 3), np.uint8)
        for x in range(0, w, 80):
            cv2.line(self.grid, (x,0), (x,h), (0,255,0), 2)
        for y in range(0, h, 80):
            cv2.line(self.grid, (0,y), (w,y), (0,255,0), 2)

    def callback(self, msg):
        # Adds a transparent grid over the image
        img = self.br.imgmsg_to_cv2(msg, 'bgr8')
        overlay = cv2.addWeighted(img, 1.0, self.grid, 0.3, 0)
        self.pub.publish(self.br.cv2_to_imgmsg(overlay, 'bgr8'))

def main(args=None):
    rclpy.init(args=args)
    node = GridNode()
    rclpy.spin(node)
    rclpy.shutdown()
