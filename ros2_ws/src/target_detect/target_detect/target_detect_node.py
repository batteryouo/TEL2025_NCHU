import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image # For both color and depth images
from cv_bridge import CvBridge # To convert ROS Image to OpenCV image (Cv2 format)

# Import the synchronization tools
import message_filters

import cv2
import numpy as np


cv2.namedWindow("img")
# cv2.setMouseCallback("img", mouse_callback)

class TargetDetectNode(Node):
    def __init__(self):
        super().__init__('target_detect_node')
        self.bridge = CvBridge()
        
        # 1. Create Subscribers
        self.color_sub = message_filters.Subscriber(
            self, 
            Image, 
            '/camera/camera/color/image_raw'
        )
        self.depth_sub = message_filters.Subscriber(
            self, 
            Image, 
            '/camera/camera/depth/image_rect_raw'
        )

        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.color_sub, self.depth_sub], 
            10, # Queue size
            0.05, # Slop (in seconds)
            allow_headerless=True
        )
        
        # 3. Register the callback
        self.ts.registerCallback(self.image_sync_callback)
        self.get_logger().info('Synchronized subscribers started.')

        self.frameIdx = 0

    def image_sync_callback(self, color_msg, depth_msg):
        """
        This callback is only triggered when a matching pair of color and depth
        messages are received within the 0.05 second time window (slop).
        """
        # self.get_logger().info('Received synchronized color and depth images.')
        
        # --- Processing Logic Starts Here ---
        try:
            # Convert ROS Image messages to OpenCV format
            cv_color_image = self.bridge.imgmsg_to_cv2(color_msg, 'bgr8')
            cv_depth_image = self.bridge.imgmsg_to_cv2(depth_msg, '16UC1') # or appropriate depth encoding

          
            cv2.imshow("img", cv_color_image)
            # print(cv_depth_image[y_global, x_global])
            # self.get_logger().info(f"depth[{y_global}, {x_global}] = {cv_depth_image[y_global, x_global]}")
            cv2.waitKey(30)
         
        except Exception as e:
            self.get_logger().error(f'Error converting images: {e}')


def main(args=None):
    rclpy.init(args=args)

    target_detect_node = TargetDetectNode()
    rclpy.spin(target_detect_node)

    target_detect_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()