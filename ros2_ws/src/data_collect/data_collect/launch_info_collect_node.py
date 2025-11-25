import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image # For both color and depth images
from std_msgs.msg import Int32
# from sensor_msgs.msg import Imu
from communicate_msg.msg import Imu
from communicate_msg.msg import Int32
from cv_bridge import CvBridge # To convert ROS Image to OpenCV image (Cv2 format)

# Import the synchronization tools
import message_filters

import cv2
import numpy as np
cv2.namedWindow("img", 0)

class LaunchInfoCollectNode(Node):
    def __init__(self):
        super().__init__('launchInfo_collect_node')
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
        self.imu_sub = message_filters.Subscriber(
            self,
            Imu,
            '/launch_imu'
        )

        self.launch_sub = message_filters.Subscriber(
            self,
            Int32,
            "/launch"
        )

        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.color_sub, self.launch_sub, self.imu_sub], 
            # [self.color_sub, self.depth_sub, self.imu_sub], 
            10, # Queue size
            0.04, # Slop (in seconds)
            allow_headerless=True
        )
        
        # 3. Register the callback
        self.ts.registerCallback(self.sync_callback)
        self.get_logger().info('Synchronized subscribers started.')

        self.frame_idx = 0
        self.counter = 0

    def sync_callback(self, color_msg, launch_msg, imu_msg):
        """
        This callback is only triggered when a matching pair of color and depth
        messages are received within the 0.05 second time window (slop).
        """
        # self.get_logger().info('Received synchronized color and depth images.')
        
        # --- Processing Logic Starts Here ---
        try:
            # Convert ROS Image messages to OpenCV format
            cv_color_image = self.bridge.imgmsg_to_cv2(color_msg, 'bgr8')
            # cv_depth_image = self.bridge.imgmsg_to_cv2(depth_msg, '16UC1') # or appropriate depth encoding

            cv2.imshow("img", cv_color_image)
            key = cv2.waitKey(25)
            trigger_flag = launch_msg.data
            # if key != -1:
            #     self.counter = 60
            if trigger_flag == 1:
                self.counter = 10
            if self.counter != 0:
                cv2.imwrite(f"/home/ee512/realsenseRawfile/image_rect_raw/{str(self.frame_idx).zfill(10)}.jpg", cv_color_image)
                # np.save(f"/home/ee512/realsenseRawfile/depth/{str(self.frame_idx).zfill(10)}.npy", cv_depth_image)
                np.save(f"/home/ee512/realsenseRawfile/imu/{str(self.frame_idx).zfill(10)}.npy", np.array(imu_msg.y)) 
                self.get_logger().info(f"Frame {self.frame_idx} saves successfully. ")
                self.frame_idx += 1
                
                self.counter -= 1
        except Exception as e:
            self.get_logger().error(f'Error converting images: {e}')


def main(args=None):
    rclpy.init(args=args)

    launch_info_collect_node = LaunchInfoCollectNode()
    rclpy.spin(launch_info_collect_node)

    launch_info_collect_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
