import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, LaserScan # For both color and depth images
from communicate_msg.msg import Imu, Int32, Mecanum
from cv_bridge import CvBridge # To convert ROS Image to OpenCV image (Cv2 format)

import math
import os
import time

import cv2
from collections import deque
import numpy as np
import torch

from ultralytics import YOLO

from target_detect.config.config import load_config


cv2.namedWindow("img", 0)
cv2.namedWindow("yolo", 0)
# cv2.setMouseCallback("img", mouse_callback)

class PID():
    def __init__(self, p, i, d, max_i):
        self.p = p
        self.i = i
        self.d = d
        self.max_i = abs(max_i)

        self.lastError = 0
        self.cumError = 0
    
    def compute(self, err):
        d_err = err-self.lastError
        self.lastError = err
        self.cumError += err
        self.check()

        return self.p*err + self.i*self.cumError + self.d*d_err
        
    
    def check(self):
        if abs(self.cumError) > self.max_i:
            self.cumError = self.cumError/abs(self.cumError) * self.max_i
        

class TargetDetectNode(Node):
    def __init__(self):

        file_dir = os.path.dirname(os.path.abspath(__file__))
        model_path = os.path.join(file_dir, "model", "weights", "best.pt")
        self.model = YOLO(model_path)
        super().__init__('target_detect_node')
        self.bridge = CvBridge()

        self.move_pub = self.create_publisher(Mecanum, 'move', 10)
        self.angle_pub = self.create_publisher(Imu, 'auto_launch_angle', 10)
        self.launch_pub = self.create_publisher(Int32, 'auto_launch', 10)
        
        self.img_sub = self.create_subscription(Image, 'camera/camera/color/image_raw', self.image_callback, 10)
        self.imu_sub = self.create_subscription(Imu, 'launch_imu', self.launch_imu_callback, 10)
        self.mode_sub = self.create_subscription(Int32, 'mode', self.mode_callback, 10)
        self.launch_sub = self.create_subscription(Int32, 'launch', self.launch_callback, 10)
        self.start_sub = self.create_subscription(Int32, 'start', self.start_callback, 10)
        lidar_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=1)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_profile=lidar_qos)

        self.get_logger().info('Synchronized subscribers started.')
        
        
        # control data
        self.angle = 0.5
        self.mode = 0
        self.launch = 1
        self.frameIdx = 0
        self.start = 0
        # lidar info
        self.reach = False

        self.launchCount = 0

        self.launch_order = {0: (0, 2), 1: (0, 2), 2: (0, 1), 3: (0, 1), 4: (0, 0), 5: (0, 0),
                             6: (1, 0), 7: (1, 0), 8: (1, 1), 9: (1, 1), 10: (1, 2), 11: (1, 2),
                             12: (2, 2), 13: (2, 2), 14: (2, 1), 15: (2, 1), 16: (2, 0), 17: (2, 0)}

        self.angleDict = {2: 0.18, 1: 0.35, 0: 0.55}
        self.launch_flag = True
        self.orderInc = 1
        self.initialIndexFlag = False

        self.startPID = PID(1.2, 0.05, 0.02, 0.2)
        self.movePID = PID(0.01, 0.001, 0.001, 0.2)
        self.rotPID = PID(0.01, 0.001, 0.001, 0.1)
        cfg = load_config()
        self.startPID.p = cfg["KP_LINEAR"]
        self.BODY_X_MIN = cfg["BODY_X_MIN"]
        self.BODY_X_MAX = cfg["BODY_X_MAX"]
        self.BODY_Y_MIN = cfg["BODY_Y_MIN"]
        self.BODY_Y_MAX = cfg["BODY_Y_MAX"]
        self.ROI_ANGLE_LIMIT = cfg["ROI_ANGLE_LIMIT"]
        self.MAX_RANGE = cfg["MAX_RANGE"]
        self.CLUSTER_TOLERANCE = cfg["CLUSTER_TOLERANCE"]
        self.MIN_CLUSTER_SIZE = cfg["MIN_CLUSTER_SIZE"]
        self.AVOID_ENABLE = cfg["AVOID_ENABLE"]
        self.AVOID_STOP_DIST = cfg["AVOID_STOP_DIST"]
        self.TARGET_DIST = cfg["TARGET_DIST"]
        self.MIN_MOVE_SPEED = cfg["MIN_MOVE_SPEED"]
        self.MAX_SPEED = cfg["MAX_SPEED"]
        self.vCenter = cfg["vCenter"]
        self.is_locked = False

        self.history_dist = deque(maxlen=5)

        print(cfg)
        
    def image_callback(self, msg):
        cv_color_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        # cv_color_image = cv2.imread(f"/home/ee512/realsenseRawfile/image_rect_raw1/{str(self.frameIdx).zfill(10)}.jpg")
        # if self.frameIdx > 1500:
        #     self.frameIdx = 0

        # self.frameIdx += 1
        results = self.model.predict(cv_color_image, conf=0.5, verbose=False)
        predict_image = results[0].plot()
        label_dict = results[0].names
        line_v_pts = [[self.vCenter, 0], [self.vCenter, cv_color_image.shape[0]]]
        cv2.line(cv_color_image, line_v_pts[0], line_v_pts[1], (0, 0, 255), 5, cv2.LINE_AA)        

        holes_list = []
        for result in results[0]:
            box = result.boxes.cpu().numpy()
            box_xywh = box.xywh[0]
            cls = box.cls[0]
            
            if label_dict[cls] == "board":
                continue
            else:
                holes_list.append(box_xywh[:2])

        if len(holes_list) < 3:
            cv2.imshow("img", cv_color_image)
            cv2.imshow("yolo", predict_image)            
            cv2.waitKey(1)
            return
        # self.reach = True 
        if not self.reach:
            print("waiting reach")
            return
        
        targetGroups = self.yoloClustering(holes_list)
        meanOfGroups = []
        for group in targetGroups:
            meanOfGroups.append(np.mean(group, axis=0)[0])

        gaps = np.diff(meanOfGroups)
        rot_err = gaps[0] - gaps[1]
        # w_compensate = self.rotPID.compute(rot_err)
        detect_correct_flag = np.all(gaps > 100)

        if not self.initialIndexFlag:
            self.initialIndexFlag = True
            if abs(self.vCenter - meanOfGroups[0]) <= abs(self.vCenter - meanOfGroups[1]):
                self.orderInc = 1
                self.launchCount = 0
            else:
                self.orderInc = -1
                self.launchCount = 17
            print(self.orderInc, self.launchCount)
            # time.sleep(10)

        launch_order = self.launch_order[self.launchCount]
        targetGroup = targetGroups[launch_order[0]]
        meanOfMid = np.mean(targetGroup, axis=0)

        diffOfMid = (self.vCenter - meanOfMid[0]) 
        # diffOfMid = 0
        v = self.movePID.compute(diffOfMid)
        targetAngle = self.angleDict[launch_order[1]]
        if self.mode == 2 and self.launch_flag and self.start == 1:
            mec = Mecanum(speed = v, angle = np.pi/2, w = 0.0)            
            self.move_pub.publish(mec)

            angle_data = Imu(y = targetAngle, p = 0.0, r =0.0)
            self.angle_pub.publish(angle_data)
            print(f"targetAngle: {targetAngle}, angle: {self.angle}")
         
        aimReady = abs(self.angle - targetAngle) < 0.025 and abs(diffOfMid) < 40 and self.launch_flag and detect_correct_flag #and abs(rot_err) < 5
        if self.mode == 2 and self.start == 1 and aimReady:
            launch_data = Int32(data=1)
            self.launch_pub.publish(launch_data)
            self.launch_flag = False
            self.launchCount += self.orderInc
        elif self.launch == 0 and not self.launch_flag:
            self.launch_flag = True
            time.sleep(3)            

        if self.launchCount > 17:
            self.launchCount = 0
        if self.launchCount < 0:
            self.launchCount = 17

        line_v_pts = [[self.vCenter, 0], [self.vCenter, cv_color_image.shape[0]]]
        cv2.line(cv_color_image, line_v_pts[0], line_v_pts[1], (0, 0, 255), 5, cv2.LINE_AA)
  
        cv2.imshow("img", cv_color_image)
        cv2.imshow("yolo", predict_image)
        key = cv2.waitKey(1)

    def scan_callback(self, msg: LaserScan):
        if self.reach:
            return
        
        if self.mode == 0 or self.start == 0:
            return


        ranges = np.array(msg.ranges)
        angles = np.arange(msg.angle_min, msg.angle_max, msg.angle_increment)
        min_len = min(len(ranges), len(angles))
        ranges = ranges[:min_len]
        angles = angles[:min_len]
        angles = np.where(angles > np.pi, angles - 2*np.pi, angles)    

        raw_x = ranges * np.cos(angles)
        raw_y = ranges * np.sin(angles)

        # filter body
        in_box_x = (raw_x > self.BODY_X_MIN) & (raw_x < self.BODY_X_MAX)
        in_box_y = (raw_y > self.BODY_Y_MIN) & (raw_y < self.BODY_Y_MAX)
        is_body = in_box_x & in_box_y
        valid_mask = ~is_body & np.isfinite(ranges)
        valid_ranges = ranges[valid_mask]
        valid_angles = angles[valid_mask]

        all_x = raw_x[valid_mask]
        all_y = raw_y[valid_mask]
        # 3. ROI
        mask_roi = (np.abs(valid_angles) <= self.ROI_ANGLE_LIMIT) & (valid_ranges < self.MAX_RANGE)
        roi_x = all_x[mask_roi]
        roi_y = all_y[mask_roi]

        clusters = self.find_clusters(roi_x, roi_y)
        target_cluster = None
        max_score = -float('inf') 
        for cluster in clusters:
            points_count = len(cluster['x'])
            if points_count < self.MIN_CLUSTER_SIZE:
                continue
            
            ys = np.array(cluster['y'])
            xs = np.array(cluster['x'])
            if np.max(ys) - np.min(ys) < 0.1:
                continue

            avg_dist = np.mean(xs)
            score = avg_dist 

            if score > max_score:
                max_score = score
                target_cluster = cluster

        if target_cluster is not None:
            ys = np.array(target_cluster['y'])
            xs = np.array(target_cluster['x'])

            min_idx = np.argmin(xs)
            raw_center_x = xs[min_idx]
            print(raw_center_x)
            self.history_dist.append(raw_center_x)
            avg_min_dist = sum(self.history_dist) / len(self.history_dist)

            if avg_min_dist < self.TARGET_DIST:
                self.reach = True
                
            # else:
            #     self.reach = False
            
            is_blocked = False
            if self.AVOID_ENABLE:
                dist_mask = (valid_ranges < self.AVOID_STOP_DIST)
                angle_mask = (np.abs(valid_angles) <= (np.pi / 4))
                obs_mask = dist_mask & angle_mask
                
                obs_x_near = all_x[obs_mask]
                obs_y_near = all_y[obs_mask]
                
                if len(obs_x_near) > 0:
                    real_obs_mask = obs_x_near < (avg_min_dist - 0.2)
                    if np.any(real_obs_mask):
                        is_blocked = True

            if is_blocked:
                cmd_vx = 0.0
                cmd_vy = 0.0
                cmd_w = 0.0
            else:
                err_dist = avg_min_dist - self.TARGET_DIST
                speed = self.startPID.compute(err_dist)
                cmd_vx = min(max(speed, -self.MAX_SPEED), self.MAX_SPEED)
                if abs(cmd_vx) < self.MIN_MOVE_SPEED and abs(cmd_vx) > 0.01:
                    cmd_vx = math.copysign(self.MIN_MOVE_SPEED, cmd_vx)
        
        else:
            cmd_vx = 0.0
            cmd_vy = 0.0
            cmd_w = 0.0
            self.history_dist.clear()
        mec = Mecanum()
        mec.speed = float(cmd_vx) 
        if cmd_vx < 0:
            cmd_vx = 0
        mec.angle = 0.0 
        mec.speed = float(cmd_vx)
        mec.w = float(0.0)
        self.move_pub.publish(mec)


    def launch_imu_callback(self, msg:Imu):
        angle = msg.y
        if angle<0:
            angle = 0.0
        if angle>1:
            angle = 1.0
        
        self.angle = angle

    def mode_callback(self, msg:Int32):
        mode = msg.data
        self.mode = mode
    
    def launch_callback(self, msg:Int32):
        self.launch = msg.data
    
    def start_callback(self, msg:Int32):
        self.start = msg.data
    
    def yoloClustering(self, holes_list):

        holes_sorted = sorted(holes_list, key=lambda p: p[0])
        gaps = []
        for i in range(len(holes_sorted) - 1):
            gap_value = holes_sorted[i+1][0] - holes_sorted[i][0]
            gaps.append((gap_value, i))
        gaps.sort(key=lambda x: x[0], reverse=True)
        top_2_gaps = [gaps[0], gaps[1]]
        split_indices = sorted([g[1] for g in top_2_gaps])
        idx1 = split_indices[0] 
        idx2 = split_indices[1] 

        left_group = holes_sorted[ : idx1 + 1]
        mid_group = holes_sorted[idx1 + 1 : idx2 + 1]
        right_group = holes_sorted[idx2 + 1 : ]

        targetGroups = [left_group, mid_group, right_group]

        return targetGroups
    
    def find_clusters(self, x_arr, y_arr):
        if len(x_arr) == 0:
            return []
        angles = np.arctan2(y_arr, x_arr)
        sorted_indices = np.argsort(angles) 
        x_sorted = x_arr[sorted_indices]
        y_sorted = y_arr[sorted_indices]
        clusters = []
        if len(x_sorted) > 0:
            current = {'x': [x_sorted[0]], 'y': [y_sorted[0]]}
            for i in range(1, len(x_sorted)):
                dist = math.sqrt((x_sorted[i] - x_sorted[i-1])**2 + (y_sorted[i] - y_sorted[i-1])**2)
                if dist < self.CLUSTER_TOLERANCE:
                    current['x'].append(x_sorted[i])
                    current['y'].append(y_sorted[i])
                else:
                    clusters.append(current)
                    current = {'x': [x_sorted[i]], 'y': [y_sorted[i]]}
            clusters.append(current)
        return clusters
    
def main(args=None):
    rclpy.init(args=args)

    target_detect_node = TargetDetectNode()
    rclpy.spin(target_detect_node)

    target_detect_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()