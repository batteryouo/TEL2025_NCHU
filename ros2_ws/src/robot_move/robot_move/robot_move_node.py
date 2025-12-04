#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import math
from collections import deque
import yaml
import os

from communicate_msg.msg import Int32
try:
    from communicate_msg.msg import Mecanum
except ImportError:
    class Mecanum:
        pass

class LidarFullBoardAlign(Node):
    def __init__(self):
        super().__init__("lidar_full_board_align")
        
        self.load_config()

        # 狀態變數
        self.is_locked = False       
        self.is_parked = False
        self.reach = False           # [新增] 到達標誌 (距離小於 3m 為 True)
        self.last_cx = 0.0           
        self.last_cy = 0.0           

        self.history_cx = deque(maxlen=self.SMOOTH_WINDOW)
        self.history_cy = deque(maxlen=self.SMOOTH_WINDOW)
        self.history_dist = deque(maxlen=self.SMOOTH_WINDOW)

        # Matplotlib 視覺化初始化
        self.fig, self.ax = plt.subplots()
        plt.ion() 
        self.fig.canvas.manager.set_window_title('全向避障 (最遠目標版)')

        self.ax.set_xlim(-4.0, 6.0) 
        self.ax.set_ylim(-0.0, 10.0)
        self.ax.set_aspect('equal')
        self.ax.grid(True, linestyle=':', alpha=0.6)
        
        self.search_area_line, = self.ax.plot([], [], 'k--', linewidth=1, alpha=0.5, label='ROI')
        self.body_rect = patches.Rectangle((0,0), 0, 0, linewidth=1, edgecolor='r', facecolor='gray', alpha=0.3, label='Filtered Body')
        self.ax.add_patch(self.body_rect)

        self.scat_noise = self.ax.scatter([], [], s=2, c='gray', alpha=0.2) 
        self.scat_target = self.ax.scatter([], [], s=15, c='red', alpha=1.0, label='Target') 
        self.scat_obstacles = self.ax.scatter([], [], s=15, c='black', alpha=1.0, label='Obstacles')
        
        self.center_line, = self.ax.plot([], [], 'b-', linewidth=2, label='Aim Line')
        self.fit_line, = self.ax.plot([], [], 'y--', linewidth=1)
        self.track_circle, = self.ax.plot([], [], 'c--', linewidth=2)
        self.avoid_circle, = self.ax.plot([], [], 'r:', linewidth=1, label='Stop Zone')

        self.title_text = self.ax.set_title("Initializing...", fontweight='bold')
        self.ax.legend(loc='upper right', fontsize='small')

        lidar_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=1)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_profile=lidar_qos)
        self.create_subscription(Int32, '/mode', self.mode_callback, 10)
        
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.move_pub = self.create_publisher(Mecanum, '/move', 10)

        self.get_logger().info(f"系統啟動！模式：只選擇距離最遠的目標")
        self.mode = 0

    def load_config(self):
        script_dir = os.path.dirname(os.path.realpath(__file__))
        config_path = os.path.join(script_dir, 'lidar_config.yaml')
        config = {}
        try:
            with open(config_path, 'r') as f:
                config = yaml.safe_load(f)
        except Exception:
            self.get_logger().warn("設定檔讀取失敗，使用預設值")

        nav_cfg = config.get('lidar_nav', {})
        self.TARGET_DIST = nav_cfg.get('target_dist', 0.5)

        det = nav_cfg.get('detection', {})
        self.MAX_RANGE = det.get('max_range', 6.0)
        angle_deg = det.get('roi_angle_deg', 45.0)
        self.ROI_ANGLE_LIMIT = np.radians(angle_deg)
        
        bbox = det.get('filter_box', {})
        self.BODY_X_MIN = bbox.get('x_min', -0.6)
        self.BODY_X_MAX = bbox.get('x_max', 0.1)
        self.BODY_Y_MIN = bbox.get('y_min', -0.3)
        self.BODY_Y_MAX = bbox.get('y_max', 0.3)

        cluster = nav_cfg.get('cluster', {})
        self.CLUSTER_TOLERANCE = cluster.get('tolerance', 0.15)
        self.MIN_CLUSTER_SIZE = cluster.get('min_size', 5)

        track = nav_cfg.get('tracking', {})
        self.TRACKING_TOLERANCE = track.get('tolerance', 0.5)

        self.SMOOTH_WINDOW = 5 
        self.STOP_TOLERANCE = 0.05
        self.MIN_MOVE_SPEED = 0.08
        self.MIN_ROT_SPEED = 0.1

        avoid = nav_cfg.get('avoidance', {})
        self.AVOID_ENABLE = avoid.get('enable', True)
        self.AVOID_STOP_DIST = avoid.get('stop_dist', 0.8)

        pid = nav_cfg.get('pid', {})
        self.KP_ANGULAR = pid.get('kp_angular', 1.2)
        self.KP_LINEAR = pid.get('kp_linear', 1.0)
        self.MAX_SPEED = pid.get('max_speed', 1.0)
        self.SEARCH_ROTATION_SPEED = pid.get('search_rot_speed', 0.3)
 

    def scan_callback(self, msg: LaserScan):
        if self.reach:
            print("QI")
        # 1. 資料轉換
        ranges = np.array(msg.ranges)
        angles = np.arange(msg.angle_min, msg.angle_max, msg.angle_increment)
        min_len = min(len(ranges), len(angles))
        ranges = ranges[:min_len]
        angles = angles[:min_len]
        angles = np.where(angles > np.pi, angles - 2*np.pi, angles)

        raw_x = ranges * np.cos(angles)
        raw_y = ranges * np.sin(angles)

        # 2. 車身濾除
        in_box_x = (raw_x > self.BODY_X_MIN) & (raw_x < self.BODY_X_MAX)
        in_box_y = (raw_y > self.BODY_Y_MIN) & (raw_y < self.BODY_Y_MAX)
        is_body = in_box_x & in_box_y
        valid_mask = ~is_body & np.isfinite(ranges)
        
        valid_ranges = ranges[valid_mask]
        valid_angles = angles[valid_mask]
        all_x = raw_x[valid_mask]
        all_y = raw_y[valid_mask]

        # 3. ROI (尋找板子用)
        mask_roi = (np.abs(valid_angles) <= self.ROI_ANGLE_LIMIT) & \
                   (valid_ranges < self.MAX_RANGE)
        
        roi_x = all_x[mask_roi]
        roi_y = all_y[mask_roi]

        # 4. 尋找板子
        clusters = self.find_clusters(roi_x, roi_y)
        target_cluster = None
        max_score = -float('inf') 
        
        best_fit_m = 0
        best_fit_c = 0

        for cluster in clusters:
            points_count = len(cluster['x'])
            if points_count < self.MIN_CLUSTER_SIZE: continue
            
            ys = np.array(cluster['y'])
            xs = np.array(cluster['x'])
            if np.max(ys) - np.min(ys) < 0.1: continue

            curr_cx = np.mean(xs)
            curr_cy = np.mean(ys)
            
            # 鎖定檢查
            if self.is_locked:
                dist_jump = math.sqrt((curr_cx - self.last_cx)**2 + (curr_cy - self.last_cy)**2)
                if dist_jump > self.TRACKING_TOLERANCE: continue

            # [修改] 極簡評分：只看平均距離 (越遠分越高)
            avg_dist = np.mean(xs)
            score = avg_dist 

            if score > max_score:
                max_score = score
                target_cluster = cluster
                # 雖然不列入評分，但為了畫圖還是算一下擬合線
                try:
                    m, c = np.polyfit(ys, xs, 1)
                    best_fit_m = m
                    best_fit_c = c
                except:
                    pass

        cmd_vx = 0.0
        cmd_vy = 0.0
        cmd_w = 0.0
        state = "SEARCHING"
        
        vis_cx, vis_cy = 0, 0
        has_target = False
        blocking_obs = []

        if target_cluster is not None:
            state = "LOCKED" if self.is_locked else "ACQUIRING"
            self.is_locked = True
            has_target = True
            
            ys = np.array(target_cluster['y'])
            xs = np.array(target_cluster['x'])
            
            min_idx = np.argmin(xs)
            raw_center_x = xs[min_idx] # 最近點
            
            geo_cy = np.mean(ys)
            self.last_cx = np.mean(xs)
            self.last_cy = geo_cy

            self.history_cx.append(raw_center_x)
            self.history_cy.append(geo_cy)
            self.history_dist.append(raw_center_x)

            avg_min_dist = sum(self.history_dist) / len(self.history_dist)
            avg_cy = sum(self.history_cy) / len(self.history_cy) 

            # [新增] 判斷是否進入 3 公尺範圍
            if avg_min_dist < 3.0:
                self.reach = True
            else:
                self.reach = False

            vis_cx, vis_cy = avg_min_dist, avg_cy

            # 避障檢查
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
                        blocking_obs = np.column_stack((obs_y_near[real_obs_mask], obs_x_near[real_obs_mask]))

            if is_blocked:
                state = "BLOCKED (WAITING)"
                cmd_vx = 0.0
                cmd_vy = 0.0
                cmd_w = 0.0
            else:
                if self.is_parked:
                    state = "PARKED"
                    if abs(avg_min_dist - self.TARGET_DIST) > (self.STOP_TOLERANCE + 0.05):
                        self.is_parked = False
                else:
                    if abs(avg_cy) > self.STOP_TOLERANCE:
                        rot_speed = float(avg_cy * self.KP_ANGULAR)
                        if abs(rot_speed) < self.MIN_ROT_SPEED:
                            rot_speed = math.copysign(self.MIN_ROT_SPEED, rot_speed)
                        cmd_w = rot_speed

                    err_dist = avg_min_dist - self.TARGET_DIST
                    if abs(err_dist) > self.STOP_TOLERANCE:
                        speed = err_dist * self.KP_LINEAR
                        cmd_vx = min(max(speed, -self.MAX_SPEED), self.MAX_SPEED)
                        if abs(cmd_vx) < self.MIN_MOVE_SPEED and abs(cmd_vx) > 0.01:
                            cmd_vx = math.copysign(self.MIN_MOVE_SPEED, cmd_vx)
                    else:
                        cmd_vx = 0.0
                        if abs(cmd_w) < 0.01: 
                            self.is_parked = True
                            state = "PARKED"
            
            cmd_vy = 0.0

        else:
            self.is_locked = False
            self.is_parked = False
            self.reach = False # 目標遺失時重置
            cmd_vx = 0.0
            cmd_vy = 0.0
            cmd_w = self.SEARCH_ROTATION_SPEED
            self.history_dist.clear()

        cmd = Twist()

        try:
            if self.mode == 2:
                mec = Mecanum()
                mec.speed = float(cmd_vx) 
                if cmd_vx < 0:
                    cmd_vx = 0
                mec.angle = 0.0 
                mec.speed = float(cmd_vx)
                mec.w = float(cmd_w)
                self.move_pub.publish(mec)
        except NameError:
            pass

        self.update_plot(all_x, all_y, target_cluster, blocking_obs, has_target, 
                         vis_cx, vis_cy, state, cmd, max_score,
                         best_fit_m, best_fit_c)
    
    def mode_callback(self, msg):
        self.mode = msg.data

    def find_clusters(self, x_arr, y_arr):
        if len(x_arr) == 0: return []
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

    def update_plot(self, all_x, all_y, target_cluster, blocking_obs, has_target, 
                    cx, cy, state, cmd, score_val,
                    fit_m, fit_c):
        
        if len(all_x) > 0:
            self.scat_noise.set_offsets(np.column_stack((all_y, all_x)))
        else:
            self.scat_noise.set_offsets(np.empty((0, 2)))

        limit_dist = self.MAX_RANGE
        limit_angle = self.ROI_ANGLE_LIMIT
        lx = limit_dist * np.sin(limit_angle)
        ly = limit_dist * np.cos(limit_angle)
        rx = limit_dist * np.sin(-limit_angle)
        ry = limit_dist * np.cos(-limit_angle)
        self.search_area_line.set_data([lx, 0, rx], [ly, 0, ry])

        rect_w = self.BODY_Y_MAX - self.BODY_Y_MIN
        rect_h = self.BODY_X_MAX - self.BODY_X_MIN
        self.body_rect.set_xy((self.BODY_Y_MIN, self.BODY_X_MIN))
        self.body_rect.set_width(rect_w)
        self.body_rect.set_height(rect_h)

        stop_angle = np.pi / 4
        theta = np.linspace(-stop_angle, stop_angle, 30)
        c_x = self.AVOID_STOP_DIST * np.sin(theta)
        c_y = self.AVOID_STOP_DIST * np.cos(theta)
        c_x = np.concatenate(([0], c_x, [0]))
        c_y = np.concatenate(([0], c_y, [0]))
        self.avoid_circle.set_data(c_x, c_y)

        if len(blocking_obs) > 0:
             self.scat_obstacles.set_offsets(np.column_stack((blocking_obs[:,0], blocking_obs[:,1]))) 
        else:
             self.scat_obstacles.set_offsets(np.empty((0, 2)))

        if has_target:
            self.scat_target.set_offsets(np.column_stack((target_cluster['y'], target_cluster['x'])))
            self.center_line.set_data([0, cy], [0, cx])
            
            fit_y = np.linspace(min(target_cluster['y']), max(target_cluster['y']), 10) 
            fit_x = fit_m * fit_y + fit_c
            self.fit_line.set_data(fit_y, fit_x)
            
            if self.is_locked:
                theta = np.linspace(0, 2*np.pi, 30)
                circle_x = self.last_cy + self.TRACKING_TOLERANCE * np.cos(theta)
                circle_y = self.last_cx + self.TRACKING_TOLERANCE * np.sin(theta)
                self.track_circle.set_data(circle_x, circle_y)
            else:
                self.track_circle.set_data([], [])

            color = 'blue'
            if "PARKED" in state: color = 'green'
            elif "BLOCKED" in state: color = 'red'
            
            # [修改] 顯示 reach 狀態
            reach_str = "REACHED" if self.reach else "APPROACHING"
            info = f"D:{cx:.2f}m ErrY:{cy:.2f}m [{reach_str}]"
        else:
            self.scat_target.set_offsets(np.empty((0, 2)))
            self.center_line.set_data([], [])
            self.fit_line.set_data([], [])
            self.track_circle.set_data([], [])
            
            color = 'orange'
            info = "Scanning..."

        self.title_text.set_text(f"Mode: {state}\n{info}\nCmd X:{cmd.linear.x:.2f} W:{cmd.angular.z:.2f}")
        self.title_text.set_color(color)
        plt.pause(0.001)

def main(args=None):
    rclpy.init(args=args)
    node = LidarFullBoardAlign()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        stop_cmd = Twist()
        node.pub_cmd.publish(stop_cmd)
        try:
            from communicate_msg.msg import Mecanum
            stop_mec = Mecanum()
            stop_mec.speed = 0.0
            node.move_pub.publish(stop_mec)
        except:
            pass
        node.get_logger().info("Stopping robot...")
    finally:
        plt.close()
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()