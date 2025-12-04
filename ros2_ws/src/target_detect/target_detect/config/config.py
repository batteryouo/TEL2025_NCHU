import os
import yaml

import numpy as np
def load_config():
    script_dir = os.path.dirname(os.path.realpath(__file__))
    config_path = os.path.join(script_dir, 'config.yaml')
    config = {}
    returnDict = {}
    try:
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)
    except Exception:
        print("Loading file fails. Using Default Value")

    nav_cfg = config.get('lidar_nav', {})
    returnDict["TARGET_DIST"] = nav_cfg.get('target_dist', 3)
    
    det = nav_cfg.get('detection', {})
    returnDict["MAX_RANGE"] = det.get('max_range', 10.0)
    returnDict["angle_deg"] = det.get('roi_angle_deg', 25.0)
    returnDict["ROI_ANGLE_LIMIT"] = np.radians(returnDict["angle_deg"])
    
    bbox = det.get('filter_box', {})
    returnDict["BODY_X_MIN"] = bbox.get('x_min', -0.6)
    returnDict["BODY_X_MAX"] = bbox.get('x_max', 0.1)
    returnDict["BODY_Y_MIN"] = bbox.get('y_min', -0.3)
    returnDict["BODY_Y_MAX"] = bbox.get('y_max', 0.3)

    cluster = nav_cfg.get('cluster', {})
    returnDict["CLUSTER_TOLERANCE"] = cluster.get('tolerance', 0.15)
    returnDict["MIN_CLUSTER_SIZE"] = cluster.get('min_size', 5)

    track = nav_cfg.get('tracking', {})
    returnDict["TRACKING_TOLERANCE"] = track.get('tolerance', 0.5)

    returnDict["SMOOTH_WINDOW"] = 5
    returnDict["STOP_TOLERANCE"] = 0.05
    returnDict["MIN_MOVE_SPEED"] = 0.08
    returnDict["MIN_ROT_SPEED"] = 0.1

    avoid = nav_cfg.get('avoidance', {})
    returnDict["AVOID_ENABLE"] = avoid.get('enable', True)
    returnDict["AVOID_STOP_DIST"] = avoid.get('stop_dist', 0.8)

    pid = nav_cfg.get('pid', {})
    returnDict["KP_ANGULAR"] = pid.get('kp_angular', 1.2)
    returnDict["KP_LINEAR"] = pid.get('kp_linear', 1.0)
    returnDict["MAX_SPEED"] = pid.get('max_speed', 5.0)
    returnDict["SEARCH_ROTATION_SPEED"] = pid.get('search_rot_speed', 0.0)

    cam_cfg = config.get('cam_param', {})
    returnDict["vCenter"] = cam_cfg.get('vCenter', 610)

    return returnDict