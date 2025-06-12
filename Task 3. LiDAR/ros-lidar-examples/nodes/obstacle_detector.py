#!/usr/bin/env python3
# encoding: utf-8

import numpy as np
from typing import Optional, Tuple

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

class ObstacleDetector:
    def __init__(self):
        rospy.init_node('obstacle_detector')

        self.warning_threshold = 0.5
        self.fov_degrees = 30  
        self.stable_threshold = 0.01
        self.prev_distance = None
        self.prev_time = None
        
        self.movement_pub = rospy.Publisher('/obstacle_movement', String, queue_size=10)
        self.sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)

    def scan_callback(self, msg: LaserScan) -> None:
        ranges = np.array(msg.ranges)
        angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment
        frontal_ranges = self.get_frontal_sector(ranges, angles, msg.range_min, msg.range_max)
        
        if frontal_ranges.size == 0:
            return
        
        min_distance = np.min(frontal_ranges)
        current_time = rospy.Time.now().to_sec()
        if min_distance < self.warning_threshold:
            rospy.logwarn(f"WARNING! Obstacle too close: {min_distance:.2f}m")
        movement, speed = self.detect_movement(min_distance, current_time)
        self.publish_movement(movement, min_distance, speed)

    def get_frontal_sector(self, ranges: np.ndarray, angles: np.ndarray, 
                         range_min: float, range_max: float) -> np.ndarray:
        mask = np.abs(angles) <= np.radians(self.fov_degrees)
        valid_mask = (~np.isnan(ranges)) & (~np.isinf(ranges)) & \
                    (ranges >= range_min) & (ranges <= range_max)
        
        return ranges[mask & valid_mask]

    def detect_movement(self, current_distance: float, current_time: float) -> Tuple[str, float]:
        movement = "stable"
        speed = 0.0

        if self.prev_distance is not None and self.prev_time is not None:
            delta_distance = current_distance - self.prev_distance
            delta_time = current_time - self.prev_time
            
            if delta_time > 0:
                speed = delta_distance / delta_time 
            
            if abs(delta_distance) > self.stable_threshold:
                movement = "approaching" if delta_distance < 0 else "receding"

        self.prev_distance = current_distance
        self.prev_time = current_time
        
        return movement, speed

    def publish_movement(self, movement: str, distance: float, speed: float) -> None:
        msg = String()
        msg.data = f"{movement}, distance: {distance:.2f}m, speed: {speed:.3f}m/s"
        
        self.movement_pub.publish(msg)
        colors = {
            "approaching": "\033[91m",
            "receding": "\033[92m",
            "stable": "\033[94m"
        }
        reset = "\033[0m"
        
        rospy.loginfo(
            f"{colors[movement]}{movement.upper()}{reset} | "
            f"Distance: {distance:.2f}m | "
            f"Speed: {speed:.3f}m/s"
        )

def main() -> None:
    ObstacleDetector()
    rospy.spin()

if __name__ == '__main__':
    main()