#!/usr/bin/env python3
import numpy as np
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, String, ColorRGBA
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

class WallTracker:
    def __init__(self):
        rospy.init_node('wall_tracker')

        self.sector_width = rospy.get_param('~sector_width', 5.0)
        self.stable_threshold = rospy.get_param('~stable_threshold', 0.03)
        self.prev_distance = None
        self.current_state = "NO_WALL"
        self.dist_pub = rospy.Publisher('/wall_distance', Float32, queue_size=1)
        self.motion_pub = rospy.Publisher('/wall_motion', String, queue_size=1)
        self.marker_pub = rospy.Publisher('/visualization_marker', Marker, queue_size=1)
        rospy.Subscriber('/scan_filtered', LaserScan, self.scan_callback, queue_size=1)

    def scan_callback(self, msg):
        angles = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))
        half_width = np.radians(self.sector_width/2)
        sector_mask = np.abs(angles) <= half_width
        sector_ranges = np.array(msg.ranges)[sector_mask]
        valid_mask = (sector_ranges < msg.range_max) & (sector_ranges >= msg.range_min)
        valid_ranges = sector_ranges[valid_mask]
        
        if len(valid_ranges) == 0:
            self.current_state = "NO_WALL"
            rospy.logwarn("No wall detected in scanning sector!")
            self.publish_results(float('nan'))
            return
            
        current_dist = np.min(valid_ranges)
        
        if self.prev_distance is not None:
            delta = self.prev_distance - current_dist
            
            if abs(delta) > self.stable_threshold:
                self.current_state = "APPROACHING" if delta > 0 else "RECEDING"
            else:
                self.current_state = "STABLE"
        
        self.prev_distance = current_dist
        self.publish_results(current_dist)

    def publish_results(self, distance):
        self.dist_pub.publish(Float32(distance))
        
        motion_msg = String()
        motion_msg.data = self.current_state
        self.motion_pub.publish(motion_msg)
        
        if self.current_state != "NO_WALL":
            colors = {
                "APPROACHING": "\033[91m", 
                "RECEDING": "\033[92m", 
                "STABLE": "\033[94m" 
            }
            reset = "\033[0m"
            
            rospy.loginfo(
                f"{colors[self.current_state]}{self.current_state}{reset} | "
                f"Distance: {distance:.2f}m"
            )
        
        self.visualize_state()

    def visualize_state(self):
        marker = Marker()
        marker.header.frame_id = "base_scan"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "wall_tracker"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        
        start = Point(0, 0, 0)
        end = Point()
        
        if self.current_state == "APPROACHING":
            end.x = -0.3
            color = ColorRGBA(1, 0, 0, 1)
        elif self.current_state == "RECEDING":
            end.x = 0.3
            color = ColorRGBA(0, 1, 0, 1)
        elif self.current_state == "STABLE":
            end.y = 0.2
            color = ColorRGBA(0, 0, 1, 1)
        else:
            end.z = 0.2
            color = ColorRGBA(0.5, 0.5, 0.5, 0.5)
        
        marker.points = [start, end]
        marker.scale.x = 0.05
        marker.scale.y = 0.1
        marker.color = color
        marker.lifetime = rospy.Duration(0.5)
        
        self.marker_pub.publish(marker)

if __name__ == '__main__':
    WallTracker()
    rospy.spin()