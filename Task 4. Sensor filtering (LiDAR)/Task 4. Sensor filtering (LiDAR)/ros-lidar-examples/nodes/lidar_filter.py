#!/usr/bin/env python3
import numpy as np
import rospy
from sensor_msgs.msg import LaserScan
from collections import deque

class LidarFilter:
    def __init__(self):
        rospy.init_node('lidar_filter')
        
        self.window_size = rospy.get_param('~window_size', 5)
        self.smoothing_factor = rospy.get_param('~smoothing_factor', 0.3)
        self.scan_buffer = deque(maxlen=self.window_size)
        self.filtered_publisher = rospy.Publisher('/scan_filtered', LaserScan, queue_size=10)
        self.raw_subscriber = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.last_publish_time = rospy.Time.now()
        self.publish_rate = 0

    def scan_callback(self, msg):
        ranges = np.array(msg.ranges)
        valid_mask = (~np.isnan(ranges)) & (~np.isinf(ranges)) & \
                    (ranges >= msg.range_min) & (ranges <= msg.range_max)
        filtered_ranges = ranges.copy()
        filtered_ranges[~valid_mask] = msg.range_max
        self.scan_buffer.append(filtered_ranges)
        if len(self.scan_buffer) < self.window_size:
            return
        median_filtered = np.median(self.scan_buffer, axis=0)

        if not hasattr(self, 'prev_filtered'):
            self.prev_filtered = median_filtered
        smoothed_ranges = self.smoothing_factor * median_filtered + (1 - self.smoothing_factor) * self.prev_filtered
        self.prev_filtered = smoothed_ranges

        filtered_msg = msg
        filtered_msg.ranges = smoothed_ranges.tolist()
        self.filtered_publisher.publish(filtered_msg)

        now = rospy.Time.now()
        self.publish_rate = 1.0 / (now - self.last_publish_time).to_sec()
        self.last_publish_time = now

if __name__ == '__main__':
    try:
        LidarFilter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass