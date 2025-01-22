#!/usr/bin/env python3
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from vision_msgs.msg import Detection2DArray

class ObstacleAvoidance:
    def __init__(self):
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.detection_sub = rospy.Subscriber('detected_objects', Detection2DArray, self.detection_callback)
        self.target_position = None
        self.safe_distance = 0.5  # meters

    def detection_callback(self, msg):
        if len(msg.detections) > 0:
            self.target_position = msg.detections[0].bbox.center

    def scan_callback(self, msg):
        cmd = Twist()
        front_scan = np.array(msg.ranges[-30:] + msg.ranges[:30])
        min_distance = np.nanmin(front_scan)
        
        if min_distance < self.safe_distance:
            # Obstacle avoidance behavior
            cmd.angular.z = -0.5 if np.mean(msg.ranges[:180]) > np.mean(msg.ranges[180:]) else 0.5
        elif self.target_position is not None:
            # Object following behavior
            error_x = self.target_position.x - 320  # 640x480 image width
            cmd.linear.x = 0.2
            cmd.angular.z = -error_x * 0.01
        else:
            # Search behavior
            cmd.angular.z = 0.5
        
        self.cmd_pub.publish(cmd)

if __name__ == '__main__':
    rospy.init_node('obstacle_avoidance')
    oa = ObstacleAvoidance()
    rospy.spin()