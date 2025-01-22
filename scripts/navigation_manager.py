#!/usr/bin/env python3
import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from geometry_msgs.msg import PoseStamped

class NavigationManager:
    def __init__(self):
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
        self.goal = MoveBaseGoal()
        self.goal.target_pose.header.frame_id = "map"

    def navigate_to_object(self, detection):
        if detection is not None:
            self.goal.target_pose.pose.position.x = detection.bbox.center.x
            self.goal.target_pose.pose.position.y = detection.bbox.center.y
            self.goal.target_pose.header.stamp = rospy.Time.now()
            self.client.send_goal(self.goal)

if __name__ == '__main__':
    rospy.init_node('navigation_manager')
    nm = NavigationManager()
    rospy.spin()