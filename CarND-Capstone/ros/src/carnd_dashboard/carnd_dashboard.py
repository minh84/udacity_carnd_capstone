#!/usr/bin/python

import rospy
from styx_msgs.msg import Lane

# this controls how many update per second is sent to rviz
CARND_RQT_PUBLISH_RATE = 5

class CarndDashboard(object):
    def __init__(self):
        rospy.init_node('carnd_rviz')

        # get based waypoints
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # get planned waypoints
        rospy.Subscriber("/final_waypoints", Lane, self.final_waypoints_cb)