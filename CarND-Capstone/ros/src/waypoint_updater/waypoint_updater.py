#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32
from scipy.spatial import KDTree
import numpy as np

import math

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number
MAX_DECEL = 0.5

# this rate should be 10, don't use 50 as suggested in Udacity
# since it will cause the whole program slow down unbearably
WAYPOINT_PUBLISH_RATE = 10

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)


        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.pose = None
        self.base_waypoints = None
        self.stopline_wp_idx = -1
        self.waypoints_2d = None
        self.waypoints_tree = None

        self.loop()

    def loop(self):
        rate = rospy.Rate(WAYPOINT_PUBLISH_RATE)

        while not rospy.is_shutdown():
            if self.pose is not None and self.base_waypoints is not None:
                # get closest waypoint
                closest_waypoint_idx = self.get_closest_waypoint_idx()
                self.publish_waypoints(closest_waypoint_idx)
            rate.sleep()

    def pose_cb(self, msg):
        # TODO: Implement
        self.pose = msg

    def get_closest_waypoint_idx(self):
        x = self.pose.pose.position.x
        y = self.pose.pose.position.y
        closest_idx = self.waypoints_tree.query([x, y], 1)[1]

        # check if closest is ahead or behind vehicle
        closest_coord = np.array(self.waypoints_2d[closest_idx])
        prev_coord = np.array(self.waypoints_2d[closest_idx - 1])
        pos_xy = np.array([x, y])

        # check the direction
        val = np.dot(closest_coord - prev_coord, pos_xy - closest_coord)

        if (val > 0): # closest point is behind pos_xy
            closest_idx = (closest_idx + 1) % len(self.waypoints_2d)

        return closest_idx

    def publish_waypoints(self, closest_idx):
        final_lane = self.generate_lane()
        self.final_waypoints_pub.publish(final_lane)

    def generate_lane(self):
        lane = Lane()
        lane.header = self.base_waypoints.header
        closest_idx = self.get_closest_waypoint_idx()
        lookahead_idx = closest_idx + LOOKAHEAD_WPS
        base_waypoints = self.base_waypoints.waypoints[closest_idx : lookahead_idx]

        if self.stopline_wp_idx==-1 or (self.stopline_wp_idx >= lookahead_idx):
            lane.waypoints = base_waypoints
        else:
            lane.waypoints = self.decelerate_waypoints(base_waypoints, closest_idx)
        return lane

    def decelerate_waypoints(self, waypoints, closest_idx):
        stop_idx = max(self.stopline_wp_idx - closest_idx - 2, 0)
        rospy.loginfo("[waypoint_updater] closest_idx={}, stopline_wp_idx={}".format(closest_idx, self.stopline_wp_idx))
        temp = []

        for i, wp in enumerate(waypoints):
            p = Waypoint()
            p.pose = wp.pose

            dist = self.distance(waypoints, i, stop_idx)
            vel = np.sqrt(2 * MAX_DECEL * dist)
            if vel < 1.:
                vel = 0.

            p.twist.twist.linear.x = min(vel, p.twist.twist.linear.x )
            temp.append(p)
        return temp

    def waypoints_cb(self, lane):
        # TODO: Implement
        if not self.base_waypoints:
            self.base_waypoints = lane
            self.waypoints_2d = [[waypoint.pose.pose.position.x,
                                  waypoint.pose.pose.position.y] for waypoint in lane.waypoints]

            # create KDTree for finding nearest point in 2D
            self.waypoints_tree = KDTree(self.waypoints_2d)


    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        self.stopline_wp_idx = msg.data

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
