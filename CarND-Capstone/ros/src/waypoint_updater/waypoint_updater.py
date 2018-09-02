#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32, Float64
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

LOOKAHEAD_WPS = 100  # Number of waypoints we will publish. You can change this number
MAX_DECEL = 0.5
MAX_ACCEL = 1

# this rate should be 10, don't use 50 as suggested in Udacity
# since it will cause the whole program slow down unbearably
WAYPOINT_PUBLISH_RATE = 10


def kmh_to_mps(km_per_hour):
    return km_per_hour / 3.6


def compute_dist(a, b):
    return math.sqrt((a.x - b.x) ** 2 +
                     (a.y - b.y) ** 2)


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        # get parameters
        self.speed_limit_mps = kmh_to_mps(rospy.get_param('~/waypoint_loader/velocity',
                                                          40.0))
        rospy.loginfo('speed limit [{:.2f}]'.format(self.speed_limit_mps))

        # TODO: Add other member variables you need below
        self.pose = None
        self.current_vel = 0.0
        self.waypoints = None
        self.stopline_wp_idx = -1
        self.closest_wp_idx = None
        self.prev_closest_wp_idx = None
        self.waypoints_2d = None
        self.waypoints_tree = None

        # action can be either 'stop' or 'go'
        self.current_action = None
        self.planned_wp_speed = [0] * LOOKAHEAD_WPS

        self.base_wp_sub = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # current position and speed
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)

        # get current velocity
        rospy.Subscriber('/current_velocity', TwistStamped, self.current_velocity_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        self.loop()

    def loop(self):
        rate = rospy.Rate(WAYPOINT_PUBLISH_RATE)

        while not rospy.is_shutdown():
            if None not in (self.pose,
                            self.waypoints,
                            self.waypoints_tree):
                self.publish_waypoints()

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

        if (val > 0):  # closest point is behind pos_xy
            closest_idx = (closest_idx + 1) % len(self.waypoints_2d)

        return closest_idx

    def get_next_action(self):
        if self.stopline_wp_idx == -1 or \
                self.stopline_wp_idx >= self.closest_wp_idx + LOOKAHEAD_WPS:
            return 'go'
        else:
            return 'stop'

    def publish_waypoints(self):
        # get closest waypoint
        self.closest_wp_idx = self.get_closest_waypoint_idx()
        next_action = self.get_next_action()

        # get list of waypoints
        planned_waypoints = []
        for i in range(self.closest_wp_idx, self.closest_wp_idx + LOOKAHEAD_WPS):
            planned_waypoints.append(self.waypoints[i % len(self.waypoints_2d)])

        if (next_action != self.current_action):
            self.planned_wp_speed = self.generate_speed(planned_waypoints,
                                                        next_action)
        else:
            self.planned_wp_speed = self.generate_speed_same_action(next_action)

        # update
        self.current_action = next_action
        self.prev_closest_wp_idx = self.closest_wp_idx

        for i, speed in enumerate(self.planned_wp_speed):
            self.set_waypoint_velocity(planned_waypoints, i, speed)

        lane = Lane()
        lane.header.frame_id = '/World'
        lane.header.stamp = rospy.Time(0)
        lane.waypoints = planned_waypoints

        self.final_waypoints_pub.publish(lane)

    def get_speed(self, init_speed, dist, action, decel):
        if action == 'go':
            vel = math.sqrt(init_speed ** 2 + 2 * MAX_ACCEL * dist)
            if vel > self.speed_limit_mps:
                vel = self.speed_limit_mps
        else:
            vel = init_speed ** 2 - 2 * decel * dist
            if vel < 1.0:
                vel = 0.
            else:
                vel = math.sqrt(vel)

        return vel

    def compute_decel(self):
        # ensure we stop at 2 waypoints before the stop-line
        dist = self.distance(self.waypoints,
                             self.closest_wp_idx,
                             self.stopline_wp_idx - 2)
        return self.current_vel ** 2 / (2.0 * dist)

    def generate_speed(self, waypoints, action):
        init_speed = self.current_vel

        decel = MAX_DECEL
        if action == 'stop':
            decel = self.compute_decel()

        prev_pos = self.pose.pose.position
        dist = 0.
        speeds = []
        for i in range(0, len(waypoints)):
            dist += compute_dist(prev_pos,
                                 waypoints[i].pose.pose.position)
            speeds.append(self.get_speed(init_speed, dist, action, decel))
            prev_pos = waypoints[i].pose.pose.position

        return speeds

    def generate_speed_same_action(self, action):
        offset = 0
        while offset < LOOKAHEAD_WPS:
            diff = offset + self.prev_closest_wp_idx - self.closest_wp_idx
            if (diff % len(self.waypoints_2d)) == 0:
                break
            offset += 1

        if offset > 0:
            last_speed = self.speed_limit_mps if action == 'go' else 0.0
            return self.planned_wp_speed[offset:] + [last_speed] * offset
        else:
            return self.planned_wp_speed

    def waypoints_cb(self, lane):
        # TODO: Implement
        self.waypoints = [waypoint for waypoint in lane.waypoints]
        self.waypoints_2d = [[waypoint.pose.pose.position.x,
                              waypoint.pose.pose.position.y] for waypoint in lane.waypoints]

        # create KDTree for finding nearest point in 2D
        self.waypoints_tree = KDTree(self.waypoints_2d)

        # un-subcribe from /base_waypoint topic so this callback is not called again

        self.base_wp_sub.unregister()
        rospy.loginfo("Unregistered from [/base_waypoints] topic")

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        self.stopline_wp_idx = msg.data

    def current_velocity_cb(self, msg):
        vx, vy = [msg.twist.linear.x, msg.twist.linear.y]
        self.current_vel = math.sqrt(vx ** 2 + vy ** 2)

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        for i in range(wp1 + 1, wp2 + 1):
            dist += compute_dist(waypoints[wp1].pose.pose.position,
                                 waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
