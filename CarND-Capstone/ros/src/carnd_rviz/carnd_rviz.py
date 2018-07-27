#!/usr/bin/python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from nav_msgs.msg import Path

# this controls how many update per second is sent to rviz
CARND_RVIZ_PUBLISH_RATE = 5

MAP_FRAME_ID = "/world"

def createPose(x, y, frame_id):
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = 0
    pose.pose.orientation.x = 0
    pose.pose.orientation.y = 0
    pose.pose.orientation.z = 0
    pose.pose.orientation.w = 0
    return pose

def createPath(waypoints, frame_id):
    path = Path()

    path.header.frame_id = frame_id

    for waypoint in waypoints:
        wp_pose = waypoint.pose.pose
        path.poses.append(
            createPose(wp_pose.position.x, wp_pose.position.y, MAP_FRAME_ID)
        )
    return path

class CarndRviz(object):
    def __init__(self):
        rospy.init_node('carnd_rviz')

        # get base waypoints
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber("/final_waypoints", Lane, self.final_waypoints_callback)

        # publish a path of base waypoints
        # (see http://wiki.ros.org/rospy/Overview/Publishers%20and%20Subscribers#rospy.Publisher_initialization)
        self._path_pub = rospy.Publisher('/carnd_rviz/base_waypoints', Path, queue_size=1, latch=True)
        self._final_path_pub = rospy.Publisher('/carnd_rviz/final_waypoints', Path, queue_size=1)

        # init member
        self._base_path = None

        # spinning
        self.loop()

    def loop(self):
        rate = rospy.Rate(CARND_RVIZ_PUBLISH_RATE)

        while not rospy.is_shutdown():
            rate.sleep()

    def waypoints_cb(self, lane):
        if self._base_path is None:
            self._base_path = createPath(lane.waypoints, MAP_FRAME_ID)
            rospy.loginfo("base path has {} poses".format(len(self._base_path.poses)))

        # publish path
        self._path_pub.publish(self._base_path)

    def final_waypoints_callback(self, lane):
        # publish planned waypoints
        self._final_path_pub.publish(createPath(lane.waypoints, MAP_FRAME_ID))

if __name__ == '__main__':
    try:
        CarndRviz()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')