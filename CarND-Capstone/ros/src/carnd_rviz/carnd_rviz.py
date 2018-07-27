#!/usr/bin/python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint, TrafficLightArray, TrafficLight

from nav_msgs.msg import Path
from visualization_msgs.msg import MarkerArray, Marker

from scipy.spatial import KDTree


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

def createLightMarker(light, index, frame_id):
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.ns = "light_ns"
    marker.id = index
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD

    marker.pose.position = light.pose.pose.position
    marker.pose.orientation = light.pose.pose.orientation
    marker.scale.x = 10.0
    marker.scale.y = 10.0
    marker.scale.z = 10.0

    # light = TrafficLight()
    if light.state == TrafficLight.RED:
        marker.color.r = 1.0
        marker.color.g = 0.0
    elif light.state == TrafficLight.GREEN:
        marker.color.r = 0.0
        marker.color.g = 1.0
    elif light.state == TrafficLight.YELLOW:
        marker.color.r = 1.0
        marker.color.g = 1.0
    elif light.state == TrafficLight.UNKNOWN:
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0

    marker.color.a = 1.0
    return marker

class CarndRviz(object):
    def __init__(self):
        rospy.init_node('carnd_rviz')

        # get base waypoints & final waypoints
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber("/final_waypoints", Lane, self.final_waypoints_callback)

        # get current pose
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)

        # get traffic light info, in sim, we get the light-status
        rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)

        # publish a path of base waypoints
        # (see http://wiki.ros.org/rospy/Overview/Publishers%20and%20Subscribers#rospy.Publisher_initialization)
        self._path_pub = rospy.Publisher('/carnd_rviz/base_waypoints', Path, queue_size=1, latch=True)
        self._final_path_pub = rospy.Publisher('/carnd_rviz/final_waypoints', Path, queue_size=1)
        self._traffic_light_pub = rospy.Publisher("/carnd_rviz/traffic_lights", MarkerArray, queue_size=1)

        # init member
        self._base_path = None
        self._base_tree = None
        self._current_pose = None
        self._trafic_light = None

        # spinning
        self.loop()

    def loop(self):
        rate = rospy.Rate(CARND_RVIZ_PUBLISH_RATE)

        while not rospy.is_shutdown():
            # generate the marker for all light (this only
            self.publish_traffic_light_markers()

            rate.sleep()

    def publish_traffic_light_markers(self):
        if self._trafic_light is None:
            return

        marker_array = MarkerArray()
        for index, light in enumerate(self._trafic_light):
            marker_array.markers.append(createLightMarker(light, index, MAP_FRAME_ID))

        self._traffic_light_pub.publish(marker_array)

    def waypoints_cb(self, lane):
        if self._base_path is None:
            self._base_path = createPath(lane.waypoints, MAP_FRAME_ID)
            waypoints_2d = [[waypoint.pose.pose.position.x,
                             waypoint.pose.pose.position.y] for waypoint in lane.waypoints]

            # create KDTree for finding nearest point in 2D
            self._base_tree = KDTree(waypoints_2d)

        # publish path
        self._path_pub.publish(self._base_path)

    def pose_cb(self, pose):
        self._current_pose = [pose.pose.position.x, pose.pose.position.y]

    def final_waypoints_callback(self, lane):
        # publish planned waypoints
        self._final_path_pub.publish(createPath(lane.waypoints, MAP_FRAME_ID))

    def traffic_cb(self, lightMsg):
        self._trafic_light = lightMsg.lights;


if __name__ == '__main__':
    try:
        CarndRviz()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')