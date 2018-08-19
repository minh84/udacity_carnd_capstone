#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32, Float64
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
from scipy.spatial import KDTree

import tf
import cv2
import numpy as np
import yaml

STATE_COUNT_THRESHOLD = 3

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.waypoints_tree = None
        self.stop_line_waypoint_idx = None
        self.camera_image = None
        self.lights = []

        # construction of light-classifier take a bit of time
        # so we move it here before setup call-back
        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.light_classifier = TLClassifier(self.config)
        rospy.loginfo('[TLDetector] Traffic-Light classifier is constructed')

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb
                                , queue_size=1
                                , buff_size=2 * 52428800)



        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)
        self.distance_to_traffic_light_pub = rospy.Publisher('distance_to_traffic', Float64, queue_size=1)

        self.bridge = CvBridge()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints.waypoints

        if not self.waypoints_tree:
            waypoints_2d = [[waypoint.pose.pose.position.x,
                             waypoint.pose.pose.position.y] for waypoint in self.waypoints]

            # create KDTree for finding nearest point in 2D
            self.waypoints_tree = KDTree(waypoints_2d)

            # query the nearest waypoint to each stop-line
            self.stop_line_waypoint_idx = []
            for i, stop_line_pos in enumerate(self.config['stop_line_positions']):
                light_wp_idx = self.get_closest_waypoint(stop_line_pos[0],
                                                         stop_line_pos[1])

                self.stop_line_waypoint_idx.append(light_wp_idx)

    def traffic_cb(self, msg):
        self.lights = msg.lights

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """

        # we receive from camera
        self.has_image = True
        self.camera_image = msg

        light_wp, state, dist = self.process_traffic_lights()

        if(dist is not None):
            self.distance_to_traffic_light_pub.publish(dist)

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            light_wp = light_wp if state == TrafficLight.RED else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1

    def get_closest_waypoint(self, x, y):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        #TODO implement
        closest_idx = self.waypoints_tree.query([x, y], 1)[1]
        return closest_idx

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """

        # for simulator, we can have state from light
        if not self.config['is_site'] and self.config.get('use_sim_light', False):
            return light.state
        else:
            if(not self.has_image):
                self.prev_light_loc = None
                return TrafficLight.UNKNOWN

            # we convert ROS image message to OpenCV format (ref: http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython)
            cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, 'bgr8')

            # Get classification
            state = self.light_classifier.get_classification(cv_image)
            if not self.config['is_site']:
                rospy.loginfo('Detected light-state {} ground-truth {}'.format(state, light.state))
            return state

    def distance(self, wp1, wp2):
        dist = 0.
        dl = lambda a, b: np.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1 + 1, wp2 + 1):
            dist += dl(self.waypoints[i-1].pose.pose.position,
                       self.waypoints[i].pose.pose.position)

        return dist

    def find_nearest_traffic_light(self, car_wp_idx):
        """
        Given car waypoint index, we find the nearest & ahead stop-line waypoint index
        Here we use loop since there is only few stop-lines
        Note that if self.stop_line_waypoint_idx is increasing one can use binary search
        :param car_wp_idx:
        :return:
        """
        closest_light_idx = None
        diff = len(self.waypoints)

        # loop through the line and find the nearest
        for i, stop_line_idx in enumerate(self.stop_line_waypoint_idx):
            d = stop_line_idx - car_wp_idx
            # we want to find stop-line that
            #   ahead of us => d >= 0
            #   nearest => minimize diff
            if d >= 0 and d < diff:
                diff = d
                closest_light_idx = i
        return closest_light_idx

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        closest_light_idx = None
        car_wp_idx = None
        dist = None

        if(self.pose):
            car_wp_idx = self.get_closest_waypoint(self.pose.pose.position.x,
                                                   self.pose.pose.position.y)

            closest_light_idx = self.find_nearest_traffic_light(car_wp_idx)

        #TODO find the closest visible traffic light (if one exists)
        if closest_light_idx is not None:
            line_wp_idx = self.stop_line_waypoint_idx[closest_light_idx]
            closest_light = self.lights[closest_light_idx]

            # we only check if stop-line waypoint is less than 200 ahead of the current car
            if (line_wp_idx < car_wp_idx + self.config['look_ahead_waypoints']):
                dist = self.distance(car_wp_idx, line_wp_idx)
                state = self.get_light_state(closest_light)

                return line_wp_idx, state, dist

        return -1, TrafficLight.UNKNOWN, dist

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
