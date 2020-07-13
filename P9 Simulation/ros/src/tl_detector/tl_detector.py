#!/usr/bin/env python
import os
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml
from scipy.spatial import KDTree
import numpy as np
import logging
import threading
import math
import atexit

from darknet_ros_msgs.msg import BoundingBox
from darknet_ros_msgs.msg import BoundingBoxes

STATE_COUNT_THRESHOLD = 2

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.waypoints_2d   = None
        self.waypoint_tree  = None
        self.lights = []

        self.state2light = {0: 'Red', 1: 'Yellow', 2: 'Green', 4: 'Unknown'}

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
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)

        self.pose_lock          = threading.RLock()
        self.base_wp_lock       = threading.RLock()
        self.lights_lock        = threading.RLock()
        self.image_lock         = threading.RLock()
        self.bbox_lock          = threading.RLock()

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.stop_line_positions = self.config['stop_line_positions']
        rospy.loginfo("Length of self.stop_line_positions {0}".format(len(self.stop_line_positions)))

        self.stoplines_2d = [[stopline[0], stopline[1]] for stopline in self.stop_line_positions]
        self.stopline_tree = KDTree(self.stoplines_2d)
        rospy.loginfo("Length of self.stoplines_2d {0}: {1}".format(len(self.stoplines_2d), self.stoplines_2d))

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0
        self.prev_detected = TrafficLight.UNKNOWN

        # darknet_ros message
        sub5 = rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, self.detected_bb_cb)

        # Get simulator_mode parameter (1== ON, 0==OFF)
        self.simulator_mode = rospy.get_param("/simulator_mode")

        if int(self.simulator_mode) == 0:
            self.cropped_tl_bb_pub = rospy.Publisher('/cropped_bb', Image, queue_size=1)

        self.TL_BB_list = []

        # atexit.register(self._ros_atexit)

#        self.loop()
#
#    def loop(self):
#        rate = rospy.Rate(10)  # 10hz
#        while not rospy.is_shutdown():
#            if None not in [self.pose, self.waypoints, self.camera_image, self.waypoints_2d, self.waypoint_tree]:
#                self.update_waypoints()
#            rate.sleep()

        rospy.spin()

    def detected_bb_cb(self, msg):
        simulator_bb_size_threshold = 24 #px
        site_bb_size_threshold = 40 #px
        # min probability of detection
        simulator_bb_probability = 0.75
        site_bb_probability = 0.25

        if int(self.simulator_mode) == 1:
            prob_thresh = simulator_bb_probability
            size_thresh = simulator_bb_size_threshold
        else:
            prob_thresh = site_bb_probability
            size_thresh = site_bb_size_threshold

        self.bbox_lock.acquire()
        try:
            self.TL_BB_list = []
            for bb in msg.bounding_boxes:
                if str(bb.Class) == 'traffic light' and bb.probability >= prob_thresh:
                    if math.sqrt((bb.xmin - bb.xmax)**2 + (bb.ymin - bb.ymax)**2) >= size_thresh:
                        self.TL_BB_list.append(bb)
            if None not in [self.pose, self.waypoints, self.camera_image, self.waypoints_2d, self.waypoint_tree] and len(self.TL_BB_list) > 0:
                self.update_waypoints()
        finally:
            self.bbox_lock.release()

    def _ros_atexit(self):
        rospy.loginfo("Shutdown signal received")

    def update_waypoints(self):
        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        self.pose_lock.acquire()        
        try:
            pose = self.pose
            vehicle_idx = self.get_closest_waypoint(pose.pose.position.x, pose.pose.position.y)
        finally:
            self.pose_lock.release()

        self.base_wp_lock.acquire()
        try:
            waypoints = self.waypoints
        finally:
            self.base_wp_lock.release()
        
        self.lights_lock.acquire()
        try:
            lights = self.lights
        finally:
            self.lights_lock.release()

        self.image_lock.acquire()
        try:
            camera_image = self.camera_image
        finally:
            self.image_lock.release()

        self.bbox_lock.acquire()
        try:
            TL_BB_list = self.TL_BB_list
        finally:
            self.bbox_lock.release()
        
        light_wp, state = self.process_traffic_lights(pose, lights, TL_BB_list, camera_image)

        if self.state == TrafficLight.UNKNOWN or self.state != state:
            self.state = state
            self.state_count = 0
        elif self.state == state: 
            self.state_count += 1 

        if self.state_count >= STATE_COUNT_THRESHOLD: 
            self.state_count = 0
            self.last_state = self.state
            self.state = state
            behind = self.is_behind(waypoints, light_wp, vehicle_idx)
            return_wp = light_wp if (state == TrafficLight.RED or state == TrafficLight.UNKNOWN) and not behind else -1
            self.last_wp = return_wp
            rospy.loginfo("{0} light at wp {1}, vehicle_wp: {2}, return_wp: {3} (behind: {4})".format(self.state2light[state], light_wp, vehicle_idx, return_wp, behind)) 
            self.upcoming_red_light_pub.publish(Int32(return_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))

    def pose_cb(self, msg):
        self.pose_lock.acquire()
        try:
            self.pose = msg
        finally:
            self.pose_lock.release()

    def waypoints_cb(self, lane):
        self.base_wp_lock.acquire()
        try:
            self.waypoints = lane.waypoints
            if self.waypoints_2d is None:
                self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in self.waypoints]
                self.waypoint_tree = KDTree(self.waypoints_2d)

        finally:
            self.base_wp_lock.release()

    def traffic_cb(self, msg):
        self.lights_lock.acquire() 
        try:
            self.lights = msg.lights
        finally:
            self.lights_lock.release()

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        self.image_lock.acquire()
        try:
            self.camera_image = msg
            self.has_image = True
        finally:
            self.image_lock.release()

    def is_behind(self, waypoints, object_wp_idx, vehicle_wp_idx):
        wps_len = len(waypoints)
        
        if object_wp_idx == vehicle_wp_idx:
            return False

        if (object_wp_idx > vehicle_wp_idx):
            if (vehicle_wp_idx+wps_len)-object_wp_idx < (object_wp_idx-vehicle_wp_idx):
                return True
            else:
                return False
        else:
            if (object_wp_idx+wps_len)-vehicle_wp_idx < (vehicle_wp_idx-object_wp_idx):
                return False
            else:
                return True


    def get_closest_waypoint(self, x, y):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        wps_len = len(self.waypoints_2d)
        closest_idx = self.waypoint_tree.query([x, y], 1)[1]

        closest_coord = self.waypoints_2d[closest_idx]
        prev_coord = self.waypoints_2d[(wps_len+closest_idx-1)%wps_len]

        cl_vect = np.array(closest_coord)
        prev_vect = np.array(prev_coord)
        pose_vect = np.array([x, y])

        val = np.dot(cl_vect-prev_vect, pose_vect-cl_vect)
        if (val < 0.0):
            closest_idx = (closest_idx+wps_len-1) % wps_len

        return closest_idx

    def get_closest_stopline(self, x, y):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        closest_idx = self.stopline_tree.query([x, y], 1)[1]
        #rospy.loginfo("get_closest_stopline x {0} y {1}".format(x, y))
        #rospy.loginfo("Closest stop line {0}: x {1} y {2}".format(closest_idx, x, y))
        #rospy.loginfo("Length of self.stoplines_2d {0}: {1}".format(len(self.stoplines_2d), self.stoplines_2d))
        return closest_idx


    def get_light_state(self, camera_image, light, tl_bb_list):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #if(not self.has_image):
        #    self.prev_light_loc = None
        #    return False
        state = TrafficLight.UNKNOWN
        if len(tl_bb_list) > 0:
            cv_image = self.bridge.imgmsg_to_cv2(camera_image, "bgr8")

            #if int(self.simulator_mode) == 0:
            #Get classification
            state = self.light_classifier.get_classification(cv_image, tl_bb_list, self.simulator_mode)
        return state

    def process_traffic_lights(self, pose, lights, tl_bb_list, camera_image):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        light = None

        # List of positions that correspond to the line to stop in front of for a given intersection
        # stop_line_positions = self.config['stop_line_positions']
        light_idx = self.get_closest_stopline(pose.pose.position.x, pose.pose.position.y)
        # rospy.loginfo("Pose x {0} y {1}".format(pose.pose.position.x, pose.pose.position.y))
        # rospy.loginfo("light x {0} y {1}".format(self.stop_line_positions[light_idx][0], self.stop_line_positions[light_idx][1]))
        light_wp = self.get_closest_waypoint(self.stop_line_positions[light_idx][0], self.stop_line_positions[light_idx][1])
        # rospy.loginfo("light wp {0}".format(light_wp))
        light = self.lights[light_idx]

        #TODO find the closest visible traffic light (if one exists)

        if light is not None:
            state = self.get_light_state(camera_image, light, tl_bb_list)
            return light_wp, state
        return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
