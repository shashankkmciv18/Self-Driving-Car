#!/usr/bin/env python

import sys
import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane, Waypoint, TrafficLight
from dbw_mkz_msgs.msg import SteeringReport
from std_msgs.msg import Int32
#from scipy.interpolate import interp1d
from scipy.spatial import KDTree
import numpy as np
import logging
import threading
import atexit
import math
import tf

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

LOOKAHEAD_WPS = 200
MAX_DECEL = 1.0  # Maximum Deceleration value
BRAKE_DISTANCE = 35  # distance to begin slowing down
MAX_ACCEL = 1.0  # Maximum Acceleration value


class WaypointUpdater(object):
    def __init__(self):
        # TODO: Add other member variables you need below

        rospy.init_node('waypoint_updater')
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        rospy.Subscriber('/current_velocity', TwistStamped, self.velocity_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        self.logger = logging.getLogger()
        self.logger.setLevel(logging.DEBUG)
        self.formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')
        self.fh = logging.FileHandler('log_filename.txt')
        self.fh.setLevel(logging.DEBUG)
        self.fh.setFormatter(self.formatter)
        self.logger.addHandler(self.fh)

        self.pose_lock       = threading.RLock()
        self.velocity_lock   = threading.RLock() 
        self.base_wp_lock    = threading.RLock()
        self.traffic_wp_lock = threading.RLock()

        self.brake_dist = BRAKE_DISTANCE

        self.accel = 0.0
        self.stop_index_dist = -1
        self.pose = None
        self.velocity = None
        self.base_waypoints = None
        self.waypoints_2d = None
        self.traffic_light_pos = -1
        self.waypoint_tree = None
        self.lane_header = None
        self.speed_limit = self.kmph2mps(rospy.get_param('/waypoint_loader/velocity')) 
        self.lookahead_wps = LOOKAHEAD_WPS

        atexit.register(self._ros_atexit)

        self.loop()

    def loop(self):
        rate = rospy.Rate(50)  # 50hz
        while not rospy.is_shutdown():
            if self.velocity is not None and self.base_waypoints is not None and self.pose is not None and self.waypoint_tree is not None:
                self.update_waypoints()
            rate.sleep()

    # rospy.spin()

    def _ros_atexit(self):
        rospy.loginfo("Shutdown signal received")

    def update_waypoints(self):
        self.pose_lock.acquire()
        try:
            pose = self.pose
        finally:
            self.pose_lock.release()

        self.base_wp_lock.acquire()
        try:
            base_waypoints = self.base_waypoints
        finally:
            self.base_wp_lock.release()

        self.traffic_wp_lock.acquire()
        try:
            traffic_light_pos = self.traffic_light_pos
        finally:
            self.traffic_wp_lock.release()

        self.velocity_lock.acquire()
        try:
            velocity = self.velocity
        finally:
            self.velocity_lock.release()

        closest_dist, closest_idx = self.get_closest_waypoint_index(pose.pose.position.x, pose.pose.position.y);

        if traffic_light_pos != -1:
            light_dist = self.distance(base_waypoints, closest_idx, traffic_light_pos)
            if MAX_DECEL > 0.00001:
                self.brake_dist = abs(velocity**2/2.0*MAX_DECEL)
            else:
                self.brake_dist = BRAKE_DISTANCE
            
            if light_dist <= self.brake_dist:
                self.stop_index_dist = self.get_wp_distance(base_waypoints, closest_idx, traffic_light_pos)
                if self.stop_index_dist > 0:
                    self.accel = velocity/float(self.stop_index_dist)
                else: 
                    self.accel = -MAX_DECEL
        else:
            self.stop_index_dist = -1
            if  abs(velocity-self.speed_limit) <= 0.001:
                self.accel = 0.0
            elif velocity > self.speed_limit:
                self.accel = -MAX_DECEL
            else:
                self.accel = MAX_ACCEL

        rospy.loginfo("traffic_light_pos: {0}, self.stop_index_dist: {1}, self.accel {2}".format(traffic_light_pos, self.stop_index_dist, self.accel))
        self.publish_waypoints(base_waypoints, pose, closest_dist, closest_idx, velocity, self.accel)

    def publish_waypoints(self, waypoints, pose, closest_dist, closest_idx, velocity, accel):   
        lane = Lane();
        lane.header = self.lane_header
        wps_len = len(waypoints)
        farest_idx = closest_idx + self.lookahead_wps
        if farest_idx < wps_len:
            waypoints_tmp = waypoints[closest_idx:farest_idx]
        else:
            waypoints_tmp = waypoints[closest_idx:] + waypoints[:farest_idx%wps_len]

        start_idx = int(velocity/6.0) # waypoints to skip if latency is high

        wp = waypoints_tmp[start_idx]
        dist = math.sqrt((pose.pose.position.x-wp.pose.pose.position.x)**2+(pose.pose.position.y-wp.pose.pose.position.y)**2+(pose.pose.position.z-wp.pose.pose.position.z)**2)
        v0 = velocity 
        vpow2 = v0**2 + 2.0*accel*dist
        if vpow2 > 0.0: 
            v  = math.sqrt(vpow2) # based on v^2 = v0^2 + 2a(x-x0)
        else:
            v = 0.0

        v_tmp = v
        if self.stop_index_dist >= 0:
            if self.accel > 0:
                self.accel = -self.accel
            if self.stop_index_dist == 0:
                stop_accel = self.accel
            else:
                stop_accel = -abs(v/float(self.stop_index_dist))
        else:
            stop_accel = 0.0
        
        for i in range(start_idx, self.lookahead_wps):
            if self.stop_index_dist >= 0:
                v_tmp += stop_accel
                v = min(v, v_tmp)

            if v > self.speed_limit:
                v = self.speed_limit
            if v < 0.0:
                v = 0.0

            waypoints_tmp[i].twist.twist.linear.x = v 
            if i <= self.lookahead_wps-2:
                v0 = v
                dist = math.sqrt((waypoints_tmp[i].pose.pose.position.x-waypoints_tmp[i+1].pose.pose.position.x)**2+(waypoints_tmp[i].pose.pose.position.y-waypoints_tmp[i+1].pose.pose.position.y)**2+(waypoints_tmp[i].pose.pose.position.z-waypoints_tmp[i+1].pose.pose.position.z)**2)

                vpow2 = v0**2 + 2.0*accel*dist
                if vpow2 > 0.0:
                    v  = math.sqrt(vpow2) # based on v^2 = v0^2 + 2a(x-x0)
                else:
                    v = 0.0

        
        lane.waypoints = waypoints_tmp[start_idx:self.lookahead_wps]
        # rospy.loginfo("WP Len: {0} : {1}".format(len(lane.waypoints), lane.waypoints[0:10]));
        self.final_waypoints_pub.publish(lane)

    # get index distance from wp1 to wp2
    def get_wp_distance(self, waypoints, wp1_idx, wp2_idx):
        wps_len = len(waypoints)

        if wp1_idx == wp2_idx:
            return 0

        if (wp1_idx > wp2_idx):
            return (wp2_idx + wps_len) - wp1_idx
        else: 
            return (wp2_idx - wp1_idx)


    def get_closest_waypoint_index(self, x, y):
        wps_len = len(self.waypoints_2d)
        closest_dist, closest_idx = self.waypoint_tree.query([x, y], 1)

        closest_coord = self.waypoints_2d[closest_idx]
        prev_coord = self.waypoints_2d[(wps_len+closest_idx-1)%wps_len]
        
        cl_vect = np.array(closest_coord)
        prev_vect = np.array(prev_coord)
        pose_vect = np.array([x, y])
        
        val = np.dot(cl_vect-prev_vect, pose_vect-cl_vect)
        if val >= 0.0:
            closest_idx = (closest_idx + 1) % wps_len

        #rospy.loginfo("Closest dist: {0}, Closest idx {1}".format(closest_dist, closest_idx))

        return closest_dist, closest_idx

    def pose_cb(self, msg):
        self.pose_lock.acquire()
        try:
            self.pose = msg
        finally:
            self.pose_lock.release()
 
    def waypoints_cb(self, lane):
        self.base_wp_lock.acquire()
        try:
            self.base_waypoints = lane.waypoints
            self.lane_header    = lane.header
            if self.waypoints_2d is None:
                self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in self.base_waypoints]
                self.waypoint_tree = KDTree(self.waypoints_2d)
        finally:
            self.base_wp_lock.release()

    def velocity_cb(self, msg):
        self.velocity_lock.acquire()
        try:
            self.velocity = msg.twist.linear.x
        finally:
            self.velocity_lock.release()
                
    def traffic_cb(self, msg):
        self.traffic_wp_lock.acquire()
        try:
            self.traffic_light_pos = msg.data
        finally:
            self.traffic_wp_lock.release()

        #if self.traffic_light_pos != -1:
        #    rospy.loginfo("Received light for waypoint {0}".format(self.traffic_light_pos))

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, start, end):
        dist = 0
        wps_len = len(waypoints)
        if end%wps_len < start%wps_len:
            end = end%wps_len + wps_len
        dl = lambda a, b: math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2 + (a.z - b.z) ** 2)
        for i in range(start, end):
            dist += dl(waypoints[i%wps_len].pose.pose.position, waypoints[(i+1)%wps_len].pose.pose.position)
        return dist


    ## taken from the waypoint_loader
    def kmph2mps(self, velocity_kmph):
        return (velocity_kmph * 1000.) / (60. * 60.)

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
