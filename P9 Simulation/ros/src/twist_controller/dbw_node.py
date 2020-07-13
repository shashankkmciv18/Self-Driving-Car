#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped
import math
import threading

from twist_controller import Controller


'''
This file defines the dbw node of the vehicle that controls the throttle, brake and steering commands.
It subscribes to the following topics:
	1. /vehicle/dbw_enabled
	2. /twist_cmd
	3. /current_velocity
And publishes the following commands:
	1. Throttle
	2. Brake
	3. Steering
It uses the Controller class from twist_controller.py to calculate the required throttle, brake and steering inputs.
The commands are only published when the driver enables the dbw module.
'''


class DBWNode(object):
    def __init__(self):
        rospy.init_node('dbw_node')

        vehicle_mass = rospy.get_param('~vehicle_mass', 1736.35)
        fuel_capacity = rospy.get_param('~fuel_capacity', 13.5)
        brake_deadband = rospy.get_param('~brake_deadband', .1)
        decel_limit = rospy.get_param('~decel_limit', -5)
        accel_limit = rospy.get_param('~accel_limit', 1.)
        wheel_radius = rospy.get_param('~wheel_radius', 0.2413)
        wheel_base = rospy.get_param('~wheel_base', 2.8498)
        steer_ratio = rospy.get_param('~steer_ratio', 14.8)
        max_lat_accel = rospy.get_param('~max_lat_accel', 3.)
        max_steer_angle = rospy.get_param('~max_steer_angle', 8.)

        self.steer_pub = rospy.Publisher('/vehicle/steering_cmd',
                                         SteeringCmd, queue_size=1)
        self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd',
                                            ThrottleCmd, queue_size=1)
        self.brake_pub = rospy.Publisher('/vehicle/brake_cmd',
                                         BrakeCmd, queue_size=1)

        # TODO: Create `Controller` object
        # self.controller = Controller(<Arguments you wish to provide>)
        self.controller = Controller(vehicle_mass=vehicle_mass,
                                    fuel_capacity=fuel_capacity,
                                    brake_deadband=brake_deadband,
                                    decel_limit=decel_limit,
                                    accel_limit=accel_limit,
                                    wheel_radius=wheel_radius,
                                    wheel_base=wheel_base,
                                    steer_ratio=steer_ratio,
                                    max_lat_accel=max_lat_accel,
                                    max_steer_angle=max_steer_angle)

        # TODO: Subscribe to all the topics you need to
        rospy.Subscriber('/vehicle/dbw_enabled', Bool, self.dbw_enabled_cb)
        rospy.Subscriber('/twist_cmd', TwistStamped, self.twist_cb)
        rospy.Subscriber('/current_velocity', TwistStamped, self.velocity_cb)

        self.dbw_lock       = threading.RLock()
        self.twist_lock     = threading.RLock()
        self.veloc_lock     = threading.RLock()

        self.current_vel = None
        self.curr_ang_vel = None
        self.dbw_enabled = None
        self.linear_vel = None
        self.angular_vel = None
        self.throttle = self.steering = self.brake = 0

        self.loop()

    def loop(self):
        rate = rospy.Rate(50) # 50Hz
        while not rospy.is_shutdown():
            # TODO: Get predicted throttle, brake, and steering using `twist_controller`
            # You should only publish the control commands if dbw is enabled
            # throttle, brake, steering = self.controller.control(<proposed linear velocity>,
            #                                                     <proposed angular velocity>,
            #                                                     <current linear velocity>,
            #                                                     <dbw status>,
            #                                                     <any other argument you need>)
            if not None in (self.current_vel, self.linear_vel, self.angular_vel):
                self.dbw_lock.acquire()
                try:
                    dbw_enabled = self.dbw_enabled
                finally:
                    self.dbw_lock.release()

                self.twist_lock.acquire()
                try:
                    linear_vel = self.linear_vel
                    angular_vel = self.angular_vel
                finally: 
                    self.twist_lock.release()
                
                self.veloc_lock.acquire()
                try:
                    current_vel = self.current_vel
                finally:
                    self.veloc_lock.release()

                self.throttle, self.brake, self.steering = self.controller.control(current_vel,
                                                                    dbw_enabled,
                                                                    linear_vel,
                                                                    angular_vel)
                # if <dbw is enabled>:
                #   self.publish(throttle, brake, steer)
                if self.dbw_enabled:
                    self.publish(self.throttle, self.brake, self.steering)
            rate.sleep()

    def dbw_enabled_cb(self, msg):
        self.dbw_lock.acquire()
        try:
            self.dbw_enabled = msg
        finally:
            self.dbw_lock.release()

    def twist_cb(self, msg):
        self.twist_lock.acquire()
        try:
            self.linear_vel = msg.twist.linear.x
            self.angular_vel = msg.twist.angular.z
        finally:
            self.twist_lock.release()

    def velocity_cb(self, msg):
        self.veloc_lock.acquire()
        try:
            self.current_vel = msg.twist.linear.x
        finally:
            self.veloc_lock.release()

    def publish(self, throttle, brake, steer):
      
        # rospy.loginfo("throttle: {0}, brake {1}, steer {2}, linear {3}, angular {4}".format(throttle, brake, steer, self.linear_vel, self.angular_vel))
        tcmd = ThrottleCmd()
        tcmd.enable = True
        tcmd.pedal_cmd_type = ThrottleCmd.CMD_PERCENT
        tcmd.pedal_cmd = throttle
        self.throttle_pub.publish(tcmd)

        scmd = SteeringCmd()
        scmd.enable = True
        scmd.steering_wheel_angle_cmd = steer
        self.steer_pub.publish(scmd)

        bcmd = BrakeCmd()
        bcmd.enable = True
        bcmd.pedal_cmd_type = BrakeCmd.CMD_TORQUE
        bcmd.pedal_cmd = brake
        self.brake_pub.publish(bcmd)


if __name__ == '__main__':
    DBWNode()
