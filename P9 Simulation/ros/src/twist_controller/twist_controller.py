from pid import PID 
from lowpass import LowPassFilter
from yaw_controller import YawController 
import rospy
import math

GAS_DENSITY = 2.858
ONE_MPH = 0.44704

'''
This file defines the Controller class that performs the calculations to get the throttle, steering and brake inputs.
The control function takes the current velocity, linear velocity, angular velocity, dbw enabled flag;
and returns the throttle, brake and steering commands
It uses functions from the YawController class in yaw_controller.py to calculate the steering and functions from PID class in PID.py to adjust the
throttle depending on the velocity error.
The brake torque is calculated based on the velocity error and throttle values. If the vehicle is stopped i.e. the reference linear velocity = 0 and the current velocity is below 0.1, the brake torque is a constant 400 N-m, else if the reference velocity is lower than the current velocity & the throttle input is below 0.1, the brake torque is calculated based on the deceleration required, the mass of the vehicle and the wheel radius
It returns the calculated values only if the dbw module is enabled, else it returns a 0 value for all commands.
'''

class Controller(object):
    def __init__(self, vehicle_mass, fuel_capacity, brake_deadband, decel_limit,
        accel_limit, wheel_radius, wheel_base, steer_ratio, max_lat_accel, max_steer_angle):

        # TODO: Implement
        self.yaw_controller = YawController(wheel_base, steer_ratio, 0.1, max_lat_accel, max_steer_angle)

        th_kp = 0.271
        th_ki = 0.1
        th_kd = 0.001
        th_mn = 0. # Minimum throttle value
        th_mx = 1.0 # Maximum throttle value
        self.max_throttle = th_mx
        self.throttle_controller = PID(th_kp, th_ki, th_kd, th_mn, th_mx)

        tau = 0.5 # 1/(2pi*tau) = cutoff frequency
        ts = 0.02 # Sample time
        self.vel_lpf = LowPassFilter(tau, ts)

        self.vehicle_mass = vehicle_mass
        self.fuel_capacity = fuel_capacity
        self.brake_deadband = brake_deadband
        self.decel_limit = decel_limit
        self.accel_limit = accel_limit
        self.wheel_radius = wheel_radius
        
        self.last_time = rospy.get_time()

    def control(self, current_vel, dbw_enabled, linear_vel, angular_vel):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer

        if not dbw_enabled:
            self.throttle_controller.reset()
            return 0., 0., 0.

        current_vel = self.vel_lpf.filt(current_vel)

        steering = self.yaw_controller.get_steering(linear_vel, angular_vel, current_vel)

        vel_error = (linear_vel - current_vel)
        throt_error = vel_error*(1.0 - (1.0/math.exp(abs(vel_error))))

        self.last_vel = current_vel

        current_time = rospy.get_time()
        sample_time = rospy.Duration(current_time - self.last_time);
        self.last_time = current_time

        throttle = self.throttle_controller.step(throt_error, sample_time.to_sec())
        brake = 0.0

        if linear_vel == 0. and current_vel < 0.1:
            throttle = 0.
            brake = 400 #N-m - to hold the car in place if we stopped at a light. Acceleration ~ 1m/s^2
        elif throttle < 0.1 and throt_error < 0.0:
            throttle = 0.
            decel = max(throt_error, self.decel_limit)
            brake = abs(decel)*self.vehicle_mass*self.wheel_radius
        if brake > 0.:
          rospy.loginfo("throttle: {0}, brake: {1}, steering: {2}, linear_vel: {3}, current_vel: {4}, vel_error: {5}, throt_error: {6}".format(throttle, brake, steering, linear_vel, current_vel, vel_error, throt_error))

        return throttle, brake, steering
