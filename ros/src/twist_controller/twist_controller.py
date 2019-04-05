from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter
import rospy

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle, brake_deadband,
            accel_limit, decel_limit, vehicle_mass, wheel_radius, fuel_capacity, sample_freq):
        # TODO: Implement
        self.dt = 1.0/sample_freq
        self.brake_deadband = brake_deadband
        self.vehicle_mass = vehicle_mass
        self.wheel_radius = wheel_radius
        self.fuel_capacity = fuel_capacity
        self.decel_limit = decel_limit
        self.accel_limit = accel_limit

        #initalize the YAW controller, accel controller, throttle controller and low pass filter 
        self.yaw_controller = YawController(wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle)
        self.accel_controller = PID(kp=0.45, ki=0.02, kd=0.01, mn=decel_limit, mx=accel_limit)
        self.throttle_controller = PID(kp=1.0, ki=0.001, kd=0.10,  mn=0.0, mx=0.2)
        self.lowpass_filter = LowPassFilter(0.15, self.dt)


    def control(self, dbw_enabled, target_linear_velocity, target_angular_velocity, current_velocity):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        
        #check if dbw enabled then no need to compute the brake, steering and throttle values, return 0
        if not dbw_enabled:
            self.throttle_controller.reset()
            return 0., 0., 0.

        vel_error = target_linear_velocity = current_velocity
        raw_accel = self.accel_controller.step(vel_error, self.dt)

        self.lowpass_filter.filt(raw_accel)
        accel = self.lowpass_filter.get()

        #Get sterring values after getting angular and current velocity
        steer = self.yaw_controller.get_steering(target_linear_velocity, target_angular_velocity, current_velocity)

        brake = 0.0
        throttle = 0.0

        if accel > 0.0:
            throttle = self.throttle_controller.step(accel, self.dt)

        if target_linear_velocity == 0 and current_velocity < 0.1:
            throttle = 0
            # To prevent Carla from moving requires about 700 Nm of torque
            brake = 400

        elif throttle < .1 and accel < 0:
            throttle = 0
            accel = max(accel, self.decel_limit)
            brake = abs(accel) * (self.vehicle_mass + self.fuel_capacity * GAS_DENSITY) * self.wheel_radius * 2

        return throttle, brake, steer
