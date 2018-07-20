from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter

import rospy

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self,
                 vehicle_mass,
                 fuel_capacity,
                 brake_deadband,
                 decel_limit,
                 accel_limit,
                 wheel_radius,
                 wheel_base,
                 steer_ratio,
                 max_lat_accel,
                 max_steer_angle):
        # TODO: Implement

        # control steering angle
        self.yaw_controller = YawController(wheel_base,
                                            steer_ratio,
                                            0.1,
                                            max_lat_accel,
                                            max_steer_angle)

        # control throttle
        kp = 0.3
        ki = 0.1
        kd = 0.0
        throttle_min = 0.0
        throttle_max = 0.2
        self.throttle_controller = PID(kp, ki, kd, throttle_min, throttle_max)

        # smooth out velocity
        tau = 0.5  # cut off frequency
        ts = 0.02  # sample duration (50Hz)
        self.velocity_lpf = LowPassFilter(tau, ts)

        # store car's parameter
        self.vehicle_mass = vehicle_mass
        self.fuel_capacity = fuel_capacity
        self.brake_deadband = brake_deadband
        self.decel_limit = decel_limit
        self.accel_limit = accel_limit
        self.wheel_radius = wheel_radius

        self.last_time = rospy.get_time()


    def control(self,
                current_vel,
                linear_vel,
                angular_vel,
                dbw_enabled):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer

        if not dbw_enabled:
            self.throttle_controller.reset()
            return 0., 0., 0.

        # smooth out velocity
        current_vel = self.velocity_lpf.filt(current_vel)

        # compute steering
        steering = self.yaw_controller.get_steering(linear_vel, angular_vel, current_vel)

        # compute throttle
        error_vel = linear_vel - current_vel
        self.last_vel = current_vel

        current_time = rospy.get_time()
        sample_time = current_time - self.last_time
        self.last_time = current_time

        throttle = self.throttle_controller.step(error_vel, sample_time)

        # compute brake
        brake = 0.0

        if linear_vel == 0.0 and current_vel < 0.1:
            throttle = 0.0
            brake = 700
        elif throttle < 0.1 and error_vel < 0.:
            throttle = 0.0
            decel = max(error_vel, self.decel_limit)
            brake = abs(decel) * self.vehicle_mass * self.wheel_radius

        return  throttle, brake, steering

