import rospy

from lowpass import LowPassFilter
from pid import PID
from yaw_controller import YawController

GAS_DENSITY = 2.858

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

        # store car's parameter
        self.brake_deadband = brake_deadband
        self.decel_limit = decel_limit
        self.accel_limit = accel_limit
        self.wheel_radius = wheel_radius

        self.total_mass = vehicle_mass + fuel_capacity * GAS_DENSITY

        # control steering angle
        self.yaw_controller = YawController(wheel_base,
                                            steer_ratio,
                                            0.1,
                                            max_lat_accel,
                                            max_steer_angle)

        # control throttle
        kp = 1.5
        ki = 0.001
        kd = 0.0
        self.throttle_controller = PID(kp, ki, kd,
                                       self.decel_limit,
                                       self.accel_limit)

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

        # compute steering
        steering = self.yaw_controller.get_steering(linear_vel, angular_vel, current_vel)

        # compute throttle
        error_vel = linear_vel - current_vel
        self.last_vel = current_vel

        current_time = rospy.get_time()
        sample_time = current_time - self.last_time
        self.last_time = current_time

        # throttle is acceleration
        throttle = self.throttle_controller.step(error_vel, sample_time)

        # compute brake
        brake = 0.0

        if linear_vel == 0.0 and current_vel < 0.1:
            throttle = 0.0
            brake = 700
        elif throttle < 0.1 and error_vel < 0.:
            throttle = 0.0
            decel = max(error_vel, self.decel_limit)
            brake = abs(decel) * self.total_mass * self.wheel_radius

        return throttle, brake, steering
