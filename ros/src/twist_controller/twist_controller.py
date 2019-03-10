import rospy
from pid import PID
from yaw_controller import YawController
from lowpass import LowPassFilter

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, wheel_base, steer_ratio,max_lat_accel, max_steer_angle,vehicle_mass,wheel_radius):
        # TODO: Implement
        self.yaw_controller = YawController(wheel_base, steer_ratio, 0.1, max_lat_accel, max_steer_angle)

        kp = 0.3
        ki = 0.1
        kd = 0
        mn = 0
        mx = 0.2
        self.throttle_controller = PID(kp,ki,kd,mn,mx)

        tau = 0.5
        ts = 0.02
        self.vel_lpf = LowPassFilter(tau,ts)

        self.last_time = rospy.get_time()
        self.vehicle_mass = vehicle_mass
        self.wheel_radius = wheel_radius
        pass

    def control(self, current_val, linear_val, angular_val, dbw_enable,decel_limit):
        # TODO: Change the arg, kwarg list to suit your needs
        if not dbw_enable:
            self.throttle_controller.reset()
            return 0,0,0
        current_val = self.vel_lpf.filt(current_val)
        self.last_val = current_val

        steering = self.yaw_controller.get_steering(linear_val,angular_val,current_val)

        current_time = rospy.get_time()
        sample_time = current_time - self.last_time
        self.last_time = current_time

        error_val = linear_val - current_val
        throttle = self.throttle_controller.step(error_val,sample_time)
        brake = 0
        if linear_val == 0 and current_val < 0.1:
            brake = 700
        elif throttle < 0.1 and error_val < 0:
            throttle = 0
            decel = max(error_val,decel_limit)
            brake = abs(decel)*self.vehicle_mass*self.wheel_radius

        # Return throttle, brake, steer
        return throttle, brake, steering
