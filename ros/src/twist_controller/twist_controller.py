import rospy

from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, vehicle_mass, fuel_capacity, brake_deadband,
                 decel_limit, accel_limit, wheel_radius, wheel_base, steer_ratio,
                 speed_kp, accel_kp, accel_ki, max_lat_accel, accel_tau,
                 max_steer_angle):
        self.vehicle_mass = vehicle_mass
        self.fuel_capacity = fuel_capacity
        self.brake_deadband = brake_deadband
        self.decel_limit = decel_limit
        self.accel_limit = accel_limit
        self.wheel_radius = wheel_radius
        self.wheel_base = wheel_base
        self.steer_ratio = steer_ratio

        self.last_velocity = 0.

        self.lpf_fuel = LowPassFilter(60.0, 0.1)
        self.lpf_accel = LowPassFilter(accel_tau, 0.02)

        self.accel_pid = PID(accel_kp, accel_ki, 0.0, 0.0, 1.0)
        self.speed_pid = PID(speed_kp, 0.0, 0.0, self.decel_limit,
                             self.accel_limit)

        self.yaw_control = YawController(wheel_base, steer_ratio, 4. * ONE_MPH,
                                         max_lat_accel, max_steer_angle)
        self.last_ts = None

    def set_fuel(self, level):
        self.lpf_fuel.filt(level)

    def get_vehicle_mass(self):
        return self.vehicle_mass + \
               self.lpf_fuel.get() / 100.0 * self.fuel_capacity * GAS_DENSITY

    def time_elasped(self, msg=None):
        now = rospy.get_time()
        if self.last_ts is None:
            self.last_ts = now
        elasped, self.last_ts = now - self.last_ts,  now
        return elasped

    def control(self, linear_velocity, angular_velocity, current_velocity,
                dbw_enbaled):
        time_elasped = self.time_elasped()
        if time_elasped > 1./5 or time_elasped < 1e-4:
            self.speed_pid.reset()
            self.accel_pid.reset()
            self.last_velocity = current_velocity
            return 0., 0., 0.

        vehicle_mass = self.get_vehicle_mass()
        vel_error = linear_velocity - current_velocity

        if abs(linear_velocity) < ONE_MPH:
            self.speed_pid.reset()

        accel_cmd = self.speed_pid.step(vel_error, time_elasped)

        min_speed = ONE_MPH * 5
        if linear_velocity < 0.01:
            accel_cmd = min(accel_cmd,
                            -530. / vehicle_mass / self.wheel_radius)
        elif linear_velocity < min_speed:
            angular_velocity *= min_speed/linear_velocity
            linear_velocity = min_speed

        accel = (current_velocity - self.last_velocity) / time_elasped
        #rospy.logwarn("Current accel = %f, Last = %f, New = %f", accel, self.last_velocity, current_velocity)
        self.lpf_accel.filt(accel)
        self.last_velocity = current_velocity

        throttle, brake, steering = 0., 0., 0.
        if dbw_enbaled:
            if accel_cmd >= 0:
                throttle = self.accel_pid.step(accel_cmd - self.lpf_accel.get(),
                                               time_elasped)
            else:
                self.accel_pid.reset()
            if (accel_cmd < -self.brake_deadband) or \
               (linear_velocity < min_speed):
              brake = -accel_cmd * vehicle_mass * self.wheel_radius

            steering = self.yaw_control.get_steering(linear_velocity,
                                                     angular_velocity,
                                                     current_velocity)
        else:
            self.speed_pid.reset()
            self.accel_pid.reset()
        return throttle, brake, steering

