
from control.lowpass import LowPassFilter
from math import atan, isclose
from control.pid import PID
import rospy

MINIMUM_VELOCITY = 0.1

class SteeringAngleController():
    def __init__(self, wheel_base, steering_ratio, max_steering_angle, max_lateral_acc):
        self.wheel_base = wheel_base
        self.steering_ratio = steering_ratio
        self.max_lateral_acc = max_lateral_acc
        self.max_steering_angle = max_steering_angle
    
    def control(self, current_linear_velocity, target_linear_velocity, target_angular_velocity):
        # 1. Compute angular velocity:
        #   - assume angular velocity / linear velocity = target angular velocity / target linear velocity
        #   - make sure to cap angular velocity to avoid exceeeding max lateral acceleration (clip angular velocity to [-max_angular_vel, max_angular_vel])
        #   - Compute max allowed angular velocity from given max allowed lateral acceleration using uniform circular motion equations 
        if abs(current_linear_velocity) > 0. and target_linear_velocity > 0:
            current_angular_velocity = current_linear_velocity * (target_angular_velocity / target_linear_velocity)

            if abs(current_linear_velocity) > MINIMUM_VELOCITY:
                max_angular_vel = abs(self.max_lateral_acc / current_linear_velocity)
                current_angular_velocity = max(min(max_angular_vel, target_angular_velocity), -max_angular_vel)
        else:
            current_angular_velocity = 0    
        
        # 2. Compute the vehicle steering angle:
        #   - if angular velocity is 0, the angle should also be 0
        #   - compute the turning radius from angular velocity using circular motion equations
        #   - use the bicycle model equations to compute the steering angle corresponding to the turning radius
        # curent speed v = r * angular speed
        radius = current_linear_velocity / current_angular_velocity if current_angular_velocity else 0.0
        vehicle_steering_angle = atan(self.wheel_base / radius) if radius else 0.0
        
        # 3. Compute the *steering wheel* angle:
        #   - steering wheel angle = steering angle * steering ratio
        steering_wheel_angle = vehicle_steering_angle * self.steering_ratio
        #   - clip the steering wheel angle to [-max steering wheel angle, max steering wheel angle]
        steering_wheel_angle = max(min(self.max_steering_angle, steering_wheel_angle), -self.max_steering_angle)
        
        return steering_wheel_angle

class ThrottleBrakeController():
    def __init__(self, car_mass, wheel_radius, deceleration_limit):
        self.lowpass_filter = LowPassFilter(tau=0.5, ts=.02)
        self.car_mass = car_mass
        self.deceleration_limit = deceleration_limit
        self.wheel_radius = wheel_radius
        self.last_time = rospy.get_time()
        k_p = 1
        k_i = 0.00001
        k_d = 0.5
        self.pid_controller = PID(k_p, k_i, k_d)

    def control(self, current_speed, target_speed, current_time):
        # If the target speed is less than the current, the car should break
        if target_speed < current_speed:
            deceleration = (target_speed - current_speed)
            # Bound the deceleration (use max as deceleration is a negative number)
            deceleration = max(self.deceleration_limit, deceleration)
            brake = abs(deceleration * self.car_mass * self.wheel_radius)
            return 0., brake
        else:
            update_rate = current_time - self.last_time
            self.last_time = current_time
            current_speed = self.lowpass_filter.filter(current_speed) # Use LowPass filter to filter noise from velocity
            cte_velocity_err = target_speed - current_speed
            throttle = self.pid_controller.step(cte_velocity_err, update_rate)
            # Bound throttle
            throttle =  max(min(1, throttle), 0)
            return throttle, 0.
