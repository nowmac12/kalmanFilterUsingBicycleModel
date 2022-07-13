import numpy as np
import math

# ERP-42 Parameters
max_steer = np.radians(28.0)  # [rad] max steering angle
L = 1.04  # [m] Wheel base of vehicle
Lr = L / 2.0  # [m]
Lf = L - Lr
LENGTH = 2.02 # [m]
WIDTH = 1.16  # [m]

Cf = 1600.0 * 2.0  # N/rad
Cr = 1700.0 * 2.0  # N/rad
  # kg/m2
m = 400.0  # kg
Iz = 1500#m*(LENGTH**2 + WIDTH**2)/12
print(Iz)
# dt = 0.05

# non-linear lateral bicycle model
class NonLinearBicycleModel():
    def __init__(self, x=0.0, y=0.0, yaw=0.0, vx=0.01, vy=0, omega=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.vx = vx
        self.vy = vy
        self.omega = omega
        # Aerodynamic and friction coefficients
        self.c_a = 0.001
        self.c_r1 = 0.1

    def update(self, throttle, delta, dt):
        delta = np.clip(delta, -max_steer, max_steer)
        self.x = self.x + self.vx * math.cos(self.yaw) * dt - self.vy * math.sin(self.yaw) * dt
        self.y = self.y + self.vx * math.sin(self.yaw) * dt + self.vy * math.cos(self.yaw) * dt
        self.yaw = self.yaw + self.omega * dt
        self.yaw = normalize_angle(self.yaw)
        Ffy = -Cf * math.atan2(((self.vy + Lf * self.omega) / self.vx - delta), 1.0)
        Fry = -Cr * math.atan2((self.vy - Lr * self.omega) / self.vx, 1.0)
        R_x = self.c_r1 * self.vx
        F_aero = self.c_a * self.vx ** 2
        F_load = F_aero + R_x
        self.vx = self.vx + (throttle - Ffy * math.sin(delta) / m - F_load/m + self.vy * self.omega) * dt
        self.vy = self.vy + (Fry / m + Ffy * math.cos(delta) / m - self.vx * self.omega) * dt
        self.omega = self.omega + (Ffy * Lf * math.cos(delta) - Fry * Lr) / Iz * dt


# reference: https://www.coursera.org/lecture/intro-self-driving-cars/lesson-2-the-kinematic-bicycle-model-Bi8yE,
# we used the "Rear Alex Bicycle model" as mentioned in that tutorial. TODO: update Read.me
class LinearBicycleModel(object):
    """
    Class representing the state of a vehicle.
    :param x: (float) x-coordinate
    :param y: (float) y-coordinate
    :param yaw: (float) yaw angle
    :param v: (float) speed
    """

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        """Instantiate the object."""
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v

    def update(self, throttle, delta, dt):
        """
        Update the state of the vehicle.
        Stanley Control uses bicycle model.
        :param a: (float) Acceleration
        :param delta: (float) Steering
        """
        delta = np.clip(delta, -max_steer, max_steer)
        self.x += self.v * np.cos(self.yaw) * dt
        self.y += self.v * np.sin(self.yaw) * dt
        self.yaw += self.v / L * np.tan(delta) * dt
        self.yaw = normalize_angle(self.yaw)
        self.v += throttle * dt


def normalize_angle(angle):
    """
    Normalize an angle to [-pi, pi].
    :param angle: (float)
    :return: (float) Angle in radian in [-pi, pi]
    """
    while angle > np.pi:
        angle -= 2.0 * np.pi

    while angle < -np.pi:
        angle += 2.0 * np.pi

    return angle
