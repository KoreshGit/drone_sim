"""This file aims to simulate some of the senors for the drone"""

import numpy as np
from drone_sim.sim.parameters import *

class Sensor:
    def __init__(self):
        self.drone = None

    def attach_to(self, drone):
        raise NotImplementedError

    def sense(self):
        raise NotImplementedError

    def name(self):
        return self.__class__.__name__

class PositionTracker(Sensor):
    def __init__(self, add_noise=True):
        super(PositionTracker, self).__init__()

        self.drone = None

        if self.add_noise:
            self.mean = GPS_MEAN
            self.std_dev = GPS_STDDEV
        else:
            self.mean = 0
            self.std_dev = 0

    def attach_to(self, drone):
        self.drone = drone
        self.drone.attach_sensor(self)

    def sense(self):
        self.x = self.drone.x + (self.mean + np.random.randn() * self.std_dev)
        self.y = self.drone.y + (self.mean + np.random.randn() * self.std_dev)
        self.z = self.drone.z + (self.mean + np.random.randn() * self.std_dev)

        return [self.x, self.y, self.z]


class StatusTracker(Sensor):
    def __init__(self, add_noise=True):
        super(StatusTracker, self).__init__()


    def attach_to(self, drone):
        self.drone = drone
        self.drone.attach_sensor(self)

    def sense(self):
        state = {}
        state['linear_position'] = lambda: np.array([self.drone.x, self.drone.y, self.drone.z]).reshape(3, 1)
        state['angular_position'] = lambda: np.array([self.drone.phi, self.drone.theta, self.drone.psi]).reshape(3, 1)
        state['linear_velocity'] = lambda: np.array([self.drone.vx, self.drone.vy, self.drone.vz]).reshape(3, 1)
        state['angular_velocity'] = lambda: np.array([self.drone.p, self.drone.q, self.drone.r]).reshape(3, 1)
        return state



class IMU(Sensor):
    def __init__(self, add_noise=True):
        super(IMU, self).__init__()

        self.drone = None
        self.add_noise = add_noise
        if self.add_noise:
            self.initialise_random_walks()
        else:
            self.gyrox_bias = 0
            self.gyroy_bias = 0
            self.gyroz_bias = 0

    def initialise_random_walks(self):
        self.gyrox_bias = 0
        self.gyroy_bias = 0
        self.gyroz_bias = 0

    def update_bias(self):
        self.gyrox_bias += (IMU_MEANS["gyrox"] + np.random.randn() * IMU_STDDEV["gyrox"])*DT
        self.gyroy_bias += (IMU_MEANS["gyroy"] + np.random.randn() * IMU_STDDEV["gyroy"])*DT
        self.gyroz_bias += (IMU_MEANS["gyroz"] + np.random.randn() * IMU_STDDEV["gyroz"])*DT

    def attach_to(self, drone):
        self.drone = drone
        self.drone.attach_sensor(self)

    def sense(self):
        if self.add_noise:
            # If noise is added, gyro's drift has a random walk.
            wx = self.drone.p + self.gyrox_bias
            wy = self.drone.q + self.gyroy_bias
            wz = self.drone.r + self.gyroz_bias

            ax = self.drone.acceleration[0]
            ay = self.drone.acceleration[1]
            az = self.drone.acceleration[2]

            # Update bias
            self.update_bias()

        else:
            wx = self.drone.p
            wy = self.drone.q
            wz = self.drone.r

            ax = self.drone.acceleration[0]
            ay = self.drone.acceleration[1]
            az = self.drone.acceleration[2]

        return [ax, ay, az], [wx, wy, wz]
