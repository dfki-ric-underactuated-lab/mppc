import json
import numpy as np
from dataclasses import dataclass, field


def initialize_hopper(json_path):
    with open(json_path, 'r') as file_data:
        parameters = json.load(file_data)
    return Hopper(Boomstick(**parameters['Boomstick']), Leg(**parameters['Leg']),
                  Limit(**parameters['Limit']), Margin(**parameters['Margin']))

@dataclass
class Boomstick(object):
    circumference: float
    height: float

@dataclass
class Leg(object):
    mass: float
    length_link_1: float
    length_link_2: float
    r_touchdown: float
    r_staging: float
    r_takeoff: float
    r_flight: float
    theta_flight: float

@dataclass
class Limit(object):
    t_min: float
    t_max: float
    v_min: float
    v_max: float
    theta_min: float
    theta_max: float

@dataclass
class Margin(object):
    x_knee: float

@dataclass
class Hopper(object):
    boomstick: Boomstick
    leg: Leg
    limit: Limit
    margin: Margin
    dx_hip_foot: float = field(init=False)
    dz_hip_foot: float = field(init=False)
    dx_hip_knee: float = field(init=False)
    dz_hip_knee: float = field(init=False)
    gravity: float = 8.0665

    def __post_init__(self):
        self.dx_hip_foot = - self.leg.r_flight * np.cos(np.radians(self.leg.theta_flight))
        self.dz_hip_foot = - self.leg.r_flight * np.sin(np.radians(self.leg.theta_flight))

        alpha = np.arccos((self.leg.length_link_1 ** 2 - self.leg.length_link_2 ** 2
                           - self.leg.r_flight ** 2) / (- 2 * self.leg.length_link_2 * self.leg.r_flight))

        phi = np.radians(180 - self.leg.theta_flight) - alpha
        self.dx_foot_knee = - self.leg.length_link_2 * np.cos(phi)
        self.dz_foot_knee = self.leg.length_link_2 * np.sin(phi)

    def jacobian(self, q1, q2):
        return np.array([[-self.leg.length_link_1 * np.sin(q1) 
                          - self.leg.length_link_2 * np.sin(q1+q2),
                          -self.leg.length_link_2 * np.sin(q1+q2)],
                          [self.leg.length_link_1 * np.cos(q1)
                           + self.leg.length_link_2 * np.cos(q1+q2),
                           self.leg.length_link_2 * np.cos(q1+q2)]])

    def kinematics_new_frame(self, r, theta):
        dr = np.arccos((self.leg.length_link_1**2 + self.leg.length_link_2**2 - r**2)
                       / (2 * self.leg.length_link_1 * self.leg.length_link_2))
        dL1 = np.arccos((r**2 + self.leg.length_link_2**2 - self.leg.length_link_1**2)
                        / (2 * r * self.leg.length_link_2))
        dL2 = np.radians(180) - dr - dL1
        q1 = theta - np.radians(90) + dL2
        q2 = dr - np.radians(180)
        print(f"q1, q2: {np.degrees([q1, q2])} deg")
        return q1, q2
    
    def forward_velocity(self, q1, q2, dq1, dq2):
        vx = (-dq1 * self.leg.length_link_1 * np.sin(q1)
              - (dq1 + dq2) * self.leg.length_link_2 * np.sin(q1+q2))
        vy = (dq1 * self.leg.length_link_1 * np.cos(q1)
              + (dq1 + dq2) * self.leg.length_link_2 * np.cos(q1+q2))
        return vx, vy
    
    def kinematics(self, r, theta):
        content = max(-1, (r**2 - self.leg.length_link_1**2 - self.leg.length_link_2**2) 
                      / (-2 * self.leg.length_link_1 * self.leg.length_link_2))
        dr = np.arccos(content)
        nenner = (-2 * r * self.leg.length_link_1)
        if nenner == 0:
            dL2 = np.radians(90)
        else:
            dL2 = np.arccos((self.leg.length_link_2**2 - r**2 - self.leg.length_link_1**2)
                            / (-2 * r * self.leg.length_link_1))
        q1 = theta - dL2 - np.radians(90)
        q2 = np.radians(180) - dr
        return q1, q2
    
    def inverse_kinematics(self, q1, q2):
        x = self.leg.length_link_1 * np.cos(q1) + self.leg.length_link_2 * np.cos(q1 + q2)
        y = self.leg.length_link_1 * np.sin(q1) + self.leg.length_link_2 * np.sin(q1 + q2)
        r = np.sqrt(x**2 + y**2)
        rem = np.remainder(q1 + np.radians(90), np.radians(360))
        if x >= 0:
            theta = np.radians(180) - np.arccos(y / r)
            if rem > np.radians(270):
                theta += np.radians(360)
        else:
            theta = np.radians(180) + np.arccos(y / r)
            if rem < np.radians(90):
                theta -= np.radians(360)
        theta += np.floor((q1 + np.radians(90)) / np.radians(360)) * np.radians(360)
        return r, theta