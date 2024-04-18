import pybullet as pb
import numpy as np

class Encoder:
        def __init__(self, client):
            self.yaw_vel = np.zeros(3)
            self.client = client


        def read(self, n):
            yaw_joint = pb.getJointState(1,2, physicsClientId=self.client)
            pitch_joint = pb.getJointState(1,3, physicsClientId=self.client)
            return np.array([[0, pitch_joint[0], yaw_joint[0]], [0, pitch_joint[1], yaw_joint[1]]])
