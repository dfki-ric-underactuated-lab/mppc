import numpy as np
import pybullet as pb
import csv

class Endeffector:
    def __init__(self, timesteps, path):
        self.path = "/".join([path, "footData.csv"])
        self.X = np.zeros(timesteps)
        self.Y = np.zeros(timesteps)
        self.Z = np.zeros(timesteps)
        self.park_x = np.zeros(timesteps)


    def calc_xpos(self, n, client_se, yaw_pos, pitch_pos, hip_pos, knee_pos):
        robot = 0
        pb.resetJointState(bodyUniqueId=robot, jointIndex=2, targetValue=yaw_pos, physicsClientId=client_se)
        pb.resetJointState(robot, 3, pitch_pos, physicsClientId=client_se)
        pb.resetJointState(robot, 10, hip_pos, physicsClientId=client_se)
        pb.resetJointState(robot, 11, knee_pos, physicsClientId=client_se)
        linkWorldPosition = pb.getLinkState(robot, 12, computeForwardKinematics=True, physicsClientId=client_se)[0]
        self.X[n] = linkWorldPosition[0]
        self.Y[n] = linkWorldPosition[1]
        self.Z[n] = linkWorldPosition[2]
        if self.Y[n] < 0:
            yaw = np.arctan(self.X[n]/-self.Y[n]) + np.floor((yaw_pos + np.radians(180))/np.radians(360)) * np.radians(360)
        elif self.Y[n] == 0:
            yaw = np.sign(self.X[n]) * np.radians(90) + np.floor((yaw_pos + np.radians(180))/np.radians(360)) * np.radians(360)
        elif self.Y[n] > 0:
            yaw = np.arctan(self.X[n]/-self.Y[n]) + np.radians(180) + np.floor(yaw_pos/np.radians(360)) * np.radians(360)
        self.park_x[n] = 7.2 * yaw/(2 * np.pi)
        return self.park_x[n]


    def save(self, record, n):
        with open(self.path, 'w', newline='') as csvfile:
            fieldnames = ['time', 'phase', 'foot_x', 'foot_y', 'foot_z', 'parkour_x']
            data = [{'time': record.t[i], 'phase': record.phase[i], 'foot_x': self.X[i], 'foot_y': self.Y[i], 'foot_z': self.Z[i], 
                     'parkour_x': self.park_x[i]} for i in range(len(self.X[:n+1]))]
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(data)
    
    
    def calc_xpos_test(yaw_pos, X, Y):
        if Y < 0:
            yaw = np.arctan(X/-Y) + np.floor((yaw_pos + np.radians(180))/np.radians(360)) * np.radians(360)
        elif Y == 0:
            yaw = np.sign(X) * np.radians(90) + np.floor((yaw_pos + np.radians(180))/np.radians(360)) * np.radians(360)
        elif Y > 0:
            yaw = np.arctan(X/-Y) + np.radians(180) + np.floor(yaw_pos/np.radians(360)) * np.radians(360)
        print(f"Corrected Yaw Angle: {np.degrees(yaw)}")
        return 7.2 * yaw/(2 * np.pi)


    def yaw_dist(yaw, deviation=5):
        r = 1.15
        dist = np.radians(np.random.rand(1)[0] * 2 * deviation - deviation)
        print(f"Disturbance: {np.degrees(dist)}")
        x = np.sin(yaw + dist) * r
        y = - np.cos(yaw + dist) * r
        return yaw, x, y