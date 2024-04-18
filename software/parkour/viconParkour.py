from shapely.geometry import Point, LineString, Polygon
from shapely.affinity import rotate
import numpy as np
import json
import copy
import zmq
import csv

class vicon_parkour:
    def __init__(self, path, Parkour):
        self.vicon_path = "/".join([path, "viconData.csv"])
        # zmq
        context = zmq.Context()
        self.socket = context.socket(zmq.PULL)
        self.socket.connect("tcp://127.0.0.1:5555")
        self.base = self.socket.recv_json()
        self.obs1_dim = [100, 100, 100]
        self.obs2_dim = [100, 200, 300]
        self.data = {"obs_1_pos": [],
                     "obs_1_rot": [],
                     "obs_2_pos": [],
                     "obs_2_rot": []}
        self.data_struct = []

        self.Parkour = Parkour
        theta = np.linspace(0, 2*np.pi, 200)
        radius = 1.145915 * 1000
        x = radius * -np.cos(theta)
        y = radius * -np.sin(theta)
        self.runway = LineString(zip(x, y))


    def track(self):
        try:
            message = json.loads(self.socket.recv_json(flags=zmq.NOBLOCK))
            if message:
                self.data["obs_1_pos"] = np.array(message[0][:3]) - np.array(self.base)
                self.data["obs_1_rot"] = np.array(message[0][3:])
                self.data["obs_2_pos"] = np.array(message[1][:3]) - np.array(self.base)
                self.data["obs_2_rot"] = np.array(message[1][3:])
                self.data_struct.append(copy.deepcopy(self.data))
        except zmq.Again:
            self.data_struct.append(self.data_struct[-1])


    def parkour_mod(self, x_start):
        self.Parkour["position"] = np.delete(self.Parkour["position"], self.Parkour["vicon"].tolist())
        self.Parkour["height"] = np.delete(self.Parkour["height"], self.Parkour["vicon"].tolist())
        self.Parkour["width"] = np.delete(self.Parkour["width"], self.Parkour["vicon"].tolist())
        self.Parkour["vicon"] = np.array([])

        # Obstacle 1
        dx = self.obs1_dim[0] / 2
        dy = self.obs1_dim[1] / 2
        obstacle_points = [(self.data["obs_1_pos"][0] - dx, self.data["obs_1_pos"][1] - dy),
                           (self.data["obs_1_pos"][0] + dx, self.data["obs_1_pos"][1] - dy),
                           (self.data["obs_1_pos"][0] + dx, self.data["obs_1_pos"][1] + dy),
                           (self.data["obs_1_pos"][0] - dx, self.data["obs_1_pos"][1] + dy)]
        obstacle = rotate(Polygon(obstacle_points), self.data["obs_1_rot"][2], origin='center', use_radians=True)
        b = obstacle.boundary.coords
        obstacle_linestrings = [LineString(b[k:k+2]) for k in range(len(b) - 1)]
        intersection = self.runway.intersection(obstacle_linestrings)
        waypoint = np.zeros(2)
        i = 0
        for point in intersection:
            if not point.is_empty:
                phi = np.arctan2(-point.y, -point.x)
                if np.sign(phi) == -1:
                    phi += 2 * np.pi
                waypoint[i] = phi * 7.2 / (2 * np.pi)
                i += 1
        if waypoint.all() == 0:
            print("Obstacle 1 is not part of the parkour.")
            index = -2
        else:
            waypoint.sort()
            if self.Parkour["goal_x"] == 14.4 and x_start >= waypoint[1]:
                waypoint += 7.2
            print(f"Obstacle 1 is part of the parkour at x = {waypoint[0] + (waypoint[1] - waypoint[0]) / 2}"
                  f"with a width of {waypoint[1] - waypoint[0]}.")
            x_pos = waypoint[0] + (waypoint[1] - waypoint[0]) / 2
            index = np.searchsorted(self.Parkour["position"], x_pos)
            self.Parkour["position"] = np.insert(self.Parkour["position"], index, np.round(x_pos, 3))
            self.Parkour["height"] = np.insert(self.Parkour["height"], index, np.round(self.obs1_dim[2]/1000, 2))
            self.Parkour["width"] = np.insert(self.Parkour["width"], index, np.round(waypoint[1] - waypoint[0], 3))

        # Obstacle 2
        dx = self.obs2_dim[0] / 2
        dy = self.obs2_dim[1] / 2
        obstacle_points = [(self.data["obs_2_pos"][0] - dx, self.data["obs_2_pos"][1] - dy),
                           (self.data["obs_2_pos"][0] + dx, self.data["obs_2_pos"][1] - dy),
                           (self.data["obs_2_pos"][0] + dx, self.data["obs_2_pos"][1] + dy),
                           (self.data["obs_2_pos"][0] - dx, self.data["obs_2_pos"][1] + dy)]
        obstacle = rotate(Polygon(obstacle_points), self.data["obs_2_rot"][2], origin='center', use_radians=True)
        b = obstacle.boundary.coords
        obstacle_linestrings = [LineString(b[k:k+2]) for k in range(len(b) - 1)]
        intersection = self.runway.intersection(obstacle_linestrings)
        waypoint = np.zeros(2)
        i = 0
        for point in intersection:
            if not point.is_empty:
                phi = np.arctan2(-point.y, -point.x)
                if np.sign(phi) == -1:
                    phi += 2 * np.pi
                waypoint[i] = phi * 7.2 / (2 * np.pi)
                i += 1
        if waypoint.all() == 0:
            print("Obstacle 2 is not part of the parkour.")
            index_2 = -2
        else:
            waypoint.sort()
            if self.Parkour["goal_x"] == 14.4 and x_start >= waypoint[1]:
                waypoint += 7.2
            print(f"Obstacle 2 is part of the parkour at x = {waypoint[0] + (waypoint[1] - waypoint[0]) / 2}"
                  f"with a width of {waypoint[1] - waypoint[0]}.")
            x_pos = waypoint[0] + (waypoint[1] - waypoint[0]) / 2
            index_2 = np.searchsorted(self.Parkour["position"], x_pos)
            self.Parkour["position"] = np.insert(self.Parkour["position"], index_2, np.round(x_pos, 3))
            self.Parkour["height"] = np.insert(self.Parkour["height"], index_2, np.round(self.obs2_dim[2]/1000, 2))
            self.Parkour["width"] = np.insert(self.Parkour["width"], index_2, np.round(waypoint[1] - waypoint[0], 3))

        if index_2 > index:
            a = np.array([index, index_2])
            self.Parkour["vicon"] = a[a>=0]
        elif index_2 >= 0:
            a = np.array([index_2, index+1])
            self.Parkour["vicon"] = a[a>=0]
        else:
            a = np.array([index_2, index])
            self.Parkour["vicon"] = a[a>=0]
        return self.Parkour
    
    def save(self):
        with open(self.vicon_path, 'w', newline='') as csvfile:
            fieldnames = self.data_struct[0].keys()
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            writer.writeheader()
            for data in self.data_struct:
                writer.writerow(data)