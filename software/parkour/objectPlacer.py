import pybullet as pb
import numpy as np


def marker_placement(client, U):
    marker_urdf = 'model/urdf/marker.urdf'
    spacing = 0.2
    mark = 0
    r = U / (2 * np.pi) + 0.05
    for i in range(36):
        share = mark / U
        rot = 2 * np.pi * share
        x = np.sin(rot) * r
        y = - np.cos(rot) * r
        marker = pb.loadURDF(marker_urdf,
                             [x, y, 0.001],
                             baseOrientation=pb.getQuaternionFromEuler([0, 0, rot]),
                             physicsClientId=client)
        mark += spacing


def area_placement(client, U, Parkour, second_round):
    area_objects = []
    pos = Parkour["res_position"]
    for i in range(len(pos[pos>U if second_round else pos<U])):
        if Parkour["res_width"][i] == 0.4:
            area_urdf = 'model/urdf/area_40cm.urdf'
        else:
            raise Exception("-- THE CHOOSEN PARKOUR IS UNFEASIBLE FOR CURRENT DEVELOPMENT STATE--")
        rot = 2 * np.pi * ((Parkour["res_position"][i] - Parkour["res_width"][i]/2) / U) - np.pi/2
        area = pb.loadURDF(area_urdf,
                           [0, 0, -0.0495],
                           baseOrientation=pb.getQuaternionFromEuler([0, 0, rot]),
                           physicsClientId=client)
        area_objects.append(area)
    return area_objects


def vicon_placement(client, vicon_data, vicon_color, vicon_objects=[]):
    for i in vicon_objects:
        pb.removeBody(i, physicsClientId=client)
    vicon_1_urdf = 'model/urdf/box_vicon_1.urdf'
    vicon_2_urdf = 'model/urdf/box_vicon_2.urdf'
    vicon_data["obs_1_pos"][2] = max(50, vicon_data["obs_1_pos"][2])
    vicon_data["obs_2_pos"][2] = max(150, vicon_data["obs_2_pos"][2])
    vicon_1 = pb.loadURDF(vicon_1_urdf,
                          vicon_data["obs_1_pos"]/1000,
                          baseOrientation=pb.getQuaternionFromEuler(vicon_data["obs_1_rot"]),
                          physicsClientId=client)
    if vicon_color[0] ==  'g':
        pb.changeVisualShape(vicon_1, 0, rgbaColor=[0.1294117647, 0.7647058823, 0.43529411764, 1.0])
    elif vicon_color[0] ==  'r':
        pb.changeVisualShape(vicon_1, 0, rgbaColor=[0.9, 0.2, 0.2, 1])
    vicon_2 = pb.loadURDF(vicon_2_urdf,
                          vicon_data["obs_2_pos"]/1000,
                          baseOrientation=pb.getQuaternionFromEuler(vicon_data["obs_2_rot"]),
                          physicsClientId=client)
    if vicon_color[1] ==  'g':
        pb.changeVisualShape(vicon_2, 0, rgbaColor=[0.1294117647, 0.7647058823, 0.43529411764, 1.0])
    elif vicon_color[1] ==  'r':
        pb.changeVisualShape(vicon_2, 0, rgbaColor=[0.9, 0.2, 0.2, 1])
    
    return [vicon_1, vicon_2]


def obstacle_placement(client, U, Parkour, second_round):
    obstacle_objects = []
    r = U / (2 * np.pi) + 0.02
    pos = np.delete(Parkour["position"], Parkour["vicon"].tolist())
    displacement = 0
    for i in range(len(pos[pos>U if second_round else pos<U])):
        width = np.delete(Parkour["width"], Parkour["vicon"].tolist())[i]
        if width == 0.1:
            obstacle_urdf = 'model/urdf/box_10cm.urdf'
        elif width == 0.2:
            obstacle_urdf = 'model/urdf/box_20cm.urdf'
        elif width == 0.3:
            obstacle_urdf = 'model/urdf/box_30cm.urdf'
        else:
            raise Exception("-- THE CHOOSEN PARKOUR IS UNFEASIBLE FOR THE CURRENT DEVELOPMENT STATE--")
        share = pos[i] / U
        rot = 2 * np.pi * share
        x = np.sin(rot) * (r + displacement)
        y = - np.cos(rot) * (r + displacement)
        z = np.delete(Parkour["height"], Parkour["vicon"].tolist())[i] - 0.5
        if z == 0.2:
            z += 0.02
        elif z == 0.3:
            displacement += 0.04

        obstacle = pb.loadURDF(obstacle_urdf,
                                [x, y, z],
                                baseOrientation=pb.getQuaternionFromEuler([0, 0, rot]),
                                physicsClientId=client)
        obstacle_objects.append(obstacle)
    return obstacle_objects


def contact_placement(client, U, contact_x, contact_z, point_objects=[]):
    for i in point_objects:
        pb.removeBody(i, physicsClientId=client)
    contact_urdf = 'model/urdf/contact.urdf'
    contact_points = []
    r = U / (2 * np.pi)
    for i in range(len(contact_x)):
        if contact_z[i] <= 0.05:
            displacement = - 0.02
        elif contact_z[i] >= 0.25:
            displacement = 0.04 
        else:
            displacement = 0.02
        rot = 2 * np.pi * contact_x[i] / U
        x = np.sin(rot) * (r + displacement)
        y = - np.cos(rot) * (r + displacement)
        contact = pb.loadURDF(contact_urdf,
                              [x, y, contact_z[i]-0.0001],
                              baseOrientation=pb.getQuaternionFromEuler([0, 0, rot]),
                              physicsClientId=client)
        contact_points.append(contact)
    return contact_points


def area_color(U, Parkour, areas, area_objects, second_round):
    pos = Parkour["res_position"]
    k = 0
    for i in range(len(pos[pos>U if second_round else pos<U])):
        if second_round:
            i += len(pos[pos<=U])
        if i in areas:
            pb.changeVisualShape(area_objects[k], 0, rgbaColor=[0.9, 0.2, 0.2, 1])
        else:
            pb.changeVisualShape(area_objects[k], 0, rgbaColor=[0.4196078431372549, 0.48627450980392156, 0.5215686274509804, 1.0])
        k += 1


def obstacle_color(U, Parkour, obstacles, obstacle_objects, second_round=False):
    k = 0
    vicon_color = [[], []]
    pos = Parkour["position"]
    for i in range(len(pos[pos > U if second_round else pos < 7.2])):
        if second_round:
            i += len(pos[pos<=U])
        if i in Parkour["vicon"].tolist():
            if i in obstacles:
                vicon_color[0 if Parkour["height"][i] == 0.1 else 1] = 'g'
                for j in range(len(Parkour["res_position"])):
                    if (Parkour["res_position"][j] - Parkour["res_width"][j]/2) <= Parkour["position"][i] <= (Parkour["res_position"][j] + Parkour["res_width"][j]/2):
                        vicon_color[0 if Parkour["height"][i] == 0.1 else 1] = 'r'
            k -= 1
        else:
            if i in obstacles:
                pb.changeVisualShape(obstacle_objects[k], 0, rgbaColor=[0.1294117647, 0.7647058823, 0.43529411764, 1.0])
                for j in range(len(Parkour["res_position"])):
                    if (Parkour["res_position"][j] - Parkour["res_width"][j]/2) <= Parkour["position"][i] <= (Parkour["res_position"][j] + Parkour["res_width"][j]/2):
                        pb.changeVisualShape(obstacle_objects[k], 0, rgbaColor=[0.9, 0.2, 0.2, 1])
            else:
                pb.changeVisualShape(obstacle_objects[k], 0, rgbaColor=[0.4196078431372549, 0.48627450980392156, 0.5215686274509804, 1.0])
        k += 1
    return vicon_color