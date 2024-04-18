from scipy.spatial.transform import Rotation
import numpy as np
import csv
import re

def read_stateData(experiment):
    path = f'data/experiment/{experiment}/stateData.csv'
    t = []
    phase = []
    hip = []
    knee = []
    hip_tau = []
    knee_tau = []
    hip_tau_des = []
    knee_tau_des = []
    with open(path, 'r', newline='') as csvfile:
        reader = csv.DictReader(csvfile)
        for row in reader:
            t.append(float(row['time']))
            phase.append(str(row['phase']))
            hip.append(float(row['hip_pos']))
            knee.append(float(row['knee_pos']))
            hip_tau.append(float(row['hip_tau']))
            knee_tau.append(float(row['knee_tau']))
            hip_tau_des.append(float(row['hip_tau_des']))
            knee_tau_des.append(float(row['knee_tau_des']))
    return t, phase, hip, knee, hip_tau, knee_tau, hip_tau_des, knee_tau_des

def read_encoderData(experiment):
    path = f'data/experiment/{experiment}/encoderData.csv'
    pitch = []
    yaw = []
    with open(path, 'r', newline='') as csvfile:
        reader = csv.DictReader(csvfile)
        for row in reader:
            pitch.append(float(row['pitch_pos']) - 0.01)
            yaw.append(float(row['yaw_pos']))
    return pitch, yaw

def read_parkourData(experiment):
    path = f'data/experiment/{experiment}/parkourData.csv'
    parkours_list = []
    with open(path, 'r', newline='') as csvfile:
        reader = csv.DictReader(csvfile)
        for row in reader:
            for key in row:
                if key not in ['vicon', 'position', 'height', 'width', 'res_position', 'res_width']:
                    row[key] = float(row[key])
                elif key == 'vicon':
                    row[key] = np.array([int(num) for num in re.sub(r'\s+', ',', row[key].strip('[]')).split(',') if num])
                else:
                    row[key] = np.array([float(num) for num in re.sub(r'\s+', ',', row[key].strip('[]')).split(',') if num])
            parkours_list.append(row)
    return parkours_list

def read_resultData(experiment):
    path = f'data/experiment/{experiment}/resultData.csv'
    results_list = []
    with open(path, 'r', newline='') as csvfile:
        reader = csv.DictReader(csvfile)
        for row in reader:
            for key in row:
                if key in ['x_start', 'z_start']:
                    row[key] = float(row[key])
                elif key in ['obstacles', 'areas']:
                    row[key] = np.array([int(num) for num in re.sub(r'\s+', ',', row[key].strip('[]')).split(',') if num])
                else:
                    row[key] = np.array([float(num) for num in re.sub(r'\s+', ',', row[key].strip('[]')).split(',') if num])
                    
            results_list.append(row)
    return results_list

def read_viconData(experiment):
    path = f'data/experiment/{experiment}/viconData.csv'
    vicon_list = []
    r = Rotation.from_euler('z', 90, degrees=True)
    with open(path, 'r', newline='') as csvfile:
        reader = csv.DictReader(csvfile)
        for row in reader:
            for key in row:
                if key in ['obs_1_pos', 'obs_2_pos']:
                    data = np.array([float(num) for num in re.sub(r'\s+', ',', row[key].strip('[]')).split(',') if num])
                    row[key] = r.apply(data)
                else:
                    row[key] = np.array([float(num) for num in re.sub(r'\s+', ',', row[key].strip('[]')).split(',') if num])
                    row[key][2] += np.pi/2
            vicon_list.append(row)
    return vicon_list

def read_footData(experiment):
    with open(experiment, mode='r') as csv_file:
        reader = csv.DictReader(csv_file)
        EE_pos_x, EE_pos_z = [], []
        for row in reader:
            EE_pos_x.append(float(row['parkour_x']))
            EE_pos_z.append(float(row['foot_z']))
            EE_pos_z[-1] -= 0.01
    return EE_pos_x, EE_pos_z