import numpy as np
from pyrock import NameService, RTT
import time
import csv


class Encoder:

    def __init__(self, path):
        ns = NameService()
        task = ns.get_task_context("joints_drv")
        self.reader = task.reader("joints_status")
        self.timesteps = []
        self.pitch_pos = []
        self.yaw_pos = []
        self.pitch_vel = []
        self.yaw_vel = []
        time.sleep(0.1)
        status, data = self.reader.read(copy_old_data=True, return_status=True)
        self.stick_state = np.zeros((2, len(data["names"])))
        self.calibration = data
        self.state_path = "/".join([path, 'stateData.csv'])
        self.encoder_path = "/".join([path, 'encoderData.csv'])

    def read(self, n):
        status, data = self.reader.read(copy_old_data=True, return_status=True)
        for i in range(len(data["names"])):
            self.stick_state[0, i] = -(data["elements"][i]["position"] - self.calibration["elements"][i]["position"])
            self.stick_state[1, i] = data["elements"][i]["speed"]
        self.timesteps.append(n)
        self.pitch_pos.append(self.stick_state[0, 1])
        self.pitch_vel.append(self.stick_state[1, 1])
        self.yaw_pos.append(self.stick_state[0, 2])
        self.yaw_vel.append(self.stick_state[1, 2])
        return self.stick_state

    def save(self, record, n):
        # encoder data
        encoder_data = list(zip(self.timesteps, self.pitch_pos, self.pitch_vel, self.yaw_pos, self.yaw_vel))
        with open(self.encoder_path, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['n', 'pitch_pos', 'pitch_vel', 'yaw_pos', 'yaw_vel'])
            writer.writerows(encoder_data)
        
        # state data
        time = record.t
        phase = record.phase
        hpm, hvm, htm = record.matrix_msr(0)
        hpd, hvd, htd = record.matrix_des(0)
        kpm, kvm, ktm = record.matrix_msr(1)
        kpd, kvd, ktd = record.matrix_des(1)
        state_data = list(zip(time[:n+1], phase[:n+1], hpm[:n+1], hvm[:n+1], htm[:n+1], 
                              htd[:n+1], kpm[:n+1], kvm[:n+1], ktm[:n+1], ktd[:n+1]))
        with open(self.state_path, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['time', 'phase', 'hip_pos', 'hip_vel', 'hip_tau', 'hip_tau_des',
                             'knee_pos', 'knee_vel', 'knee_tau', 'knee_tau_des'])
            writer.writerows(state_data)