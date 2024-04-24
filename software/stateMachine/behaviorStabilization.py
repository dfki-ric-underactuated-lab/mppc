import os
import sys
import toml
import time
import copy
import numpy as np
import pybullet as pb

from software.spine.controller.abstract import AbstractController
import software.util.prepare as pr
from software.optimization.behaviorGeneration import mppc
from software.parkour import objectPlacer
from software.parkour.viconParkour import vicon_parkour
from software.stateEstimation.tracker import Endeffector
from software.visualization.plot.plotter import Plotter
from software.util.save import Storage
from software.util.suppress import RedirectStream

class StateMachine(AbstractController):
    """
        STAGING    -> Launch    -> EXERTION
        EXERTION   -> Take-off  -> FLIGHT
        FLIGHT     -> Touchdown -> ABSORPTION
        ABSORPTION -> Landed    -> REPOSITIONING
        REPOSITION -> Alignment -> STAGING
    """

    flight_counter = 0
    q_backflip = np.asarray([0, 0], dtype=float)

    def save(self, path):
        import shutil
        shutil.copyfile(self.gains_path, os.path.join(path, 'gains.toml'))

    def __init__(self,
                 SAVE,
                 duration,
                 motors,
                 Hopper,
                 Parkour,
                 prediction_horizon,
                 goal_tolerance,
                 GAINS,
                 system,
                 initial_phase,
                 BACKFLIP,
                 VISUALIZATION,
                 jumping_dict,
                 path,
                 client_se,
                 client,
                 objects=None):

        self.SAVE = SAVE
        self.waiter = 0
        self.duration = duration
        self.motors = motors
        self.Hopper = Hopper
        self.U = Hopper.boomstick.circumference
        self.Parkour = Parkour
        self.prediction_horizon = prediction_horizon
        self.goal_tolerance = goal_tolerance
        self.gains_path = GAINS
        self.gains = toml.load(GAINS)
        self.system = system
        self.initial_phase = initial_phase
        self.BACKFLIP = BACKFLIP
        self.VISUALIZATION = VISUALIZATION
        self.jumping_dict = jumping_dict
        self.path = path
        self.abort = False
        self.standstill = False
        self.saved = False
        self.client_se = client_se
        self.client = client
        self.objects = objects
        self.Storage = Storage(path)
        self.Endeffector = Endeffector(self.duration * motors[0].control_freq, path)
        if VISUALIZATION:
            self.plotter = Plotter(Hopper)
            self.plotter.obstacle_course(Parkour)
            
        if system == 'mjbots':
            from software.stateEstimation.encoder import Encoder
            self.encoder = Encoder(path)
            from software.stateEstimation.contact import contact_effort_threshold
            _contact_estimation = contact_effort_threshold
            self.VICON = vicon_parkour(path, Parkour)
        elif system == 'pybullet':
            from software.stateEstimation.encoderSimulation import Encoder
            self.encoder = Encoder(self.client)
            from software.stateEstimation.contactSimulation import contact_pybullet
            _contact_estimation = lambda *args, **kwargs: contact_pybullet(self.Parkour)

        self.transition = {
            'STAGING': lambda state, r, theta: "EXERTION" if
            np.linalg.norm(Hopper.forward_velocity(*state[0, :], *state[1, :])) < 0.001
            and self.jumping_dict["staging_r"] >= r
            >= self.jumping_dict["staging_r"] - 0.02
            and self.staging_theta - np.radians(2) <= theta
            <= self.staging_theta + np.radians(2)
            and not self.standstill
            else "STAGING",

            'EXERTION': lambda state, r, theta: "FLIGHT" if
            r >= self.jumping_dict["exertion_limit"]
            else "EXERTION",

            'FLIGHT': lambda state, r, theta: "ABSORPTION" if
            _contact_estimation(state[2, :], jumping_dict["contact_threshold"])
            and r <= self.jumping_dict["flight_r"]
            else "FLIGHT",

            'ABSORPTION': lambda state, r, theta: "REPOSITION" if
            r <= self.jumping_dict["touchdown_r"] and
            np.linalg.norm(Hopper.forward_velocity(*state[0, :], *state[1, :])) < 0.1
            else "ABSORPTION",

            'REPOSITION': lambda state, r, theta: "STAGING" if
            np.linalg.norm(Hopper.forward_velocity(*state[0, :], *state[1, :])) < 0.1
            and self.staging_theta - np.radians(5) <= theta
            <= self.staging_theta + np.radians(5)
            else "REPOSITION"
            }


    def get_control_output(self, n):
        count = self.flight_counter

        # initial phase
        if n == 0:
            self.RECORD.phase[0] = self.initial_phase
            self.staging_theta = np.radians(75)

        # vicon
        if self.system == 'mjbots':
            self.VICON.track()

        # state
        state = self.RECORD.state(n)
        r, theta = self.Hopper.inverse_kinematics(*state[0, :])
        self.theta = theta

        # phase
        if n == 0:
            phase = self.initial_phase
        else:
            phase = self.transition[self.RECORD.phase[max(n - 1, 0)]](state, r, theta)
        self.RECORD.phase[n] = phase
        trans = phase != self.RECORD.phase[n - 1]

        stick_state = self.encoder.read(n)
        self.x_start = self.Endeffector.calc_xpos(n, self.client_se, stick_state[0, 2],
                                                  stick_state[0, 1], state[0, 0], state[0, 1])

        # initialization - staging phase
        if trans and phase == 'STAGING':
            print(f'{count}--------------------STAGING---------------------{n}')
            self.q_star = list(self.Hopper.kinematics(self.jumping_dict["staging_r"],
                                                      self.staging_theta))
            self.qd_star = [0, 0]

        # initialization - exertion phase
        if trans and phase == 'EXERTION':
            print(f'{count}--------------------EXERTION--------------------{n}')
            self.theta_go = theta
            exertion_d = self.jumping_dict["exertion_limit"] - r
            v_rea = np.sqrt(self.launch_velocity**2 * (np.sin(2 * self.staging_theta) / np.sin(2 * theta)))
            F = self.Hopper.leg.mass * v_rea**2 / (2 * exertion_d)
            self.F_ff = [np.sin(self.theta_go) * F, -np.cos(self.theta_go) * F]

        # initialization - flight phase
        if trans and phase == 'FLIGHT':
            print(f'{count}---------------------FLIGHT---------------------{n}')
            self.flight_counter += 1
            self.q_star = list(self.Hopper.kinematics(self.jumping_dict["flight_r"],
                                                      self.jumping_dict["flight_theta"]))
            self.qd_star = [0, 0]
            if self.abort:
                self.staging_theta = np.radians(75) + (2 * np.pi if self.BACKFLIP else 0)

        # initialization - absorption phase
        if trans and phase == 'ABSORPTION':
            print(f'{count}-------------------ABSORPTION-------------------{n}')
            self.q_star_touchdown = list(self.Hopper.kinematics(self.jumping_dict["touchdown_r"],
                                                                self.jumping_dict["touchdown_theta"]))
            self.qd_star = [0, 0]

        # initialization - reposition phase
        if trans and phase == 'REPOSITION':
            print(f'{count}-------------------REPOSITION-------------------{n}')
            if self.system == 'mjbots':
                self.Parkour = self.VICON.parkour_mod(self.x_start)
            self.z_start = pr.starting_height(self.Parkour, self.x_start)
            if not self.abort:
                print('MPPC Active')
                t_start = time.time()
                x_goal = min(self.x_start + self.prediction_horizon, self.Parkour["goal_x"])
                dist = min(self.prediction_horizon, (x_goal - self.x_start))
                N = pr.number_of_jumps(self.Hopper, dist)
                PARKOUR = copy.deepcopy(self.Parkour)
                for i in N:
                    res_dict, obstacles = mppc(self.Hopper, PARKOUR, self.x_start, 
                                               x_goal, self.goal_tolerance, N=i)
                    if res_dict is None:
                        if i == N[-1]:
                            print(f"-- NO SOLUTION FOUND --")
                            sys.exit(1)
                    else:
                        t_comp = time.time() - t_start
                        res_dict["mppc_time"] = t_comp
                        self.Storage.store(self.Parkour, res_dict)
                        print(f'MPPC Computation Time: {round(t_comp, 5)} s')
                        self.contact_x, contact_z = pr.prep_contact(self.Hopper, res_dict)
                        print(f"Planned Contact x: {np.round(self.contact_x, 2)}")
                        print(f"Planned Contact z: {np.round(contact_z, 2)}")
                        self.old_goal_x = self.contact_x[1]
                        self.staging_theta = res_dict["theta"][0]
                        self.launch_velocity = res_dict["v"][0]
                        if len(res_dict["theta"]) == 1:
                            self.abort = True
                            if self.BACKFLIP:
                                self.jumping_dict["flight_theta"] += 2 * np.pi
                                self.jumping_dict["touchdown_theta"] += 2 * np.pi
                        break
            else:
                self.standstill = True
            self.q_star = list(self.Hopper.kinematics(self.jumping_dict["staging_r"],
                                                      self.staging_theta))
            self.qd_star = [0, 0]
            if self.system == 'pybullet' and self.VISUALIZATION and not self.standstill:
                objectPlacer.area_color(self.U, self.Parkour, res_dict["areas"], self.objects["area"], second_round=False)
                objectPlacer.obstacle_color(self.U, self.Parkour, res_dict["obstacles"], self.objects["obstacle"], second_round=False)
                self.objects["contact"] = objectPlacer.contact_placement(self.client, self.U, self.contact_x, contact_z, self.objects["contact"])
                self.plotter.current_plan(PARKOUR, res_dict, self.contact_x, contact_z, count)

        # control input
        if phase == 'EXERTION':
            J = self.Hopper.jacobian(*state[0, :])
            tau = np.matmul(J.T, self.F_ff)
        else:
            Kp = np.diag(self.gains[phase]['Kp'])
            Kd = np.diag(self.gains[phase]['Kd'])
            tau = Kp.dot(self.q_star- state[0, :]) + Kd.dot(self.qd_star - state[1, :])

        if abs(tau[0]) >= self.motors[0].torque_limit or abs(tau[1]) >= self.motors[1].torque_limit:
            # print(f'LIMIT EXCEEDED -> tau = {tau}----{n}')
            tau = np.sign(tau) * min([abs(t) for t in tau], [self.motors[0].torque_limit, self.motors[1].torque_limit])

        u = np.array([[0, 0],
                      [0, 0],
                      tau])
 
        if self.standstill and phase == 'STAGING':
            if self.waiter == 0:
                dx = self.Parkour["goal_x"] - self.x_start
                print(f"Final Distance from Goal (xg-xc): {np.round(dx * 100, 1)} cm")
            self.waiter += 1
            if self.waiter >= self.motors[0].control_freq * 3:
                if self.SAVE and not self.saved:
                    if self.system == 'mjbots':
                        self.encoder.save(self.RECORD, n)
                        self.VICON.save()
                    self.Storage.save()
                    self.Endeffector.save(self.RECORD, n)
                    self.saved = True
                    print(f"Data Saved: \"{self.path}\"")
                with RedirectStream(stream=sys.stdout):
                    pb.disconnect()
                print("-- Successful Parkour Traversal --")
                sys.exit(1)
        return u
