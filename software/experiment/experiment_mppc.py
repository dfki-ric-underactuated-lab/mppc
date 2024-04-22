# GENERAL
import sys
import os
from software.util.suppress import RedirectStream
with RedirectStream(stream=sys.stderr):
    import pybullet as pb
from importlib import import_module
from os.path import join, abspath
import numpy as np

# CONTROL LOOP
from software.spine.utils.log import log_experiment

# HOPPING LEG
from software.stateMachine.behaviorStabilization import StateMachine
from software.parkour.parkourStorage import choose_parkour
from software.plant.parameter import initialize_hopper

########################################################################################################################
# SETTINGS #############################################################################################################
########################################################################################################################
assert isinstance(sys.argv[1], str), 'You must provide the index of the system you want to use'
system = {
    '0': 'mjbots',
    '1': 'pybullet'
}[sys.argv[1]]

EXPERIMENT      = 'simulation_01'
obstacle_course = 1
BACKFLIP        = True
SAVE            = False
duration        = 300  # [s]

if system == 'pybullet':
    key = input(f"Would you like to visualize the MPPC behavior generation? "
                f"Please note, that this comes at the cost of a faltering simulation and that you need to close the "
                f"appearing figures to continue the simulation. (yes/no)")
    while key not in ["yes", "y", "no", "n"]:
        key = input("Please answer with 'yes' or 'no': ").strip().lower()
    VISUALIZATION = True if key in ["yes", "y"] else False
    from software.spine.sim import load
    path           = f'data/simulation/{EXPERIMENT}'
    GAINS          = 'model/gains/gains.toml'
    INITIAL_PHASE  = 'REPOSITION'
    CONTACT_EST    = 'PB'
    urdf, motors = load(f'model/config/robot_sim.toml')
    control_freq = motors[0].control_freq
else:
    path           = f'data/experiment/{EXPERIMENT}'
    GAINS          = 'model/gains/gains.toml'
    INITIAL_PHASE  = 'REPOSITION'
    CONTACT_EST    = 'ET'
    from software.spine.motors import load
    motors = load(f'model/config/robot_exp.toml')
    control_freq = motors[0].control_freq

# simulation
mu = 3
free_joints = [2, 3]
timestep_delay = False

# log
log_experiment(EXPERIMENT, f'{system}')
if SAVE and not os.path.exists(path):
    os.makedirs(path)

########################################################################################################################
# INITIALIZATION #######################################################################################################
########################################################################################################################
# plant
json_path = 'model/plant/boomstick_hopper.json'
Hopper = initialize_hopper(json_path)

# parkour
Parkour = choose_parkour(obstacle_course)
prediction_horizon = 2  # [m]
goal_tolerance = 0.3  # [m]

# pybullet system setup
if system == 'pybullet':
    from software.spine.sim.pybullet import simulation
    client = simulation([0, 0, -Hopper.gravity],
                        mu,
                        motors)
    
    from software.util.build import legV2_boomstick as model
    objects = model(client,
                    urdf,
                    Hopper.boomstick.circumference,
                    Parkour,
                    mu,
                    free_joints=free_joints)
else:
    client = None

# pybullet state estimation
with RedirectStream(stream=sys.stdout):
    client_se = pb.connect(pb.DIRECT)
urdf_se = 'model/urdf/monoped.urdf'
startPos_se = [0, 0, 0]
startAtt_se = pb.getQuaternionFromEuler([np.radians(0), -np.radians(90), np.radians(0)])
robot_se = pb.loadURDF(urdf_se, startPos_se, startAtt_se, physicsClientId=client_se)

########################################################################################################################
# Controller ###########################################################################################################
########################################################################################################################
# jumping dict
touchdown_theta = np.radians(95)
contact_threshold = 1
jumping_dict = {"touchdown_r": Hopper.leg.r_touchdown,
                "touchdown_theta": touchdown_theta,
                "staging_r": Hopper.leg.r_staging,
                "exertion_limit": Hopper.leg.r_takeoff,
                "flight_r": Hopper.leg.r_flight,
                "flight_theta": np.radians(Hopper.leg.theta_flight),
                "contact_threshold": contact_threshold}

# controller
controller = StateMachine(SAVE,
                          duration,
                          motors,
                          Hopper,
                          Parkour,
                          prediction_horizon,
                          goal_tolerance,
                          GAINS,
                          system,
                          INITIAL_PHASE,
                          BACKFLIP,
                          VISUALIZATION,
                          jumping_dict,
                          path,
                          client_se,
                          client,
                          objects)
if system == 'mjbots':
    controller.initial_state = Hopper.kinematics(0.13, np.radians(80))

########################################################################################################################
# EXECUTION ############################################################################################################
########################################################################################################################
if system == 'mjbots':
    import asyncio
    from software.spine.mjbots import motor_control_loop
    RECORD = asyncio.run(motor_control_loop(controller,
                                            motors,
                                            duration,
                                            independent_variables=[('phase', object)],
                                            plot=True,
                                            save=SAVE,
                                            directory=path,
                                            series='final',
                                            name=EXPERIMENT,
                                            timestamp=True))

if system == 'pybullet':
    from software.spine.pybullet import motor_control_loop
    RECORD = motor_control_loop(controller,
                                client,
                                pb.TORQUE_CONTROL,
                                objects["robot"],
                                motors,
                                timestep_delay,
                                duration,
                                independent_variables=[('phase', object)],
                                countdown=0,
                                plot=VISUALIZATION,
                                save=SAVE,
                                video=SAVE,
                                directory=path,
                                series='',
                                name='',
                                timestamp=False)

print("runout without error")