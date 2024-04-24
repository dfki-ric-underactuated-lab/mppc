import time
import copy
import sys
from software.util.suppress import RedirectStream
with RedirectStream(stream=sys.stderr):
    import pybullet as pb
    import pybullet_data
import numpy as np

from software.util.prepare import prep_contact
from software.plant.parameter import initialize_hopper
from software.parkour import objectPlacer
from software.util import read

# general
try:
    experiment = {
    '1': 'static',
    '2': 'dynamic_1',
    '3': 'dynamic_2',
    '4': 'dynamic_3',
    '5': 'dynamic_4',
    '6': 'two_rounds',
    '7': 'disturbances',
    }[sys.argv[1]]
except:
    print("Please indicate which experiment you want to see.\n"
          "\'1\' -> static\n"
          "\'2\' -> dynamic_1\n"
          "\'3\' -> dynamic_2\n"
          "\'4\' -> dynamic_3\n"
          "\'5\' -> dynamic_4\n"
          "\'6\' -> two_rounds\n"
          "\'7\' -> disturbances\n")
    sys.exit(1)

save = False
urdf = 'model/urdf/monoped.urdf'

# plant
json_path = 'model/plant/boomstick_hopper.json'
Hopper = initialize_hopper(json_path)
U = Hopper.boomstick.circumference

# import data
t, phase, hip, knee, hip_tau, knee_tau, hip_tau_des, knee_tau_des = read.read_stateData(experiment)
pitch, yaw = read.read_encoderData(experiment)
parkours_list = read.read_parkourData(experiment)
results_list = read.read_resultData(experiment)
vicon_list = read.read_viconData(experiment)
frequ = 50
steps = np.arange(0, t[-1] + 1, 1/frequ)
j = 0
timer = copy.deepcopy(t)

# stabilize flickering obstacles
if experiment == 'two_rounds':
    for i in range(len(vicon_list)):
        if i >= 1660:
            vicon_list[i]["obs_1_pos"] = vicon_list[i-1]["obs_1_pos"]
            vicon_list[i]["obs_1_rot"] = vicon_list[i-1]["obs_1_rot"]
        if vicon_list[i]["obs_1_pos"][0] >= 6000 or vicon_list[i]["obs_1_rot"][0] <= -1:
            vicon_list[i]["obs_1_pos"] = vicon_list[i-1]["obs_1_pos"]
            vicon_list[i]["obs_1_rot"] = vicon_list[i-1]["obs_1_rot"]

if experiment == 'sliding_2':
    for i in range(1000):
        vicon_list[i]["obs_1_pos"] = vicon_list[i+100]["obs_1_pos"]
        vicon_list[i]["obs_1_rot"] = vicon_list[i+100]["obs_1_rot"]

# time adjustment
for i in range(len(timer)-1):
    if timer[i] >= steps[j]:
        j += 1
    else:
        if phase[j] == 'REPOSITION' and phase[max(j-1, 0)] == 'ABSORPTION' and phase[j+1] == 'STAGING':
            j += 1
        else:
            del t[j]
            del phase[j]
            del hip[j]
            del knee[j]
            del pitch[j]
            del yaw[j]
            del vicon_list[j]


# pybullet
with RedirectStream(stream=sys.stdout):
    if save:
        fps = int(np.round(len(t)/t[-1]))
        print(f"Frames per second: {fps}")
        client = pb.connect(pb.GUI, options=f"--mp4=\"data/experiment/{experiment}/replay.mp4\" --mp4fps={fps}")
    else:
        client = pb.connect(pb.GUI)
pb.configureDebugVisualizer(pb.COV_ENABLE_GUI,0)
pb.resetDebugVisualizerCamera(cameraDistance=1.2,
                              cameraYaw=0,
                              cameraPitch=-21.4,
                              cameraTargetPosition=[0, -0.4, 0.13],
                              physicsClientId=client)
pb.setAdditionalSearchPath(pybullet_data.getDataPath()) 
plane = pb.loadURDF("plane.urdf")
startPos = [0, 0, 0]
startAtt = pb.getQuaternionFromEuler([np.radians(0), -np.radians(90), np.radians(0)])
robot = pb.loadURDF(urdf, startPos, startAtt, physicsClientId=client)
objectPlacer.marker_placement(client, U)
second_round = False
area_objects = objectPlacer.area_placement(client, U, parkours_list[0], second_round)
obstacle_objects = objectPlacer.obstacle_placement(client, U, parkours_list[0], second_round)
vicon_objects = []
contact_objects = []
vicon_color = [[], []]

# replay
jumps = 0
t_start = time.time()
for i in range(len(t)-1):
    pb.configureDebugVisualizer(pb.COV_ENABLE_SINGLE_STEP_RENDERING,1)
    pb.resetJointState(bodyUniqueId=robot, jointIndex=2, targetValue=yaw[i], physicsClientId=client)
    pb.resetJointState(robot, 3, pitch[i], physicsClientId=client)
    pb.resetJointState(robot, 10, hip[i], physicsClientId=client)
    pb.resetJointState(robot, 11, knee[i], physicsClientId=client)
    vicon_objects = objectPlacer.vicon_placement(client, vicon_list[i], vicon_color, vicon_objects)
    if phase[i] == 'REPOSITION' and phase[i-1] != 'REPOSITION':
        if jumps < len(parkours_list):
            contact_x, contact_z = prep_contact(Hopper, results_list[jumps])
            if experiment == 'two_rounds':
                if contact_x[1] >= U and second_round == False:
                    second_round = True
                    for j in area_objects:
                        pb.removeBody(j, physicsClientId=client)
                    for j in obstacle_objects:
                        pb.removeBody(j, physicsClientId=client)
                    area_objects = objectPlacer.area_placement(client, U, parkours_list[jumps], second_round)
                    obstacle_objects = objectPlacer.obstacle_placement(client, U, parkours_list[jumps], second_round)
            objectPlacer.area_color(U, parkours_list[jumps], results_list[jumps]["areas"], area_objects, second_round)
            vicon_color = objectPlacer.obstacle_color(U, parkours_list[jumps], results_list[jumps]["obstacles"], obstacle_objects, second_round)
            contact_objects = objectPlacer.contact_placement(client, U, contact_x, contact_z, contact_objects)
            jumps += 1
        else:
            objectPlacer.contact_placement(client, U, [], [], contact_objects)
    while t[i] > (time.time() - t_start):
        pass
    pb.stepSimulation()
    
with RedirectStream(stream=sys.stdout):
    pb.disconnect()
if save:
    print(f"Replay Saved: \"data/experiment/{experiment}/replay.mp4\"")

print("-- Successful Runout --\n")