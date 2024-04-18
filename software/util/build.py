import time
import numpy as np
import pybullet as pb
import pybullet_data as pbd
from software.parkour import objectPlacer


def propagate(client, t):
    t0 = time.time()
    dt = pb.getPhysicsEngineParameters(client)['fixedTimeStep']
    
    while time.time() - t0 < t:

        _t = time.time()
        
        pb.stepSimulation(physicsClientId=client)

        while time.time() - _t < dt:
            pass


def legV2_boomstick(client,
                    urdf,
                    U,
                    Parkour,
                    mu,
                    free_joints=[]):

    # camera position
    pb.resetDebugVisualizerCamera(cameraDistance=1.8,
                                  cameraYaw=0,
                                  cameraPitch=-25,
                                  cameraTargetPosition=[0, 0, 0.2],
                                  physicsClientId=client)
    
    # robot
    robot = pb.loadURDF(urdf,
                        [0, 0, 0],
                        baseOrientation=pb.getQuaternionFromEuler([0, -np.pi/2, 0]),
                        physicsClientId=client)
    
    pb.changeDynamics(robot,
                      -1,
                      lateralFriction=mu,
                      restitution =0,
                      contactStiffness =80000,
                      contactDamping =10,
                      physicsClientId=client)
    
    for joint in free_joints:        
        pb.setJointMotorControl2(robot,
                                 joint,
                                 pb.VELOCITY_CONTROL,
                                 force=0,
                                 physicsClientId=client)

    # obstacles
    obstacle_objects = objectPlacer.obstacle_placement(client, U, Parkour, second_round=False)
    for obstacle in obstacle_objects:
        pb.changeDynamics(obstacle,
                          0,
                          lateralFriction=mu,
                          physicsClientId=client)

    # areas
    area_objects = objectPlacer.area_placement(client, U, Parkour, second_round=False)

    # marker
    objectPlacer.marker_placement(client, U)

    propagate(client, 2)

    objects = {"robot": robot,
               "obstacle": obstacle_objects,
               "area": area_objects,
               "contact": []}

    return objects
