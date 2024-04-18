import pybullet as pb
import numpy as np


def contact_pybullet(Parkour):
    client = 0
    robot = 1
    n = pb.getNumJoints(robot, physicsClientId=client)
    objects = np.append([0], np.arange(2, 2 + len(Parkour["position"])))
    for i in objects:
        contact_points = pb.getContactPoints(bodyA=robot,
                                             bodyB=i,
                                             linkIndexA=n-1,
                                             physicsClientId=client)
        if len(contact_points) > 0:
            return True
    return False