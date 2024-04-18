"""
``pybullet``
============
"""

import os
import time

import pybullet as pb
import pybullet_data as pbd


class motor:

    torque_max = 20

    def __init__(self, joint, kp, kd, **kwargs):

        """
        ``pybullet`` motor class. Simple enough.

        **Arguments**

        ``joint`` [int]
          index of the joint at which the motor is placed in the URDF

        ``kp`` [float]
          Position error PD constant

        ``kd`` [float]
          Velocity error PD constant

        ``kwargs`` [dict]
          key-value pairs which will become attributes of the ``motor`` instance
        """
        
        self.joint  = joint
        self.kp     = kp
        self.kd     = kd

        # torque limit
        self.torque_limit = min(kwargs.pop('torque_limit', motor.torque_max), motor.torque_max)

        # other parameters
        for k, v in kwargs.items():
            setattr(self, k, v)


def simulation(g,
               mu,
               motors,
               default_freq=250):
    """
    Generic ``pybullet`` simulation setup.

    This setup may not necessarily work for you, but it is fairly
    general and a good reference to have if you want to set up
    your own simulation in ``pybullet``.

    **Arguments**

    ``g`` [float]
      actual (read: **not absolute**, that is, negative) magnitude of
      the downwards-pointing gravity vector

    ``mu`` [float]
      ``pybullet`` friction coefficient with the ground plane

    ``motors`` [list of instances of motor classes inheriting from ``AbstractMotor``]
      list of instances of ``AbstractMotor`` child classes

    ``default_freq`` [float]
      default simulation frequency

    **Output**

    ``client`` [int]
      numerical identifier of the pybullet physics client
      running the current simulation

    """

    # physics client
    import sys
    from software.util.suppress import RedirectStream
    with RedirectStream(stream=sys.stdout):
      client = pb.connect(pb.GUI)

    # gui
    pb.configureDebugVisualizer(pb.COV_ENABLE_GUI,
                                0,
                                physicsClientId=client)
    # camera
    pb.resetDebugVisualizerCamera(cameraDistance=1.0,
                                  cameraYaw=0,
                                  cameraPitch=0,
                                  cameraTargetPosition=[0, 0, 0.25],
                                  physicsClientId=client)
    
    ##################
    #  ground plane  #
    ##################
    
    pb.setAdditionalSearchPath(pbd.getDataPath())
    plane = pb.loadURDF("plane.urdf",
                        [0, 0, 0],
                        useFixedBase=1,
                        physicsClientId=client)

    ##################
    #     physics    #
    ##################

    # gravity
    pb.setGravity(g[0],
                  g[1],
                  g[2],
                  physicsClientId=client)

    # friction
    pb.changeDynamics(plane,
                      -1,
                      lateralFriction=mu,
                      physicsClientId=client)

    ##################
    #     runtime    #
    ##################

    f  = motors[0].control_freq if hasattr(motors[0], 'control_freq') else default_freq
    dt = 1/f
    
    # initial time
    pb.setRealTimeSimulation(0,
                             physicsClientId=client)

    # time step
    pb.setTimeStep(dt,
                   physicsClientId=client)

    return client
