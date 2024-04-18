"""
``mjbots``
==========
"""

import os
import asyncio
import numpy as np

import moteus

from software.spine.motors.abstract_motor import AbstractMotor


def mjbots_zero(motor_id, pi3hat, bus_id=None):
    """
    Zero an mjbots motor using the command line ``moteus_tool`` utility.

    **Arguments**

    ``motor_id`` [int]
      ``mjbots`` CAN motor ID

    ``pi3hat`` [bool]
      whether a Raspberry Pi with a Pi3Hat is being used to communicate with the motor

    ``bus_id`` [int]
      index of the CAN bus in use
    """
    
    if pi3hat:
        assert bus_id is not None, 'you must provide a *bus_id* to zero an mjbots motor when communicating with it using a RaspberryPi and Pi3Hat'

    pi3hat_cfg = f"--pi3hat-cfg '{bus_id}={motor_id}' " if pi3hat else ""
    
    os.system(f"moteus_tool --zero-offset {pi3hat_cfg}-t {motor_id}")


def mjbots_pvt(state):
    "Retrieve ~SI~ POSITION, VELOCITY and TORQUE from an mjbots state object."
    return np.array([state.values[moteus.Register.POSITION]*2*np.pi,
                     state.values[moteus.Register.VELOCITY]*2*np.pi,
                     state.values[moteus.Register.TORQUE]])


class mjbots(AbstractMotor):

    # Runtime
    runtime = 'async'

    def __init__(self,
                 motor_id,
                 transport,
                 kp, kd,
                 **kwargs):
        """
        ``mjbots`` abstract motor class.

        All ``mjbots`` motor classes inherit from this one. This is because
        model-specific characteristics of the motors, such as maximum torque,
        are not available through the ``moteus`` API, and must thus be stored
        as class attributes.

        **Arguments**

        ``motor_id`` [int]
          ``mjbots`` CAN motor ID

        ``transport`` [``moteus_pi3hat.Pi3HatRouter`` instance]
          instance of ``moteus_pi3hat.Pi3HatRouter``, which allows a Raspberry Pi with a
          Pi3Hat to communicate with ``mjbots`` motors. If a PC is being used, ``transport``
          is ``None``

        ``kp`` [float]
          Position error PD constant

        ``kd`` [float]
          Velocity error PD constant

        ``kwargs`` [dict]
          key-value pairs which will become attributes of the ``mjbots`` child class instance
        """

        # Model and manufacturer for ease of inspection
        self.model        = self.__class__.__name__
        self.manufacturer = self.__class__.__module__.split('.')[-1]
        
        # Validate inputs
        for var in [kp, kp]:
            if isinstance(var, str):
                var = float(var)
        for var in [motor_id]:
            if isinstance(var, str):
                var = int(var)

        self.motor_id = motor_id

        # Driver
        self.motor = moteus.Controller(id=motor_id, transport=transport)
        
        # Configuration
        self.kp = kp
        self.kd = kd
        self.kp_scale = kp/self.kp_default
        self.kd_scale = kd/self.kd_default

        # Other parameters
        for k, v in kwargs.items():
            setattr(self, k, v)


class qdd100(mjbots):

    # Motor characteristics
    torque_max = 18
    kp_default = 100
    kd_default = 2
    
    def __init__(self, *args, **kwargs):

        """
        ``mjbots`` qdd100 motor class.

        **Arguments**

        ``args`` [list]
          positional arguments for ``software.spine.motors.mjbots.mjbots``

        ``kargs`` [dict]
          keyword arguments for ``software.spine.motors.mjbots.mjbots``
        """
        
        # mjbots motor initialization
        super(qdd100, self).__init__(*args, **kwargs)
        
        # qdd100 torque limit
        self.torque_limit = min(kwargs.pop('torque_limit', qdd100.torque_max), qdd100.torque_max)

        
