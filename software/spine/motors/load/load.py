"""
Config Load Entrypoint
======================
"""

import re
import toml
from copy import deepcopy as dc
from importlib import import_module

from software.spine.motors.load.load_mjbots import load_mjbots

from software.spine.utils.load import general


def load(robot, zero=True, **kwargs):
    """
    Create motor instances from a ``robot`` configuration dictionary

    **Arguments**

    ``robot`` [str]
      Path to a TOML file containing the configuration of all motors in
      the system.

      It **must** contain:

      * motor sections, following the syntax below

      It **may** contain:

      * a ``general`` section. All parameters in this section will become
        attributes of all motors.

      Example follows::

         [general]
         bus_id       = 4
         control_freq = 200

         [motor_1]
         manufacturer = 'mjbots'
         model        = 'qdd100'
         bus_id       = 4
         motor_id     = 3
         # --------------------- mechanical
         kp           = 100
         kd           = 2
         torque_limit = 10
         gear_ratio   = 1

         [motor_2]
         ...

    ``zero`` [bool]
      whether to zero the motors after instantiating the motor objects

    ``kwargs`` [dict]
      keyword arguments passed directly to the manufacturer-specific motor loading functions

    **Output**

    ``motors`` [list of instances of motor classes inheriting from ``AbstractMotor``]
      list containing an instance of a motor class (inheriting from
      ``AbstractMotor``) for each motor in the ``_robot``.
    """

    # load configuration
    _robot = toml.load(robot)

    # general keyword
    _general = general(_robot)

    # names
    names = list(_robot.keys())
    names.remove(_general)
    
    # import motor classes
    getmod = lambda module: import_module(f"software.spine.motors.{module}")
    for motor in _robot.keys():
        if motor != _general:

            manufacturer = _robot[motor]['manufacturer']

            try:
                # try to retrieve <manufacturer>.<model> class (eg: mjbots.qdd100)
                _robot[motor]['class'] = getattr(getmod(manufacturer), _robot[motor]['model'])
            except AttributeError:
                # else, retrieve <manufacturer>.<manufacturer> class (eg: tmotor.tmotor)
                _robot[motor]['class'] = getattr(getmod(manufacturer), manufacturer)
                
    # ==================
    # instantiate motors
    # ==================
    # mjbots
    mjbots = load_mjbots(_robot, zero, **kwargs)

    # tmotor
    tmotor = load_tmotor(_robot, zero, **kwargs)
    
    # gather and assign motor configuration as attributes,
    # so as to save it later with the rest of the experiment's
    # if the user wishes data
    motors = mjbots + tmotor

    for motor in motors:
        motor.config = robot
        motor.name   = names[motors.index(motor)]
    
    return motors
