"""
``mjbots``
==========
"""

import sys
import logging
import traceback

from copy import deepcopy as dc
from importlib import import_module

from software.spine.utils.load import general


def load_mjbots(robot, zero):
    """
    Create ``mjbots`` motor instances from a ``robot`` configuration dictionary

    1. Determine whether a Raspberry Pi with a Pi3Hat is being used
    2. Retrieve bus ID
    3. Retrieve motor IDs of all mjbots motors
    4. If ``zero`` is set to ``True``, zero all motors using ``mjbots_zero``
    5. Create transport
    6. Create ``mjbots.<model>`` instances

    **Arguments**

    ``robot`` [dict]
      dictionary containing settings for all motors in a system. In
      this function only the motors with ``manufacturer=='mjbots'``
      will be initialized

    ``zero`` [bool]
      whether to zero the motors after instantiating the ``mjbots`` motor objects

    **Output**

    ``mjbots`` [list of ``mjbots.<model>`` instances]
      list containing all initialized motor instances
    """

    # general arguments keyword
    _general = general(robot) if general(robot) != '' else False
    
    # motor configuration dictionary 
    motors = dc(robot)
    del motors[_general]

    if any(['mjbots' in config['class'].__module__ for config in motors.values()]):

        # create lists containing the value of `param` in all mjbots motor configurations
        get_mjbots = lambda param: [{**robot[_general], **config}[param] if _general else config[param] for config in motors.values() if 'mjbots' in config['class'].__module__]
        
        # determine whether a Raspberry Pi with a pi3hat is being used
        try:
            moteus_pi3hat = import_module('moteus_pi3hat')
            pi3hat = True
        except ModuleNotFoundError:
            pi3hat = False

        # bus ID
        if pi3hat:
            bus_ids = get_mjbots('bus_id')
            assert bus_ids, 'you must provide a *bus_id* either as a "general" or motor config argument to control one or more mjbots motors from a RaspberryPi + Pi3Hat'
            bus_id = get_mjbots('bus_id')[0]
            assert isinstance(bus_id, int), 'the provided *bus_id* is not an integer. Check your motor configuration'
        else:
            bus_id = None

        # motor IDs
        motor_ids = get_mjbots('motor_id')

        # zero
        if zero:
            from software.spine.motors.mjbots import mjbots_zero
            print('\n==================')
            for motor_id in motor_ids:
                mjbots_zero(bus_id=bus_id, motor_id=motor_id, pi3hat=pi3hat)
                print(f"MOTOR[{motor_id}] :: ZEROED" + ("\n" if motor_id != motor_ids[-1] else ""))
            print('==================\n')

        # create transport
        if pi3hat:
            servo_bus_map = {bus_id: motor_ids}
            transport = moteus_pi3hat.Pi3HatRouter(
                servo_bus_map=servo_bus_map,
            )
        else:
            transport = None

        # ==================
        # instantiate motors
        # ==================

        # provide motor classes with general `robot` parameters, their specific config, and the common transport
        arguments = lambda config: {**robot[_general], **config, **{'transport': transport}}
        
        mjbots = [robot.pop(name).pop('class')(**arguments(config)) for name, config in motors.items() if 'mjbots' in robot[name]['class'].__module__]

        return mjbots

    return []
