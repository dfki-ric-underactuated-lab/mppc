"""
Config Load Entrypoint
======================
"""

import toml

from software.spine.sim.load.load_pybullet import load_pybullet

from software.spine.utils.load import general


def load(robot):
    """
    Create simulation motor instances from a ``robot`` configuration dictionary.

    **Arguments**

    ``robot`` [str]
      Path to a TOML file containing the configuration of all motors in
      the system.

      It **must** contain:

      * a ``model`` section with a ``urdf`` attribute containing the path to
        your robot's urdf file
      * motor sections, following the syntax below

      It **may** contain:

      * a ``general`` section. All parameters in this section will become
        attributes of all motors.

      Example follows::

         [model]
         urdf         = "**/.*.urdf"
    
         [general]
         simulator    = 'pybullet'
         control_freq = 250

         [motor_1]
         joint        = 2
         # --------------------- mechanical
         kp           = 200
         kd           = 8
    
         [motor_2]
            ...

     **Output**

    ``motors`` [list of instances of motor classes inheriting from ``AbstractMotor``]
      list containing an instance of a motor class (inheriting from
      ``AbstractMotor``) for each motor in the ``_robot``.
    """

    # load configuration
    _robot = toml.load(robot)

    # model
    urdf = _robot.pop('model')['urdf']

    # assimilate general parameters
    _general = _robot.pop(general(_robot))

    for config in _robot.values():
        config.update(_general)

    # names
    names = list(_robot.keys())

    # ==================
    # instantiate motors
    # ==================
    # pybullet
    pybullet = load_pybullet(_robot)

    # gather and assign motor configuration as attributes,
    # so as to save it later with the rest of the experiment's
    # if the user wishes data
    motors   = pybullet

    for motor in motors:
        motor.config = robot
        motor.name   = names[motors.index(motor)]

    return urdf, motors
    

    
