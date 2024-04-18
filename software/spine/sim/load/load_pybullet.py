"""
``pybullet``
============
"""

from copy import deepcopy as dc

from software.spine.sim.pybullet import motor


def load_pybullet(robot):
    """
    create ``software.spine.sim.pybullet.motor`` instances from a ``robot``
    configuration dictionary

    **Arguments**

    ``robot`` [dict]
      dictionary containing settings for all motors in a system. In
      this function only the motors with ``simulator=='pybullet'``
      will be initialized
    
    **Output**

    ``pybullet`` [list of ``mjbots.<model>`` instances]
      list containing all initialized motor instances
    """

    if any(['pybullet' in config['simulator'] for config in robot.values()]):

        pybullet = []
        for name, config in robot.items():

            if config['simulator'] == 'pybullet':
                
                _motor  = motor(**config)
        
                pybullet.append(_motor)

        return pybullet

    return []
