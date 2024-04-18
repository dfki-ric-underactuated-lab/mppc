"""
Load Time
=========
"""

import re


def general(robot):
    """
    Retrieve the general arguments keyword from a motor configuration
    dictionary ``robot`` using a case-insensitive regex search.

    **Arguments**

    ``robot`` [dict]
      dictionary obtained from a TOML file containing the configuration
      of all motors in the system
    """

    pattern = re.compile(r'general', flags=re.IGNORECASE)

    general = ''
    for key in robot.keys():
        if re.match(pattern, str(key)):
            general = str(key)
            
    return general
