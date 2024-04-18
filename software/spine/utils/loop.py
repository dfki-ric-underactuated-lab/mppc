"""
Control Loop
============
"""

import time


def countdown(count=5):
    """
    Display a countdown of ``count`` seconds in the command line.

    **Arguments**

    ``count`` [int]
      duration of the countdown
    """

    if count > 0:
        down  = False
        print(f'\nEXPERIMENT STARTING IN {count}')
        while not down:
            _t = time.time()
            while time.time() - _t < 1:
                pass
            count -= 1
            down   = count == 0
            print(f'                       {count}')
        
