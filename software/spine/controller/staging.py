"""
Staging
=======
"""

import numpy as np
import pandas as pd
from copy import deepcopy as dc

from scipy.interpolate import interp1d

from software.spine.controller.abstract import AbstractController_TrajectoryFollowing_ActuatorSpace_Offline


class Controller_Staging(AbstractController_TrajectoryFollowing_ActuatorSpace_Offline):

    def __init__(self, initial_state, final_state, duration):
        """
        Linear staging controller.
        
        **Arguments**

        ``initial_state`` [list of floats]
          list of length ``m``, where ``m`` is the number of
          motors -the length of ``motors``-, containing the
          initial **angular position** of each motor.

        ``duration`` [float]
          duration of the initialization manoeuvre
        """

        # initialization trajectory
        k = 200
        t = np.linspace(0, duration, k)

        trajectory = {}

        for i in range(len(initial_state)):
            
            trajectory.update({
                f'm{i}_pos': np.linspace(initial_state[i], final_state[i], k),
                f'm{i}_vel': np.zeros(k),
                f'm{i}_tau': np.zeros(k)
            })

        # create DataFrame
        self.trajectory = pd.DataFrame(data=trajectory)
        
        # interpolate
        self.interpolate_trajectory(t, self.trajectory)

    def get_gains(self, n):
        return [200, 4], [200, 4]
