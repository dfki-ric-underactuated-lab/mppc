"""
Abstract Controller
===================
"""

from abc import ABC, abstractmethod

import numpy as np
from scipy.interpolate import interp1d


class AbstractController(ABC):
    """
    Abstract controller class.

    If you want to write a controller and give it to one of the
    ``motor_control_loop`` functions, it must inherit from this class.

    **Attributes**

    ``initial_state`` [list of floats]
      list of length ``m``, where ``m`` is the number of
      motors -the length of ``motors``-, containing the
      initial **angular position** of each motor.
    """

    def _load_record(self, RECORD):
        """
        Make ``RECORD`` an attribute of the current ``AbstractController`` instance.

        **Arguments**
        
        ``RECORD`` [``spine.record.record`` instance]
          experiment ``record`` object, containing the time vector of the
          experiment as well as the desired and measured state vectors
          of all motors in the experiment.

        .. warning::

           This function must not be overwritten.
         
        """
        self.RECORD = RECORD
    
    @abstractmethod
    def get_control_output(self, n):
        """
        Control output for ``m`` motors. It must be overwritten
        by controllers children of this class.

        This function may access the measured state matrix of
        the robot as follows::

           state = self.RECORD.state(n)

        **Arguments**

        ``n`` [int]
          motor control loop iteration number

        **Output:**

        `3` (position, velocity, torque) by `m` (number
        of motors) matrix the columns of which are the desired
        state vectors of each motor at timestep ``max(0, n+1)``
        """
        pass


class AbstractController_TrajectoryFollowing_ActuatorSpace_Offline(AbstractController):
    """
    Abstract offline trajectory following controller.

    Trajectory following controllers inheriting from this one
    must have an ``interpolated_trajectory`` attribute.

    ``interpolated_trajectory`` must be a list containing the
    interpolated position, velocity and torque for each motor
    in the system, as such::

       [[m1_int_pos, m1_int_vel, m1_int_tau],
        [    ...   ,     ...   ,    ...    ],
        [mn_int_pos, mn_int_vel, mn_int_tau]]

    ``scipy.interpolate.interp1d`` is recommended to obtain the
    interpolations (1D spline interpolation). By default the
    degree of the spline interpolation is 1. You can make the spline
    cubic by specifying ``kind='cubic'`` in the ``interp1d`` call.
    """

    initialized = False

    def interpolate_trajectory(self, t, trajectory):
        """
        Interpolate a control trajectory to achieve perfect trajectory following
        control input with non-constant control frequency.

        **Arguments**

        ``t`` [numpy.ndarray]
          trajectory time vector

        ``trajectory`` [pandas.DataFrame]
          Pandas DataFrame with one column per tracked state variable. Its columns
          must contain the values that the respective state variable takes through
          the trajectory.          
        """

        self.initialized = True

        self.interpolated_trajectory = [interp1d(t, trajectory[state_var], kind='cubic') for state_var in trajectory.columns]

    def get_control_output(self, n):
        """
        Trajectory following control output.

        **Arguments**

        ``n`` [int]
          iteration number

        Process:

        1. Obtain elapsed time
        3. Calculate the input matrix

        Return:

        ``u`` [``numpy.ndarray``]
          ``3`` (position, velocity, torque) by ``m`` (number of motors)
          matrix the columns of which are the desired state vector of each motor
          at the time specified by the ``n``th element of the trajectory's time
          vector.
        """

        assert self.initialized, 'to use a trajectory following controller inheriting from *AbstractTrajectoryFollowingController* you must interpolate your discrete trajectory by calling *self.interpolate_trajectory*, providing it the time vector of your trajectory and the trajectory Pandas DataFrame. Check the *AbstractTrajectoryFollowingController* documentation for more information'

        # elapsed time
        t = self.RECORD.t[n]

        # number of motors
        m = int(len(self.interpolated_trajectory) / 3)

        u = np.empty((3, m))
        
        for i in range(m):
            u[:, i] = np.array([
                self.interpolated_trajectory[3*i](t),
                self.interpolated_trajectory[3*i+1](t),
                self.interpolated_trajectory[3*i+2](t), 
            ])

        return np.array(u)
