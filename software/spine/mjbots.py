"""
``mjbots``
==========
"""

import os
import math
import time
import shutil
import itertools
import numpy as np
from copy import deepcopy as dc

# mjbots
import asyncio
from software.spine.motors.mjbots import mjbots_pvt

# spine
from software.spine.data.record import record
from software.spine.data.plot import plot_record
from software.spine.data.record import record_directory
from software.spine.utils.loop import countdown as _countdown

from software.spine.controller.staging import Controller_Staging


async def send_command(motor,
                       position,
                       velocity=None,
                       feedforward_torque=None,
                       kp_scale=None,
                       kd_scale=None,
                       maximum_torque=None,
                       stop_position=None,
                       watchdog_timeout=None,
                       velocity_limit=None,
                       accel_limit=None,
                       query=True):
    """
    ``mjbots`` motor control command.

    **Arguments**

    ``motor`` [instance of motor class inheriting from ``AbstractMotor``]
      instance of ``AbstractMotor`` child class

    ``position`` [float]
      desired angular position in radians

    ``velocity`` [float]
      desired angular velocity in radians per second

    ``feedforward_torque`` [float]
      feedforward torque in `Nm`

    ``kp_scale`` [float]
      Kp scale used internally by the ``moteus`` PD controller

    ``kd_scale`` [float]
      Kd scale used internally by the ``moteus`` PD controller

    ``maximum_torque`` [float]
      maximum torque applicable in `Nm`

    ``stop_position`` [float]
      maximum position allowed to the motor, in radians

    ``watchdog_timeout``
      refer to the ``moteus`` documentation

    ``velocity_limit`` [float]
      maximum velocity allowed to the motor, in radians per second

    ``accel_limit`` [float]
      maximum acceleration allowed to the motor, in radians per second squared

    ``query`` [bool]
      whether to return the state of the motor after executing the command
    ````
    """
    # command
    state = await motor.set_position(position=           position,
                                     velocity=           velocity,
                                     feedforward_torque= feedforward_torque,
                                     kp_scale=           kp_scale,
                                     kd_scale=           kd_scale,
                                     maximum_torque=     maximum_torque,
                                     stop_position=      stop_position,
                                     watchdog_timeout=   watchdog_timeout,
                                     velocity_limit=     velocity_limit,
                                     accel_limit=        accel_limit,
                                     query=              query)

    # return state in SI units
    return mjbots_pvt(state)


async def test_connection(motors):
    """
    Test connection with mjbots motors by reading their state without
    sending further commands.

    **Arguments**

    ``motors``
      [list of instances of motor classes inheriting from ``AbstractMotor``]
      list of instances of ``AbstractMotor`` child classes
    """

    try:
        for motor in motors:
            pos, vel, tau = await send_command(motor, position=math.nan)

            m = motor.motor_id if hasattr(motor, 'motor_id') else motors.index(motor)
            
            print(f'M{m} ' +
                  f'p[{pos:.5f}] ' +
                  f'v[{vel:.5f}] ' +
                  f't[{tau:.5f}]')
    finally:
        for motor in motors:
            await motor.set_stop()


async def low_level_loop(controller,
                         motors,
                         variables,
                         sources,
                         independent_variables,
                         duration,
                         default_freq,
                         n,
                         RECORD=None):
    """
    ``mjbots`` low level motor control loop.
    
    **Arguments**
    
    ``controller`` [instance of a class inheriting from ``AbstractController``]
      instance of an ``AbstractController`` child class
    
    ``motors`` [list of instances of motor classes inheriting from ``AbstractMotor``]
      list of instances of ``AbstractMotor`` child classes

    ``variables`` [list of str]
      list containing the names of the variables which will
      be recorded in the experiment (eg: ``['pos', 'vel', 'tor']``)

    ``sources`` [list of str]
      list containing the names of the sources from which values
      for each variable will be obtained (eg: ``['controller', 'sensors']``)

    ``independent_variables`` [list of str]
      list containing the names of **motor-independent** variables to be recorded from
      **within the controller**, which will thus **not undergo permutation**.
      In other words, the names of individual variables which will be set
      inside the ``provided`` controller.

      .. tip::

         The ``low_level_loop`` will keep track of the state of the motors
         through a manoeuvre. However, it may be of interest to record
         a variable, such as jump height in the case of a jumping robot.

         To do this, you must pass the variable ``jump_height`` in the
         ``independent_variables`` argument of this function.
         This will allow you to access an array called ``jump_height``
         of the ``record`` instance from within your ``controller``, as such::

            self.RECORD.jump_height

         At each iteration, you will have the possibility to calculate the
         jump height of your robot by any means, and store it in the record::

            jump_height = ...

            self.RECORD.jump_height[n] = jump_height

    ``duration`` [float]
      duration of the initialization manoeuvre

    ``default_freq`` [float]
      default control loop frequency, if none is provided in the motor
      configuration file

    ``n`` [int]
      number of iterations. If not provided, it will be obtained as the ceil
      of the product of the control frequency (either that provided in
      motor configuration, or ``default_freq`` otherwise) and experiment duration

    ``RECORD`` [``software.spine.record.record`` instance]
      experiment ``record`` object, containing the time vector of the
      experiment as well as the desired and measured state vectors
      of all motors in the experiment.
    
    **Output**

    ``RECORD`` [``software.spine.record.record`` instance]
      experiment ``record`` object, containing the time vector of the
      experiment as well as the desired and measured state vectors
      of all motors in the experiment.
    """

    # motor name list
    motor_names = [motor.name for motor in motors]

    # control frequency
    f = motors[0].control_freq if hasattr(motors[0], 'control_freq') else default_freq

    # iteration number
    if n is None:
        n = math.ceil(duration * f)

    # runtime variables
    i            = 0
    t            = np.zeros(n+1)
    t_iter       = 0
    t_elapsed    = 0
    
    #######################
    #     test record     #
    #######################

    if RECORD is None:
        RECORD       = record(t, np.zeros(n),
                              motor_names,
                              variables,
                              sources)

    # iteration index
    RECORD.i = 0

    # independent variables
    if independent_variables:
        RECORD.initialize_independent(n, independent_variables)

    # gains
    if hasattr(controller, 'get_gains'): RECORD.initialize(np.zeros(n), ['kp', 'kd'])

    # load
    controller._load_record(RECORD)    
    
    # initialize motors
    for motor in motors:
        await motor.set_stop()
    
    #######################
    #     control loop    #
    #######################
    while t_elapsed < duration:

        i           = RECORD.i
        RECORD.t[i] = t_elapsed
        _t          = time.time()
        
        # get control input
        _des = np.concatenate(controller.get_control_output(max(0, i-1)).T)
        
        # get gains
        if hasattr(controller, 'get_gains'): gains = controller.get_gains(i)
        
        # record control input
        RECORD.update(i, _des, motor_names, variables, 'des')

        #######################
        #    motor command    #
        #######################
        for motor in motors:
            
            # control input
            pos_des = RECORD[f'{motor.name}_pos_des'][i]/2/np.pi
            vel_des = RECORD[f'{motor.name}_vel_des'][i]/2/np.pi
            tau_des = RECORD[f'{motor.name}_tau_des'][i]

            # gains
            if hasattr(controller, 'get_gains'):
                kp, kd   = gains[motors.index(motor)]
                kp_scale = kp/motor.kp_default
                kd_scale = kd/motor.kd_default
            else:
                kp_scale = motor.kp_scale
                kd_scale = motor.kd_scale

            # gear ratio
            if hasattr(motor, 'gear_ratio'):
                pos_des = pos_des * motor.gear_ratio
                vel_des = vel_des * motor.gear_ratio
            
            # command
            _msr = await send_command(motor,
                                      position=pos_des,
                                      velocity=vel_des,
                                      kp_scale=kp_scale,
                                      kd_scale=kd_scale,
                                      stop_position=None,
                                      feedforward_torque=tau_des,
                                      maximum_torque=motor.torque_limit,
                                      watchdog_timeout=None,
                                      query=True)
            
            # record end effector state
            if hasattr(motor, 'gear_ratio'):
                _msr[:-1] = _msr[:-1] / motor.gear_ratio
            
            # record
            RECORD.update(i, _msr, motor.name, variables, 'msr')
            if hasattr(controller, 'get_gains'): RECORD.update(i, [kp, kd], motor.name, ['kp', 'kd'])
        
        RECORD.i += 1

        if f is not None:
            while time.time() - _t < 1/f:
                pass
        
        t_iter = time.time() - _t
        
        if t_iter > 1/f * (1 + 0.01):
            # slow control loop warning if iteration time exceeds
            # that imposed by the required control frequency + 1%
            print(f't = {t_elapsed:.2f} :: ITERATION TIME ABOVE LIMIT :: {f:.0f} > {1/t_iter:.0f} [Hz]')
        
        t_elapsed += t_iter


async def stage(desired_state, motors, stage, duration=2):
    """
    Staging function.
    
    Take all ``motors`` to their ``desired_state`` using
    ``software.spine.staging.Controller_Staging``.

    If the motors have Kp and Kd zero, these will be
    modified through the staging manoeuvre and restored
    afterwards.
    
    **Arguments**

    ``initial_state`` [list of floats]
      list of length ``m``, where ``m`` is the number of
      motors -the length of ``motors``-, containing the
      initial **angular position** of each motor.

    ``motors`` [list of objects]
      list of instances of ``AbstractMotor`` child classes

    ``duration`` [float]
      duration of the initialization manoeuvre
    """

    warn = f'STAGING -> {stage}'
    conf = 'REACHED'

    print(f'\n{warn}')

    # retrieve current state of the motors
    current_state = []
    for motor in motors:
        state, _, _ = await send_command(motor, position=math.nan)

        if hasattr(motor, 'gear_ratio'):
            state = state / motor.gear_ratio

        current_state.append(state)
        
    controller = Controller_Staging(current_state, desired_state, duration)
    
    await low_level_loop(controller            = controller,
                         motors                = motors,
                         variables             = ['pos', 'vel', 'tau'],
                         sources               = ['des', 'msr'],
                         independent_variables = [],
                         duration              = duration,
                         default_freq          = 250,
                         n                     = None)

    print(f'{"="*(len(warn) - len(conf) - 1)} {conf}\n')


async def motor_control_loop(controller,
                             motors,
                             duration,
                             variables=['pos', 'vel', 'tau'],
                             sources=['des', 'msr'],
                             independent_variables=[],
                             
                             default_freq=250,
                             n=None,
                             countdown=5,
                            
                             plot=True,
                             save=False,
                             directory=None,
                             series=None,
                             name=None,
                             timestamp=True):
    """
    ``mjbots`` high level motor control loop, wrapping ``set_initial_state`` and
    ``low_level_loop``.

    1. Manoeuvre to initial state if the provided ``controller`` has ``initial_state`` attribute.
    2. Execute manoeuvre by calling ``low_level_loop`` with the provided ``controller``
    3. Manoeuvre to final state if the provided ``controller`` has ``final_state`` attribute.
    
    **Arguments: Low Level Loop**

    *Mandatory:*
    
    ``controller`` [instance of a class inheriting from ``AbstractController``]
      instance of an ``AbstractController`` child class
    
    ``motors`` [list of instances of motor classes inheriting from ``AbstractMotor``]
      list of instances of ``AbstractMotor`` child classes

    ``duration`` [float]
      duration of the initialization manoeuvre

    *Optional*
    
    ``variables`` [list of str]
      list containing the names of the variables which will
      be recorded in the experiment (eg: ``['pos', 'vel', 'tor']``)

    ``sources`` [list of str]
      list containing the names of the sources from which values
      for each variable will be obtained (eg: ``['controller', 'sensors']``)

    ``independent_variables`` [list of str]
      list containing the names of **motor-independent** variables to be recorded from
      **within the controller**, which will thus **not undergo permutation**.
      In other words, the names of individual variables which will be set
      inside the ``provided`` controller.

      .. tip::

         The ``low_level_loop`` will keep track of the state of the motors
         through a manoeuvre. However, it may be of interest to record
         a variable, such as jump height in the case of a jumping robot.

         To do this, you must pass the variable ``jump_height`` in the
         ``independent_variables`` argument of this function.
         This will allow you to access an array called ``jump_height``
         of the ``record`` instance from within your ``controller``, as such::

            self.RECORD.jump_height

         At each iteration, you will have the possibility to calculate the
         jump height of your robot by any means, and store it in the record::

            jump_height = ...

            self.RECORD.jump_height[n] = jump_height

    ``default_freq`` [float]
      default control loop frequency, if none is provided in the motor
      configuration file

    ``n`` [int]
      number of iterations

    ``countdown`` [float]
      countdown time before experiment start in seconds

    **Arguments: Data Acquisition and Visualization**

    ``plot`` [boolean]
      whether to plot the desired and measured position,
      velocity and torque in each motor

    ``save`` [boolean]
      whether to save the ``record`` object containing
      all experiment measurements in a CSV file

    ``directory`` [str]
      directory where the record directories of all experiments are
      saved. This means your project is expected to have a **single**
      directory where all data is stored. This is considered rather
      convenient for automatically storing data.
      Of course, you are free to do with your data what you will after
      it has been generated, and it is in fact recommended to only keep
      relevant data after experiment sessions (that is, to delete all
      data from failed/spoiled experiments), and to avoid saving data
      together with the source code of your project to keep ``.git``
      folders, and so cloning times, reasonably small.
    
      ``directory`` **must** point to a directory. Otherwise ``pathlib`` will
      raise an error. If ``directory`` does not exist, it will be created

    ``series`` [str]
      inside ``directory``, a ``series`` directory will be created to store the
      data of all runs of the current experiment.
      This is convenient to separate data from different sources

    ``name`` [str]
      name of the current experiment. If it is left as ``''``, each run's
      record directory will be named with a timestamp

    ``timestamp`` [bool]
      whether to include a timestamp in the CSV file name. This is
      highly recommended to avoid overwriting experiment data, as
      well as keeping track of when certain results were obtained
      (without having to look into file metadata)
    
    **Output**

    ``RECORD`` [``record`` instance]
      experiment ``record`` object, containing the time vector of the
      experiment as well as the desired and measured state vectors
      of all motors in the experiment.
    """

    #######################
    #     test record     #
    #######################

    # motor name list
    motor_names = [motor.name for motor in motors]
    
    # control frequency
    f = motors[0].control_freq if hasattr(motors[0], 'control_freq') else default_freq
    
    # iteration number
    if n is None:
        n = math.ceil(duration * f)
    
    RECORD = record(np.zeros(n),
                    np.zeros(n),
                    motor_names,
                    variables,
                    sources)

    if save:
        path = record_directory(directory, series, name, timestamp)

    #######################
    #    control loop     #
    #######################
    try:
    
        # countdown
        _countdown(count=countdown)
    
        # go to initial state if the controller has an ``initial_state`` attribute
        if hasattr(controller, 'initial_state'):
            await stage(controller.initial_state, motors, 'INITIAL STATE')
        
        # control loop
        print('\n.\n')
        await low_level_loop(controller            = controller,
                             motors                = motors,
                             duration              = duration,
                             n                     = n,
                             default_freq          = default_freq,
                             variables             = variables,
                             sources               = sources,
                             independent_variables = independent_variables,
                             RECORD                = RECORD)
        
        # go to final state if the controller has an ``final_state`` attribute
        if hasattr(controller, 'final_state'):
            await stage(controller.final_state, motors,   '  FINAL STATE')
    
    finally:

        print('\n=============')
        print('TEST FINISHED')
        print('=============\n')
        
        # disable motors
        for motor in motors:
            await motor.set_stop()
            print(f'STOPPED MOTOR {motors.index(motor)}\n')

        # clean record
        RECORD.clean()

        if plot:
            print('GENERATING PLOTS\n')
            plot_record(RECORD)

        if save:
            print(f'SAVING\n')

            # record
            RECORD.save(os.path.join(path, 'record.csv'))
            # motor configuration
            shutil.copyfile(motors[0].config, os.path.join(path, 'robot.toml'))
            # controller parameters
            if hasattr(controller, 'save'):
                controller.save(path)
            # plot
            plot_record(RECORD,
                        save=True,
                        show=False,
                        filename=os.path.join(path, 'record'))
    
    return RECORD
