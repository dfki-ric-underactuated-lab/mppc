"""
``pybullet``
============
"""

import os
import math
import time
import shutil
import itertools
import numpy as np
from copy import deepcopy as dc

# pybullet
import pybullet as pb

# spine
from software.spine.data.record import record
from software.spine.data.plot import plot_record
from software.spine.data.record import record_directory
from software.spine.utils.loop import countdown as _countdown

from software.spine.controller.staging import Controller_Staging


def send_command(robot,
                 joint,
                 force,
                 mode,
                 client,
                 **kwargs):
    """
    ``pybullet`` motor control command.

    **Arguments**

    ``robot`` [int]
      numerical identifier of robot object created
      by ``pybullet.loadURDF`` from the provided ``urdf``

    ``joint`` [int]
      numerical identifier of the joint to which a torque
      command will be sent

    ``force`` [float]
      if ``mode`` is ``pb.TORQUE_CONTROL``, ``force`` is the
      torque applied. Else, ``force`` is the torque limit of
      the joint.

    ``mode`` [int]
      numerical identifier of the desired control mode. Set
      this variable to either ``pybullet.POSITION_CONTROL``,
      ``pybullet.VELOCITY_CONTROL`` or ``pybullet.TORQUE_CONTROL``

    ``client`` [int]
      numerical identifier of the pybullet physics client
      running the current simulation

    ``kwargs`` [dict]
      further keyword arguments for ``pybullet.setJointMotorControl2``
    """
    
    # command
    pb.setJointMotorControl2(bodyIndex       = robot,
                             jointIndex      = joint,
                             force           = force,
                             controlMode     = mode,
                             physicsClientId = client,
                             **kwargs)


def read_state(robot,
               motor,
               client):
    """
    ``pybullet`` motor (joint) state retrieval command
    """
    
    pos_msr, vel_msr, _, tau_msr = pb.getJointState(robot,
                                                    motor.joint,
                                                    physicsClientId=client)

    return np.array([pos_msr, vel_msr, tau_msr])
    
    
def low_level_loop(controller,
                   client,
                   mode,
                   robot,
                   motors,
                   timestep_delay,
                   variables,
                   sources,
                   independent_variables,
                   duration,
                   default_freq,
                   n,
                   RECORD=None):
    """
    ``pybullet`` low level motor control loop.
    
    The control loop operates as follows.

       0. Initialization
       1. Reading and recording the state of each motor (joint in ``pybullet``)
       1. Calculating the control input
       2. Providing the control input to each motor
       4. Propagating the simulation
    
    .. admonition:: IMPORTANT
       :class: danger
    
       The low level software of certain physical systems, such as ``tmotor``
       motors, do **not** grant the user telemetry streams, and instead
       provide the state of the system as output of motor control command.
    
       This means that the state used to calculate the control input
       in such systems has a **delay** of **1 timestep**. This becomes
       especially problematic with low control frequencies.

       An accurate simulation **must** account for this effect, especially
       if it is to be used for training of reinforcement and other learning
       control algorithms.

       Thus, this ``low_level_loop`` offers the user the possibility to mimic
       this delay, by, at timestep ``i``, calculating the control input with
       the state recorded at timestep ``i-1``. This behavior can be activated
       by setting ``timestep_delay`` to ``True``.
    
    **Arguments**
    
    ``controller`` [instance of a class inheriting from ``AbstractController``]
      instance of an ``AbstractController`` child class

    ``client`` [int]
      numerical identifier of the pybullet physics client
      running the current simulation

    ``mode`` [int]
      numerical identifier of the desired control mode. Set
      this variable to either ``pybullet.POSITION_CONTROL``,
      ``pybullet.VELOCITY_CONTROL`` or ``pybullet.TORQUE_CONTROL``
    
    ``robot`` [int]
      numerical identifier of robot object created
      by ``pybullet.loadURDF`` from the provided ``urdf``
    
    ``motors`` [list of instances of motor classes inheriting from ``AbstractMotor``]
      list of instances of ``AbstractMotor`` child classes

    ``timestep_delay`` [bool]
      whether to simulate the 1 timestep control input delay experienced
      by systems the state of which may only be read as the output of
      motor control commands.
      That is, systems the drivers of which only return motor state readings
      from control commands, which do not support telemetry streams, etc.
      Examples include the DFKI `CAN motor control library <https://github.com/dfki-ric-underactuated-lab/mini-cheetah-tmotor-python-can>`_ for ``tmotor`` quasi-direct drive motors.

    ``duration`` [float]
      duration of the initialization manoeuvre

    ``n`` [int]
      number of iterations. If not provided, it will be obtained as the ceil
      of the product of the control frequency (either that provided in
      motor configuration, or ``default_freq`` otherwise) and experiment duration

    ``default_freq`` [float]
      default control loop frequency, if none is provided in the motor
      configuration file

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
    t_iter       = 0
    t_elapsed    = 0

    #######################
    #     test record     #
    #######################

    if RECORD is None:
        RECORD = record(np.zeros(n),
                        np.zeros(n),
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

    # record initial state
    for motor in motors:
        _msr = read_state(robot,
                          motor,
                          client)
        RECORD.update(0, _msr, motor.name, variables, 'msr')
    
    #######################
    #     control loop    #
    #######################
    while t_elapsed < duration:
        
        i           = RECORD.i
        RECORD.t[i] = t_elapsed
        _t          = time.time()
        
        # read and record motor state
        for motor in motors:
            _msr = read_state(robot,
                              motor,
                              client)
            RECORD.update(i, _msr, motor.name, variables, 'msr')
        
        # control input
        _des = np.concatenate(controller.get_control_output(i if not timestep_delay else max(0, i-1)).T)
        RECORD.update(i, _des, motor_names, variables, 'des')
        
        # input gains
        if hasattr(controller, 'get_gains'): gains = controller.get_gains(i)
        
        #######################
        #    motor command    #
        #######################
        for motor in motors:

            # measured state
            pos_msr = RECORD[f'{motor.name}_pos_msr'][i]
            vel_msr = RECORD[f'{motor.name}_vel_msr'][i]
            tau_msr = RECORD[f'{motor.name}_tau_msr'][i]
            
            # control input
            pos_des = RECORD[f'{motor.name}_pos_des'][i]
            vel_des = RECORD[f'{motor.name}_vel_des'][i]
            tau_des = RECORD[f'{motor.name}_tau_des'][i]
            
            # gains
            if hasattr(controller, 'get_gains'):
                kp, kd = gains[motors.index(motor)]
                # record gain input
                RECORD.update(i, [kp, kd], motor.name, ['kp', 'kd'])
            else:
                kp = motor.kp
                kd = motor.kd
            
            # command
            if mode == pb.TORQUE_CONTROL:
                tau = kp*(pos_des - pos_msr) + kd*(vel_des - vel_msr) + tau_des
                tau = tau_msr = np.sign(tau) * min(abs(tau), motor.torque_limit)
                
                args = {'force': tau}
                
            else:
                
                args = {
                    'force':        motor.torque_limit,
                    'positionGain': kp,
                    'velocityGain': kd
                }
                
                args.update({'targetPosition': pos_des} if mode == pb.POSITION_CONTROL else {'targetVelocity': vel_des})
            
            send_command(robot,
                         motor.joint,
                         mode=mode,
                         client=client,
                         **args)
            
            # update torque reading
            RECORD[f'{motor.name}_tau_msr'][i] = tau_msr
        
        #######################
        #     propagation     #
        #######################
        
        pb.stepSimulation(physicsClientId=client)
            
        RECORD.i += 1
        
        while time.time() - _t < 1/f:
            pass
        
        t_iter = time.time() - _t
        
        # if t_iter > 1/f * (1 + 0.01):
            # slow control loop warning if iteration time exceeds
            # that imposed by the required control frequency + 1%
            # print(f't = {t_elapsed:.2f} :: ITERATION TIME ABOVE LIMIT :: {f:.0f} > {1/t_iter:.0f} [Hz]')
        
        t_elapsed += t_iter


def stage(desired_state, client, mode, robot, motors, stage,
          duration=2,
          default_freq=1000):
    """
    Staging function.
    
    Take all ``motors`` to their ``desired_state`` using
    ``software.spine.staging.Controller_Staging``.

    If the motors have Kp and Kd zero, these will be
    modified through the staging manoeuvre and restored
    afterwards.
    
    **Arguments**

    ``client`` [int]
      numerical identifier of the pybullet physics client
      running the current simulation

    ``mode`` [int]
      numerical identifier of the desired control mode. Set
      this variable to either ``pybullet.POSITION_CONTROL``,
      ``pybullet.VELOCITY_CONTROL`` or ``pybullet.TORQUE_CONTROL``
    
    ``robot`` [int]
      numerical identifier of robot object created
      by ``pybullet.loadURDF`` from the provided ``urdf``

    ``initial_state`` [list of floats]
      list of length ``m``, where ``m`` is the number of
      motors -the length of ``motors``-, containing the
      initial **angular position** of each motor.

    ``motors`` [list of objects]
      list of instances of ``AbstractMotor`` child classes

    ``duration`` [float]
      duration of the initialization manoeuvre

    ``default_freq`` [float]
      default control frequency of the initialization manoeuvre

    ``plot`` [boolean]
      whether to plot the desired and measured position,
      velocity and torque in each motor
    """

    warn = f'STAGING -> {stage}'
    conf = 'REACHED'

    print(f'\n{warn}')

    # retrieve current state of the motors
    current_state = []
    for motor in motors:
        p, _, _, _ = pb.getJointState(robot,
                                      motor.joint,
                                      physicsClientId=client)
        
        current_state.append(p)
        
    controller = Controller_Staging(current_state, desired_state, duration)
    
    RECORD = low_level_loop(controller            = controller,
                            client                = client,
                            mode                  = mode,
                            robot                 = robot,
                            motors                = motors,
                            timestep_delay        = True,
                            variables             = ['pos', 'vel', 'tau'],
                            sources               = ['des', 'msr'],
                            independent_variables = [],
                            duration              = duration,
                            default_freq          = default_freq,
                            n                     = None)

    print(f'{"="*(len(warn) - len(conf) - 1)} {conf}\n')


def motor_control_loop(controller,
                       client,
                       mode,
                       robot,
                       motors,
                       timestep_delay,
                       duration,
                       
                       variables=['pos', 'vel', 'tau'],
                       sources=['des', 'msr'],
                       independent_variables=[],
                       
                       n=None,
                       default_freq=250,
                       countdown=5,
                       
                       plot=True,
                       video=False,
                       save=False,
                       directory=None,
                       series=None,
                       name=None,
                       timestamp=True):
    """
    ``pybullet`` high level motor control loop, wrapping ``set_initial_state`` and
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

    ``timestep_delay`` [bool]
      whether to simulate the 1 timestep control input delay experienced
      by systems the state of which may only be read as the output of
      motor control commands.
      That is, systems the drivers of which only return motor state readings
      from control commands, which do not support telemetry streams, etc.
      Examples include the DFKI `CAN motor control library <https://github.com/dfki-ric-underactuated-lab/mini-cheetah-tmotor-python-can>`_ for ``tmotor`` quasi-direct drive motors.

    *Optional:*
    
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

    ``video`` [boolean]
      whether to save a video of the simulation

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

    if video:
        filename = os.path.join(path, 'sim.mp4') if save else './sim.mp4'
        video_id = pb.startStateLogging(loggingType=pb.STATE_LOGGING_VIDEO_MP4,
                                        fileName=filename)
    
    #######################
    #    control loop     #
    #######################
    
    try:
        
        # free joints for pure torque control
        if mode == pb.TORQUE_CONTROL:

            for motor in motors:
                
                send_command(robot,
                             motor.joint,
                             force=0,
                             mode=pb.VELOCITY_CONTROL,
                             client=client)
                
        # countdown
        _countdown(count=countdown)

        # go to initial state if the controller has an ``initial_state`` attribute
        if hasattr(controller, 'initial_state'):
            stage(controller.initial_state,
                  client,
                  mode,
                  robot,
                  motors,
                  'INITIAL STATE')
        
        # control loop
        print("Motor Control Loop Active \n")

        low_level_loop(controller            = controller,
                       client                = client,
                       mode                  = mode,
                       robot                 = robot,
                       motors                = motors,
                       timestep_delay        = timestep_delay,
                       duration              = duration,
                       n                     = n,
                       default_freq          = default_freq,
                       variables             = variables,
                       sources               = sources,
                       independent_variables = independent_variables,
                       RECORD                = RECORD)
        
        # go to final state if the controller has an ``final_state`` attribute
        if hasattr(controller, 'final_state'):
            stage(controller.final_state,
                  client,
                  mode,
                  robot,
                  motors,
                  '  FINAL STATE')
    
    finally:
        
        print('\n=============')
        print('TEST FINISHED')
        print('=============\n')
        
        # if video:
        #     pb.stopStateLogging(video_id)
        
        # disconnect from simulation
        if pb.isConnected():
            pb.disconnect()
        
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
