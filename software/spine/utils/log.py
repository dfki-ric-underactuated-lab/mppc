"""
Logging
=======
"""


def log_experiment(experiment, system):
    """
    Print a string in the following form::

       =================
       <experiment name>
            <system>
       =================

    **Arguments**

    ``experiment`` [str]
      string containing the name of the experiment

    ``system`` [str]
      string containing the name of the system on which
      the experiment is being run
    """

    print(f'\n{"="*len(experiment)}')
    print(experiment)
    print(f"{' '*(int((len(experiment)-4-len(system))/2))}| {system} |{' '*(int((len(experiment)-4-len(system))/2))}")
    print(f'{"="*len(experiment)}\n')
