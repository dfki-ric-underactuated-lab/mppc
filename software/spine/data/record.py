"""
Record Object
=============
"""

import os
import time
import pathlib
import itertools
import numpy as np
import pandas as pd
from datetime import datetime

from software.spine.utils.init import varinit


def record_directory(directory, series, name='', timestamp=True, parent_file=None):
    """
    Return the name of the directory where an experiment's record files will be saved,
    in the following format::

       <directory>/<series>/<name>_<timestamp if wanted>/
    
    **Arguments**

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
      This is convenient to separate data from different sources.
      If series is ``''``, data will be saved in directories directly under
      ``directory``

    ``name`` [str]
      name of the current experiment. If it is left as ``''``, each run's
      record directory will be named with a timestamp

    ``timestamp`` [bool]
      whether to include a timestamp in the CSV file name. This is
      highly recommended to avoid overwriting experiment data, as
      well as keeping track of when certain results were obtained
      (without having to look into file metadata)

    ``parent_file`` [str]
      if ``parent_file`` is provided, ``directory`` will be considered a
      relative path with respect to ``parent_file``
    """

    assert all([not isinstance(arg, type(None)) for arg in [directory, series, name]]), '*directory*, *series* and *name* cannot be none'

    # path
    if parent_file is not None:
        path = os.path.join(os.path.dirname(os.path.realpath(__file__)), directory)
    else:
        path = directory

    # timestamp
    timestamp = (('_' if name != '' else '') + datetime.now().strftime("%Y.%m.%d_%H.%M.%S")) if timestamp else ''
    
    # directory
    directory = os.path.join(directory, series, f'{name}{timestamp}')
    
    # create if it does not exist
    pathlib.Path(directory).mkdir(parents=True, exist_ok=True)
    
    return directory


class record:
    """
    Experiment Record Object

    ``record`` instances contain the time vector of the
    experiment, as well as the desired and measured state vectors
    of all motors in the experiment.
    """
    
    def __init__(self, t, init_value, motor_names, variables, sources):
        """
        Record initialization

        Create ``record`` instance attributes:

        * Time vector ``t``
        * Motor names
        * Experiment variables
        * Sources of experiment variable values
        * All permutations of the form ``<motor_name>.<variable>.<source>``,
          initialized to the provided ``init_value``
        
        -----
        
        **Arguments**

        ``t`` [np.ndarray]
          experiment time vector

        ``init_value`` [np.ndarray]
          initial value of all recorded state variables

        ``motor_names`` [list of str]
          list containing the name (a distinctive string) of each
          motor in the system

        ``variables`` [list of str]
          list containing the names of the variables which will
          be recorded in the experiment (eg: ``['pos', 'vel', 'tor']``)

        ``sources`` [list of str]
          list containing the names of the sources from which values
          for each variable will be obtained (eg: ``['controller', 'sensors']``)
        """

        self.t           = t
        
        self.motor_names = motor_names
        self.variables   = variables
        self.sources     = sources
        self.other       = []
        self.independent = []
        
        self._initialize(init_value, self.variables, self.sources)
        

    def __getitem__(self, item):
        """
        Retrieve recorded variables using bracket syntax, by
        providing an ``item`` which may be either:

        * a string with the name of the variable
        * a numerical index of the record's ``columns()``
        * a slice object, applied to the record's ``columns()``
        """
        if isinstance(item, int) or isinstance(item, slice):
            return self.__dict__[self.columns()[item]]
        else:
            return self.__dict__[item]

    def _permutate(self, caller, *args):
        """
        Permutate the parameter lists input as ``args``
        """

        sequences = list(args)
        
        # input validation ---------------------------------------------------------------------------
        for s in range(len(sequences)):
            seq = sequences[s]
            if not isinstance(seq, list):
                assert isinstance(seq, str), f'{caller} :: inputs to variable permutation must be strings or lists of strings'
                sequences[s] = [seq]
        assert {e for seq in sequences for e in seq}.issubset(self.motor_names + self.variables +  self.sources + self.other), f'{caller} :: attempted to access unknown variable'
        # --------------------------------------------------------------------------------------------
        
        permutation = sequences.pop(0)

        for sequence in sequences:
            
            permutation = ['_'.join(t) for t in itertools.product(permutation, sequence)]

        return permutation

    def _matrix(self, sources, motor_index=0, motor_names=None):
        """
        Output a ``3*m*s`` by ``n`` matrix, where ``m`` is the number of motors,
        ``s`` the number of record sources (eg: ``des`` and ``msr`` for desired
        -control input- and measured states respectively) and ``n`` the iteration
        number of the recorded experiment, containing the recorded position, velocity
        and torque of each motor, from each source, as rows
        """
        
        if not motor_names:
            motor_names = self.motor_names[motor_index]

        params = self._permutate('matrix', motor_names, self.variables, sources)
        
        return np.stack([self.__dict__[p] for p in params])

    def _initialize(self, init_value, *args):
        """
        Initialize all the variables resulting from the permutation of
        ``self.motor_names`` and the strings in each list of strings
        provided as a positional argument.

        **Arguments**
        
        ``init_value`` [np.ndarray]
          initial value of all recorded state variables

        ``args`` [list of str]
          any number of lists of strings input after ``init_value``
        """
        
        self.__dict__.update(varinit(init_value, self.motor_names, *args))

    def initialize(self, init_value, *args):
        """
        Non-state, **motor-specific**, variable initialization (eg: gains).
        These will be recorded apart from the rest (internally, they will
        be part of the ``other`` variable list.
        
        Initialize all the variables resulting from the permutation of
        ``self.motor_names`` and the strings in each list of strings
        provided as a positional argument.

        **Arguments**
        
        ``init_value`` [np.ndarray]
          initial value of all recorded state variables

        ``args`` [list of str]
          any number of lists of strings input after ``init_value``
        """

        for l in args:
            self.other += l

        self._initialize(init_value, *args)

    def initialize_independent(self, n, *args):
        """
        Non-state, **non-motor-specific**, variable initialization (eg:
        ``jump_height`` for jumping robots).
        These will be recorded apart from the rest (internally, they will
        be part of the ``independent`` variable list.
        
        As these arguments will **not** be permutated at any point,
        the positional ``args`` may be lists of strings or strings
        themselves.

        **Furthermore, the ``args`` may be (variable name, Numpy data type) tuples**.
        This allows the user to initialize arrays with different data types among
        the independent variables. This is useful when, for example, storing
        the *state* of a system during a manoeuvre in the form of strings, such as "LIFTOFF",
        "FLIGHT" and "TOUCHDOWN" in the case of a hopping robot.

        **Arguments**
        
        ``init_value`` [np.ndarray]
          initial value of all recorded state variables
        
        ``args`` [list of str]
          any number of lists of strings input after ``init_value``
        """

        # input validation ---------------------------------------------------------------------------
        args = list(args)
        # --------------------------------------------------------------------------------------------
        
        for variable in args:
            if isinstance(variable, list):
                self.initialize_independent(n, *variable)
            else:
                # read NumPy data type from (variable, np.dtype) tuples in the VARIABLES argument
                if isinstance(variable, tuple):
                    variable, dtype = variable
                else:
                    dtype = np.float

                assert not hasattr(self, variable), f"the independent variable *{variable}* would overwrite an attribute with the same name of the experiment's ``software.spine.record.record`` object. Please choose a different name for this independent variable."
                    
                self.independent.append(variable)
                self.__dict__.update({variable: np.empty(n, dtype)})

    def update(self, i, values, motor_names, *args):
        """
        Update the ``i``th element of the record variables resulting from the
        permutation of ``self.motor_names`` and the strings in each list
        of strings provided as a positional argument.

        **Arguments**
        
        ``init_value`` [np.ndarray]
          initial value of all recorded state variables

        ``args`` [list of str]
          any number of lists of strings input after ``init_value``
        """

        params = self._permutate('update', motor_names, *args)
        
        for k in range(len(params)):
            self.__dict__[params[k]][i] = values[k]

    def clean(self):
        """
        Remove all entries of each recorded array after index ``record.i``.

        Say that you are running an experiment and for some reason interrupt it.
        The arrays in the ``record`` object of the experiment were allocated to fit
        more data than has been recorded: after the index of last iteration reached,
        the arrays will only contain placeholder data which will not be useful, and
        needs to be cleaned. It is convenient to automatically remove this data.
        
        This function allows you to store the index of the current iteration of an
        experiment in the ``record.i`` variable, and then eliminate all content in the
        variable arrays of the ``record`` after index ``i``.
        """

        assert hasattr(self, "i"), "to **clean** a **record** object, it must have an **i** attribute representing the last iteration number reached"

        columns = self.columns()
        
        for array in columns:
            setattr(self, array, getattr(self, array)[:self.i])

    def columns(self):
        """
        Return a list containing all recorded variables.
        """

        return ['t'] + \
               self._permutate('columns', self.motor_names, self.variables, 'msr') + \
               self._permutate('columns', self.motor_names, self.variables, 'des') + \
               self._permutate('columns', self.motor_names, self.other) + \
               self.independent
    
    def matrix_des(self, motor_index=0, motor_names=None):
        return self._matrix('des', motor_index, motor_names)
            
    def matrix_msr(self, motor_index=0, motor_names=None):
        return self._matrix('msr', motor_index, motor_names)

    def state(self, n):
        """
        Return a `3` (position, velocity, torque) by `m` (number
        of motors) matrix the columns of which are the state
        vectors of each motor at timestep ``n``.
        """

        matrix = []

        for id in self.motor_names:
            params = self._permutate('matrix', id, self.variables, 'msr')
            matrix.append(np.vstack([self.__dict__[p][n] for p in params]))

        return np.hstack(matrix)

    def to_df(self):

        columns = self.columns()
        
        return pd.DataFrame({p: self.__dict__[p] for p in columns})

    def save(self, filename):
        """
        Save ``experiment`` record as a CSV file with the given ``filename``.

        The ``filename`` may be absolute or relative to the directory from
        which Python is being run.

        If the path does not exist an error will be raised. This is not
        prevented to ensure the user has conscious control of where their
        experiment data is being saved.
        """

        assert filename != None, 'you must provide a filename to save the record of your experiment'

        self.path = os.path.dirname(filename)

        df = self.to_df()
        
        # mark columns of /other/ and /independent/ parameters
        df.columns = [f"{col}::o" if col in self.other else \
                      f"{col}::i" if col in self.independent else \
                      col for col in df.columns]
        
        df.to_csv(filename, index=False)
        
    @classmethod
    def load(self, filename):
        """
        Load ``record`` save in previous experiment.
        """
        
        RECORD      = pd.read_csv(filename)
        t           = RECORD.pop(RECORD.columns[0]).to_numpy()
        params      = RECORD.columns.to_list()
        
        init_value  = np.empty(len(RECORD))

        # source lists
        motor_names = []
        variables   = []
        sources     = []

        # parameter lists
        state       = []
        other       = []
        independent = []
        
        for param in params:

            if param[-3:] == "::o":
                other.append(param[:-3])
            if param[-3:] == "::i":
                independent.append(param[:-3])
            else:

                state.append(param)
            
                nodes = param.split('_')
            
                motor_names.append(nodes[0])
                variables  .append(nodes[1])
                sources    .append(nodes[2])
        
        # remove duplicates
        motor_names = list(dict.fromkeys(motor_names))
        variables   = list(dict.fromkeys(variables))
        sources     = list(dict.fromkeys(sources))

        # create record object
        _record = record(t, init_value, motor_names, variables, sources)
        _record.other       = other
        _record.independent = independent

        # store parsed values in newly created record object
        for param in state:
            setattr(_record, param, RECORD[param].to_numpy())

        for param in other:
            setattr(_record, param, RECORD[f"{param}::o"].to_numpy())

        for param in independent:
            setattr(_record, param, RECORD[f"{param}::i"].to_numpy())
            
        return _record

