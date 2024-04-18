"""
General
=======
"""

import copy
import itertools


def varinit(val, *args):
    """
    Initialize a set of variables, obtained from the permutation 
    of the lists provided as positional arguments, and provide
    them to the user in the form of a dictionary.
    """

    variables = args[0]

    if len(args) > 1:
        for sequence in args[1:]:
            variables = ['_'.join(t) for t in itertools.product(variables, sequence)]

    return {variable: copy.deepcopy(val) for variable in variables}


def varget(namespace, *args):
    """
    Retrieve all variables in the given namespace which
    contain every single string provided as a positional
    argument.
    """
    return [key for key in namespace.keys() if all([kw in key for kw in args])]
