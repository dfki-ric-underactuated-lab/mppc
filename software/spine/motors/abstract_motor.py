"""
Abstract Motor Class
====================
"""

from abc import ABC, abstractmethod



class AbstractMotor(ABC):
    """
    Abstract motor class.

    Simple enough: if you try to retrieve an attribute from a class inheriting from
    this one that it **doesn't have**, it will try to retrieve it from the ``motor``
    attribute before failing.

    That way we can create motor classes effectively wrapping the various low level
    APIs of ``mjbots``, ``tmotor``, ``pybullet`` and what not.
    """

    def __getattr__(self, name):
        """
        Retrieve unknown attributes from motor controller.
        """
        try:
            return getattr(self.__dict__['motor'], name)
        except AttributeError:
            raise AttributeError(f"{self.__class__.__name__} has no attribute \"{name}\"")
