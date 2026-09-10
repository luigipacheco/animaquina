# coding=utf-8

"""
Module implementing the SO(3) interpolator class; Slerp.
"""

__author__ = "Morten Lind"
__copyright__ = "Morten Lind 2009-2021"
__credits__ = ["Morten Lind"]
__license__ = "LGPLv3"
__maintainer__ = "Morten Lind"
__email__ = "morten@lind.fairuse.org"
__status__ = "Production"


import typing

import numpy as np

from ..orientation import Orientation, RotationVector
from ..quaternion import Versor
from .. import utils
from .interpolator import Interpolator


class SO3Interpolator(Interpolator):
    """A SLERP interpolator class in SO(3)."""

    class Error(Exception):
        """Exception class."""

        def __init__(self, message):
            self.message = 'SO3Interpolation Error: ' + message
            Exception.__init__(self, self.message)

        def __repr__(self):
            return self.message

    def __init__(self,
                 start: typing.Union[Orientation, Versor, RotationVector],
                 end: typing.Union[Orientation, Versor, RotationVector],
                 t_range: tuple[float, float] = None,
                 shortest: bool = True):
        """Initialise an SO(3) interpolation from orientation 'start' to
        orientation 'end'. If t_range is given, it must be a time
        interval in form of a sorted pair of floats. If 'shortest' is
        True the shortest rotation path is chosen, if False the long
        rotation is used, and if None it is indeterminate, given by
        the Versor objects constructed over 'start' and 'end'.
        """
        Interpolator.__init__(self, t_range)
        self._qstart = Versor(start)
        self._qend = Versor(end)
        self._qstart.normalize()
        self._qend.normalize()
        if shortest is not None:
            if shortest and (self._qstart.dist(self._qend) >
                             self._qstart.dist(-self._qend)):
                self._qend = -self._qend
            elif not shortest and (self._qstart.dist(self._qend) <
                                   self._qstart.dist(-self._qend)):
                self._qend = -self._qend
        self._qstartconj = self._qstart.conjugated.normalized
        self._qstartconjqend = (self._qstartconj * self._qend).normalized

    def __call__(self, t, checkrange=True):
        return self.versor(t, checkrange)

    def versor(self, t, checkrange=True):
        """Return the versor of the slerp at 'time'; in [0,1]."""
        return self._qstart * (self._qstartconjqend) ** self._s(t, checkrange)

    def orient(self, time, checkrange=True):
        """Return the orientation in the slerp at 'time'; in [0,1]. """
        return self.versor(time, checkrange).orientation


SO3Interpolation = SO3Interpolator
SLERP = SO3Interpolator
OrientationInterpolation = SO3Interpolator


def _test():
    """Simple test function."""
    global o, o1, q, q1, osl, qsl
    from math import pi
    o = Orientation()
    o.set_to_x_rotation(pi / 2)
    o1 = Orientation()
    o1.set_to_z_rotation(pi / 2)
    q = Versor(o)
    q1 = Versor(o1)
    qsl = SO3Interpolator(q, q1)
    osl = SO3Interpolator(o, o1)
    print(osl(0.5), qsl(0.5))
