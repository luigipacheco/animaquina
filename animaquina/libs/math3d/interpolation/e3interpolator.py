# coding=utf-8

"""
Module implementing the R^3 interpolator class.
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

from ..vector import Vector, PositionVector, FreeVector
from .. import utils
from .interpolator import Interpolator


class E3Interpolator(Interpolator):
    """Simple linear position interpolation in R^3."""

    class Error(Exception):
        """Exception class."""

        def __init__(self, message):
            self.message = 'R3Interpolation Error: ' + message
            Exception.__init__(self, self.message)

        def __repr__(self):
            return self.message

    def __init__(self,
                 p0: typing.Union[PositionVector, FreeVector],
                 p1: typing.Union[PositionVector, FreeVector],
                 t_range: tuple[float, float] = None,
                 v0: float = None):
        """Make a position interpolation between 'p0' and' p1'. 'p0' and 'p1'
        must be suitable data for creation of Vectors. If 't_range' is given it is a pair"""
        super().__init__(t_range)
        self._v0 = v0
        self._p0 = p0
        if not isinstance(p0, Vector):
            self._p0 = Vector(p0)
        self._p1 = p1
        if not isinstance(p1, Vector):
            self._p1 = Vector(p1)
        self._displ = self._p1 - self._p0
        self._dlength = self._displ.length
        if self._v0 is not None:
            # Calculate a constant acceleration trajectory
            if self._t_range is None:
                # If no time range was given for the path, establish the trivial one
                self._t_range = (0.0, 1.0)
            self._dur = self._t_range[1] - self._t_range[0]
            self._acc = 2 * (self._dlength - self._v0 *
                             self._dur) / self._dur**2

    def _s(self, t, checkrange=True):
        """Special calculus for path parameter possibly taking acceleration
        into account.
        """
        if self._v0 is None:
            return super()._s(t, checkrange)
        else:
            t_path = t - self._t_range[0]
            s = (self._v0 * t_path + 0.5 * self._acc * t_path**2) / self._dlength
            if checkrange:
                super()._checkrange(s)
            return s

    def pos(self, t, checkrange=True):
        """Called to get the interpolated position at time 't'."""
        s = self._s(t, checkrange)
        return self._p0 + self._displ * s

    __call__ = pos


R3Interpolator = E3Interpolator
R3Interpolation = E3Interpolator
PositionInterpolation = E3Interpolator


def _test_range():
    """Simple test function."""
    global p0, p1, pint
    p0 = [0, 1, 0]
    p1 = [1, 0, 1]
    pint = E3Interpolator(p0, p1, t_range=(-1.0, 1.0))
    print(pint(0.0))
    assert(pint(0.0) == m3d.FreeVector([0.5, 0.5, 0.5]))


def _test_accel():
    global p0, p1, v0, pint, t_range, ts, ps, ss, plt
    import matplotlib.pyplot as plt
    import numpy as np
    import math3d as m3d
    p0 = m3d.PositionVector([0, 1, 0])
    p1 = m3d.PositionVector([1, 0, 0])
    v0 = -1.0
    t_range = (-1.0, 1.0)
    pint = E3Interpolator(p0, p1, t_range=t_range, v0=v0)
    ts = np.linspace(*t_range, 100)
    ps = np.array([pint(t, checkrange=False).array for t in ts])
    ss = np.array([pint._s(t, checkrange=False) for t in ts])
    plt.scatter(*ps.T[:2])
    plt.scatter(*p0[:2], marker='o', s=100, facecolor='none', edgecolor='red')
    plt.scatter(*p1[:2], marker='o', s=100, facecolor='red', edgecolor='red')
    plt.show()
