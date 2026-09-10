# coding=utf-8

"""
Module implementing the SE(3) interpolator class.
"""

__author__ = "Morten Lind"
__copyright__ = "Morten Lind 2009-2021"
__credits__ = ["Morten Lind"]
__license__ = "LGPLv3"
__maintainer__ = "Morten Lind"
__email__ = "morten@lind.fairuse.org"
__status__ = "Production"


import typing

from ..transform import Transform, PoseVector
from .. import utils
from .so3interpolator import SO3Interpolator
from .e3interpolator import E3Interpolator
from .interpolator import Interpolator


class SE3Interpolator(Interpolator):
    """A class for object representing a linear interpolation in task
    space, SE(3), between two points. Interpolation is done from one
    configuration, trf0, to another, trf1. trf0 and trf1 can be given
    as Transform objects."""

    class Error(Exception):
        """Exception class."""

        def __init__(self, message):
            self.message = 'SE3Interpolation Error: ' + message
            Exception.__init__(self, self.message)

        def __repr__(self):
            return self.message

    def __init__(self,
                 trf0: typing.Union[Transform, PoseVector],
                 trf1: typing.Union[Transform, PoseVector],
                 shortest: bool = True,
                 t_range: tuple[float, float] = None,
                 v0: float = None):
        """Initialise an SE(3) interpolation from transform 'trf0' to
        transform 'trf1'. If 'shortest' is True, the shortest rotation
        path is chosen, if False, the long rotation is used, and if
        None it is indeterminate, given by the Versor objects being
        constructed from the transforms.
        """
        Interpolator.__init__(self, t_range)
        self._trf0 = Transform(trf0)
        self._trf1 = Transform(trf1)
        self._so3i = SO3Interpolator(self._trf0.orient,
                                     self._trf1.orient,
                                     shortest=shortest,
                                     t_range=None)
        self._v0 = v0
        self._e3i = E3Interpolator(self._trf0.pos, self._trf1.pos,
                                   t_range=None,
                                   v0=None)

    def _s(self, t, checkrange=True):
        """Special calculus for path parameter possibly taking acceleration
        into account.
        """
        if self._v0 is None:
            return super()._s(t, checkrange)
        else:
            t_path = t - self._t_range[0]
            s = (self._v0 * t_path + 0.5 * self._acc *
                 t_path**2) / self._e3i._dlength
            if checkrange:
                super()._checkrange(s)
            return s

    def __call__(self, t, checkrange=True):
        """Class callable method for giving the transform at time
        't'; in [0,1] or in 't_range'."""
        # Use the R3 interpolator for handling the acceleration if any
        s = self._s(t, checkrange)
        return Transform(self._so3i.orient(s, checkrange=checkrange),
                         self._e3i.pos(s, checkrange=checkrange))


SE3Interpolation = SE3Interpolator
TaskLinearInterpolation = SE3Interpolator
EuclideanInterpolation = SE3Interpolator


def _test():
    """Simple test function."""
    global o0, o1, tint, p0, p1
    from math3d.orientation import Orientation
    from math3d.vector import Vector
    from math import pi
    p0 = Vector([0, 1, 0])
    p1 = Vector([1, 0, 1])
    o0 = Orientation()
    o0.set_to_x_rotation(pi / 2)
    o1 = Orientation()
    o1.set_to_z_rotation(pi / 2)
    tint = SE3Interpolation(Transform(o0, p0), Transform(o1, p1))
