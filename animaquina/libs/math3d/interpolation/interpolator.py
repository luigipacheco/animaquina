# coding=utf-8

"""
"""

__author__ = "Morten Lind"
__copyright__ = "Morten Lind 2021"
__credits__ = ["Morten Lind"]
__license__ = "LGPLv3"
__maintainer__ = "Morten Lind"
__email__ = "morten@lind.fairuse.org"
__status__ = "Development"


from .. import utils


class Interpolator:
    """Base class for interpolators. Mainly does the time accounting and
    checking."""

    def __init__(self, t_range=None):
        self._t_range = t_range
        if t_range is not None:
            self._dur = self._t_range[1] - self._t_range[0]
        else:
            self._dur = None

    def _s(self, t, checkrange=True):
        """Calculate and return the path parameter given a path time."""
        t = utils.flt(t)
        if self._t_range is None:
            s = t
        else:
            s = (t - self._t_range[0]) / self._dur
        if checkrange:
            self._checkrange(s)
        return s

    def _checkrange(self, s):
        """Check if the given path parameter 's' is within the valid range
        [0.0 ; 1.0]. Raises exceptions if not.
        """
        if s < 0.0 or s > 1.0:
            raise self.Error('"t" must be number in [0.0 ; 1.0]. ' +
                             f'It was {s}')
