# coding=utf-8

"""
Module for line segment class
"""

__author__ = "Morten Lind"
__copyright__ = "Morten Lind 2021"
__credits__ = ["Morten Lind"]
__license__ = "LGPLv3"
__maintainer__ = "Morten Lind"
__email__ = "morten@lind.fairuse.org"
__status__ = "Development"


import typing

import math3d as m3d
import numpy as np

from . import Line


class LineSegment(Line):
    """A line class."""

    def __init__(self, *args, **kwargs):
        """Supported positional arguments:

        * Two arguments of type FreeVector and PositionVector, one of
          each in any order. The position is perceived as the start
          point, and the free vector is perceived as the displacement
          from start to end.

        Supported named arguments:

        * 'point_displacement' or 'pd_pair': An ordered pair of
          vectors representing a point on the line and the segments
          displacement from start to end point.

        * 'point', 'displacement': Separate vectors for point on line and
          direction of line in named arguments. The direction does not
          have to be a unit vector.

        * 'start', 'end': Two points defining the start and end of the
          line segment.

        * 'points': A set of at least two points is used for
        PCA-identification of the direction of the line, and where the
        line segment start and end points are chosen at the ultimate
        projections of the point cloud on the line.

        """
        if len(args) not in [0, 2]:
            raise Exception(
                self.__class__.__name__ +
                ': Can only create on two positional arguments ' +
                'as of types PositionVector and FreeVector!' +
                f' Was given "{args}"')
        elif len(args) == 2:
            # Infer on types Position- and FreeVector
            if(type(args[0]) == m3d.PositionVector
               and type(args[1]) == m3d.FreeVector):
                self._p = args[0].copy()
                self._d = args[1].copy()
            elif (type(args[1]) == m3d.PositionVector
                  and type(args[0]) == m3d.FreeVector):
                self._p = args[1].copy()
                self._d = args[0].copy()
            elif (type(args[0]) == m3d.PositionVector
                  and type(args[1]) == m3d.PositionVector):
                self._p = args[0]
                self._d = args[1] - self._p
            else:
                raise Exception(
                    self.__class__.__name__ +
                    ': Can only create on two positional arguments' +
                    ' of types PositionVector and FreeVector (in any order). ' +
                    f'Was given "{args}"')
        elif 'point_direction' in kwargs:
            p, d = kwargs['point_displacement']
            self._p = m3d.PositionVector(p)
            self._d = m3d.FreeVector(d)
        elif 'pd_pair' in kwargs:
            p, d = kwargs['pd_pair']
            self._p = m3d.PositionVector(p)
            self._d = m3d.FreeVector(d)
        elif 'point' in kwargs and 'displacement' in kwargs:
            self._p = m3d.PositionVector(kwargs['point'])
            self._d = m3d.FreeVector(kwargs['displacement'])
        elif 'start' in kwargs and 'end' in kwargs:
            self._p = m3d.PositionVector(kwargs['start'])
            self._d = m3d.PositionVector(kwargs['end']) - self._p
        elif 'points' in kwargs:
            self.fit_points(kwargs['points'])
        else:
            raise Exception(
                self.__class__.__name__ +
                f'Can not create Line object on arguments "{args}" ' +
                f'and kw-arguments "{kwargs}"')
        # Create the unit direction vector
        self._ud = self._d.normalized
        self.length = self._d.length

    def __repr__(self):
        return ('<Line: p=[{:.3f}, {:.3f}, {:.3f}] ' +
                'ud=[{:.3f}, {:.3f}, {:.3f}]>').format(
                    *tuple(self._p), *tuple(self._ud))

    @property
    def start(self):
        return self._p.copy()

    @property
    def end(self):
        return self._p + self._d

    @property
    def point(self):
        return self._p.copy()

    @property
    def displacement(self):
        return self._d.copy()

    @property
    def unit_direction(self):
        return self._ud.copy()

    def __rmul__(self, trf: m3d.Transform):
        """Return a new line transformed to new coordinate reference as given
        by 'trf'.
        """
        return Line(trf * self._p, trf * self._d)

    def fit_points(self, points: np.ndarray):
        """Compute the line segment from a set of points. 'points' must be an
        array of row position vectors, Nx3, such that points[i] is a
        position vector.
        """
        # Fit a line to points
        line = Line.new_fitted_points(points)
        # Project points
        projs = (points - line.point.array) @ (line.unit_direction.array)
        # Minimum and maximum projections give start and end points
        start = line.point + projs.min() * line.unit_direction
        end = line.point + projs.max() * line.unit_direction
        self._p = start
        self._d = end - start

        
    @classmethod
    def new_fitted_points(cls, points: np.ndarray):
        """Create a new line segment from a set of points. 'points' must be an
        array of row position vectors, Nx3, such that points[i] is a
        position vector.
        """
        return cls(points=points)

    def projected_point(self, p: m3d.PositionVector):
        """Return the point on the line segment closest to the given position
        vector.
        """
        s = (self.unit_direction * (p - self._p))
        if s <= 0:
            return self.start
        elif s >= self.length:
            return self.end
        else:
            return self._p + s * self._ud

    def dist(self, p: m3d.PositionVector):
        return p.dist(self.projected_point(p))


def _test():
    # Simple test of projected line.
    ls1 = LineSegment(start=(0, 1, 1), end=(0, 0.1, 1))
    ls2 = LineSegment(point=(1, 1, 0.1), displacement=(0, 1, 0))
    # p = ls1.projected_line(ls2)
    # # print('Projected line: {}'.format(str(pl)))
    # assert((pl-l1.projected_point(pl))
    #        .length < 10 * m3d.utils.eps)
    # Test line fitting.
    points = np.array([[1, 1, 2],
                       [2, 2, 3],
                       [3, 3, 4]])
    fls = LineSegment.new_fitted_points(points)
    print('Fitted line: {}'.format(str(fls)))
    print(f'fls.start: {fls.point} , fls.end: {fls.end}')
    assert(fls.start == m3d.PositionVector(1, 1, 2))
    assert(fls.end == m3d.PositionVector(3, 3, 4))
    assert(1.0 - np.abs(fls.displacement * m3d.FreeVector(1, 1, 1).normalized)
           < 10 * m3d.utils.eps)
