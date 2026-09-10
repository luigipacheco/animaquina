# coding=utf-8

"""
Module for line class
"""

__author__ = "Morten Lind"
__copyright__ = "Morten Lind 2016-2021"
__credits__ = ["Morten Lind"]
__license__ = "LGPLv3"
__maintainer__ = "Morten Lind"
__email__ = "morten@lind.fairuse.org"
__status__ = "Development"


import math3d as m3d
import numpy as np


class Line:
    """A line class."""

    def __init__(self, *args, **kwargs):
        """Supported positional arguments:

        * Two arguments of type FreeVector and PositionVector, one of
          each in any order.

        Supported named arguments:

        * 'point_direction' or 'pd_pair': An ordered pair of vectors
          representing a point on the line and the line's
          direction. The direction does not have to be a unit vector.

        * 'point', 'direction': Separate vectors for point on line and
          direction of line in named arguments. The direction does not
          have to be a unit vector.

        * 'point0', 'point1': Two points defining the line, in named
         arguments.

        * 'points': A set of at least two points is used for
        PCA-identification of the direction of the line, and where the
        line point is chosen as the average position.

        The internal representation is point-direction.
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
            else:
                raise Exception(
                    self.__class__.__name__ +
                    ': Can only create on two positional arguments' +
                    ' of types PositionVector and FreeVector (in any order). ' +
                    f'Was given "{args}"')
        elif 'point_direction' in kwargs:
            p, d = kwargs['point_direction']
            self._p = m3d.PositionVector(p)
            self._d = m3d.FreeVector(d)
        elif 'pd_pair' in kwargs:
            p, d = kwargs['pd_pair']
            self._p = m3d.PositionVector(p)
            self._d = m3d.FreeVector(d)
        elif 'point' in kwargs and 'direction' in kwargs:
            self._p = m3d.PositionVector(kwargs['point'])
            self._d = m3d.FreeVector(kwargs['direction'])
        elif 'point0' in kwargs and 'point1' in kwargs:
            self._p = m3d.PositionVector(kwargs['point0'])
            self._d = m3d.PositionVector(kwargs['point1']) - self._p
        elif 'points' in kwargs:
            self.fit_points(kwargs['points'])
        else:
            raise Exception(
                self.__class__.__name__ +
                f'Can not create Line object on arguments "{args}" ' +
                f'and kw-arguments "{kwargs}"')
        # Create the unit direction vector
        self._ud = self._d.normalized

    def __repr__(self):
        return ('<Line: p=[{:.3f}, {:.3f}, {:.3f}] ' +
                'ud=[{:.3f}, {:.3f}, {:.3f}]>').format(
                    *tuple(self._p), *tuple(self._ud))

    @property
    def point(self):
        return self._p.copy()

    @property
    def direction(self):
        return self._d.copy()

    @property
    def unit_direction(self):
        return self._ud.copy()

    ud = unit_direction
    
    def __rmul__(self, trf: m3d.Transform):
        """Return a new line transformed to new coordinate reference as given
        by 'trf'.
        """
        return Line(trf * self._p, trf * self._d)

    def fit_points(self, points):
        """Compute the line from a set of points. 'points'
        must be an array of row position vectors, such that
        points[i] is a position vector."""
        points = np.array(points)
        centre = np.sum(points, axis=0)/len(points)
        eigen = np.linalg.eig(np.cov(points.T))
        max_ev_i = np.where(eigen[0] == max(eigen[0]))[0][0]
        direction = eigen[1].T[max_ev_i]
        self._p = m3d.PositionVector(centre)
        self._d = self._ud = m3d.FreeVector(direction)

    @classmethod
    def new_fitted_points(cls, points):
        return cls(points=points)

    def projected_point(self, p: m3d.PositionVector):
        """Return the projection of 'p' onto this line."""
        p2p = (self._p - p)
        return p + p2p - (self._ud * p2p) * self._ud

    def projected_line(self, line: 'Line'):
        """Return the projection of 'line' onto this line. I.e. return the
        point in this line, closest to 'line'. If the lines are
        parallel, the origin point of this line is returned. See
        https://en.wikipedia.org/wiki/Skew_lines#Nearest_Points.
        """
        # Check if the lines are parallel
        if (1 - self._ud * line._ud) < 10 * m3d.utils.eps:
            # Return any point on the line
            return self.point
        p1 = self._p
        p2 = line._p
        d1 = self._ud
        d2 = line._ud
        n2 = d2.cross(d1.cross(d2))
        return p1 + ((p2-p1) * n2 / (d1 * n2)) * d1

    def nearest_points(self, line):
        """Return the pair of closest points on this line and 'line'. The
        first of the returned points is on this line. See
        https://en.wikipedia.org/wiki/Skew_lines#Nearest_Points
        """
        # Check if the lines are parallel
        if (1 - self._ud * line._ud) < 10 * m3d.utils.eps:
            return self.point
        # https://en.wikipedia.org/wiki/Skew_lines#Nearest_Points
        p1 = self._p
        p2 = line._p
        d1 = self._ud
        d2 = line._ud
        n1 = d1.cross(d2.cross(d1))
        n2 = d2.cross(d1.cross(d2))
        return (p1 + ((p2-p1) * n2 / (d1 * n2)) * d1,
                p2 + ((p1-p2) * n1 / (d2 * n1)) * d2)


def _test():
    # Simple test of projected line.
    l1 = Line(point=(0, 1, 1), direction=(0, 0.1, 1))
    l2 = Line(point=(1, 1, 0.1), direction=(0, 1, 0))
    print(f'Line 1: {l1}')
    print(f'Line 2: {l2}')
    nps = l1.nearest_points(l2)
    print(f'Nearest points: {nps}')
    pl = l1.projected_line(l2)
    # print('Projected line: {}'.format(str(pl)))
    assert((pl-l1.projected_point(pl))
           .length < 10 * m3d.utils.eps)
    # Test line fitting.
    fl = Line.new_fitted_points([[1, 1, 1],
                                 [2, 2, 2],
                                 [3, 3, 3]])
    print('Fitted line: {}'.format(str(fl)))
    print(f'fl.point: {fl.point}')
    assert(fl.point == m3d.PositionVector(2, 2, 2))
    assert(1.0 - np.abs(fl.direction * m3d.FreeVector(1, 1, 1).normalized)
           < 10 * m3d.utils.eps)
