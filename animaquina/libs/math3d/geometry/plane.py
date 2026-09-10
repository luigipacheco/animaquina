# coding=utf-8

"""
Module for Plane class.
"""

__author__ = "Morten Lind"
__copyright__ = "Morten Lind 2013-2021"
__credits__ = ["Morten Lind"]
__license__ = "LGPLv3"
__maintainer__ = "Morten Lind"
__email__ = "morten@lind.fairuse.org"
__status__ = "Development"


import math3d as m3d
import numpy as np

from .. import utils


class Plane:
    def __init__(self, *args, **kwargs):
        """Create a plane representation by one of the following named
        arguments:
        
        * Two arguments of type FreeVector (normal) and PositionVector
          (point).

        * 'plane_vector': A normalized plane vector. The normal will
        be pointing away from the origo. If kw-argument 'origo_inside'
        is given, this will determine the direction of the plane
        normal; otherwise origo will be set inside.

        * 'pn_pair': An ordered sequence for creating a reference
        point and a normal vector. The normal and point are conserved
        as they are given.

        * 'points': A set of at least three points for fitting a
        plane.

        * 'coeffs': Four coefficients (a,b,c,d) for the plane equation
          ax+by+cz+d=0.

        The internal representation is point and normal. If given as a
        pn_pair, A boolean, 'origo_inside', is held to decide the
        direction of the normal vector, such that the origo of the
        defining coordinate system is on the inside when true.
        """

        if len(args) == 2:
            if(isinstance(args[0], m3d.PositionVector)
               and isinstance(args[1], m3d.FreeVector)):
                self._p = m3d.PositionVector(args[0])
                self._n = m3d.FreeVector(args[1]).normalized
            elif(isinstance(args[1], m3d.PositionVector)
                 and isinstance(args[0], m3d.FreeVector)):
                self._p = m3d.PositionVector(args[1])
                self._n = m3d.FreeVector(args[0]).normalized
            else:
                raise Exception(
                    'Plane.__init__: When give positional arguments, exactly ' +
                    ' two must be given and types must be one FreeVector and ' +
                    f'one PositionVector! Was given {args}')
        elif len(args) != 0:
            raise Exception(
                'Plane.__init__: When give positional arguments, exactly ' +
                ' two must be given and types must be one FreeVector and ' +
                f'one PositionVector! Was given {args}')
        else:
            self._origo_inside = kwargs.get('origo_inside', True)
            if 'plane_vector' in kwargs:
                pv = m3d.Vector(kwargs['plane_vector'])
                (self._p, self._n) = self.pv_to_pn(pv)
            elif 'pn_pair' in kwargs:
                pn_pair = kwargs['pn_pair']
                self._p = m3d.PositionVector(pn_pair[0])
                self._n = m3d.FreeVector(pn_pair[1]).normalized
                # Override a given origo inside.
                self._origo_inside = (self._p * self._n) > 0
                # Make point a 'minimal' point on the plane, i.e. the
                # projection of origo in the plane.
                self._p = m3d.PositionVector((self._p * self._n) * self._n)
            elif 'points' in kwargs:
                self.fit_points(kwargs['points'])
            elif 'coeffs' in kwargs:
                self.coeffs = kwargs['coeffs']
            else:
                raise Exception(
                    'Plane.__init__ : Must have either of constructor ' +
                    'kw-arguments: "plane_vector", "pn_pair", or ' +
                    '"points". Neither given!')

    def copy(self):
        return Plane(pn_pair=(self._p, self._n))

    def __repr__(self):
        return ('<Plane: p=[{:.3f}, {:.3f}, {:.3f}] ' +
                'un=[{:.3f}, {:.3f}, {:.3f}]>').format(
                    *tuple(self._p), *tuple(self._n))

    # def __repr__(self):
    #     return '<Plane: [{:.5f}, {:.5f}, {:.5f}]>'.format(
    #         *tuple(self.plane_vector.array))

    def __rmul__(self, transf):
        """Support transformation of this plane to another coordinate
        system by multiplication of an m3d.Transform from left."""
        if type(transf) != m3d.Transform:
            return NotImplemented
        tnormal = transf * self._n
        tpoint = transf * self._p
        return Plane(pn_pair=(tpoint, tnormal))

    def dist(self, p: m3d.PositionVector):
        """Signed distance to a point, measured positive along the
        normal vector direction."""
        return (m3d.PositionVector(p) - self._p) * self._n

    def get_plane_vector(self):
        return self.pn_to_pv(self._p, self._n)

    def set_plane_vector(self, pv):
        (self._p, self._n) = self.pv_to_pn(pv)

    plane_vector = property(get_plane_vector, set_plane_vector)

    @property
    def point_normal(self):
        return (self._p, self._n)

    @property
    def point(self):
        return self._p

    @property
    def normal(self):
        return self._n

    @property
    def unit_normal(self):
        return self._n.normalized

    def get_coeffs(self):
        """Return the four coefficients of the plane."""
        return list(self._n) + [self.dist([0, 0, 0])]

    def set_coeffs(self, coeffs):
        """Set the plane to the one given by the four coefficients."""
        self.plane_vector = m3d.Vector(coeffs[:3]) / -coeffs[3]
        # if not len(coeffs) == 4:
        #     raise Exception('Plane needs four coefficients!')
        # self._n = m3d.Vector(coeffs[:3]).normalized
        # self._p = -coeffs[3] * self._n

    coeffs = property(get_coeffs, set_coeffs)

    def fit_points(self, points):
        """Compute the plane vector from a set of points. 'points'
        must be an array of row position vectors, such that
        points[i] is a position vector."""
        points = np.array(points)
        centre = np.sum(points, axis=0)/len(points)
        eigen = np.linalg.eig(np.cov(points.T))
        min_ev_i = np.where(eigen[0] == min(eigen[0]))[0][0]
        normal = eigen[1].T[min_ev_i]
        (self._p, self._n) = (m3d.PositionVector(centre), m3d.FreeVector(normal))

    @classmethod
    def new_fitted_points(cls, points):
        return cls(points=points)

    def fit_plane(self, points):
        print('Deprecation warning: fit_plane -> fit_points')
        self.fit_points(points)

    @classmethod
    def pn_to_pv(cls, point, normal):
        """Compute the plane vector of a plane represented by a point
        and normal."""
        if not isinstance(point, m3d.PositionVector):
            point = m3d.PositionVector(point)
        if not isinstance(normal, m3d.FreeVector):
            normal = m3d.FreeVector(normal)
        un = normal.normalized
        # Origo projection on plane
        p0 = (point * un) * un
        # Square of offset from origo
        d2 = p0.length_squared
        # return the plane vector
        return m3d.Vector(p0 / d2)

    @classmethod
    def pv_to_pn(cls, pv):
        """Calculate a point-normal representation of the plane
        described by the given plane vector."""
        if isinstance(pv, m3d.Vector):
            pv = m3d.Vector(pv)
        d = pv.length
        n = m3d.FreeVector(pv / d)
        p = m3d.PositionVector(n / d)
        return (p, n)

    def projection(self, point):
        """Return the projection of the 'point' on the plane."""
        if isinstance(point, m3d.PositionVector):
            point = m3d.PositionVector(point)
        return point - self._n * (point - self._p) * self._n

    def line_intersection(self, line):
        """Compute the intersection with the given 'line'. If the line is
        parallel to the plane, None is returned, irregardless of
        whether the line is lying in the plane. See
        http://geomalgorithms.com/a05-_intersect-1.html. In-line
        comments use terminology from
        https://en.wikipedia.org/wiki/Skew_lines#Nearest_Points.
        """
        if type(line) != m3d.geometry.Line:
            raise Exception(
                'Method only implemented for math3d.geometry.Line object')
        p0 = line._p  # p1
        u = line._ud  # d1
        n = self._n  # d2 x (d1 x d2) = n2
        v0 = self._p  # p2
        w = p0 - v0  # p1 - p2
        nu = n * u
        if np.abs(nu) < 10 * utils.eps:
            # The line is parallel to the plane
            return None
        else:
            si = (n * -w) / nu  # (p2 - p1) * n2 / (d1 * n2)
            return p0 + si * u  # p1 + si * d1

    def plane_intersection(self, other):
        """Find the line of intersection with 'other' plane. Method found in
        http://paulbourke.net/geometry/pointlineplane/
        """
        if not isinstance(other, Plane):
            raise Exception(
                'Method only implemented for math3d.geometry.Plane object')
        ld = self._n.cross(other._n)
        if ld.length < utils.eps:
            return None
        ld.normalize()
        ndot = self._n * other._n
        det = 1 - ndot ** 2
        ds = -self.coeffs[3]
        do = -other.coeffs[3]
        cs = (ds - do * ndot) / det
        co = (do - ds * ndot) / det
        lp = cs * self._n + co * other._n
        return m3d.geometry.Line(point_direction=(lp, ld))

    def intersection(self, other):
        """Polymorphic intersection method."""
        if isinstance(other, Plane):
            return self.plane_intersection(other)
        elif isinstance(other, m3d.geometry.Line):
            return self.line_intersection(other)
        else:
            raise NotImplementedError('Can not compute intersection with ' +
                                      'object of type {}'.format(type(other)))


def _test():
    # Test creation on points
    pln = Plane(points=((1, 0, 0), (0, 1, 0), (0, 0, 1)))
    assert (np.abs(pln.normal * m3d.Vector(1, 1, 1).normalized) - 1
            < utils.eps)
    pln0 = Plane(plane_vector=(1, 0, 0))
    pln1 = Plane(plane_vector=(0, 1, 0))
    # Test plane-plane intersection
    line = pln0.intersection(pln1)
    assert(line.point.x == 1 and line.point.y == 1)
    assert(np.abs(line.direction * m3d.Vector.ez) == 1)
    # Test for intersection with unsupported object
    try:
        pln0.intersection(m3d.Vector.ex)
    except NotImplementedError as nie:
        print('Caught expected exception from intersection of plane ' +
              'with vector. "{}"'.format(str(nie)))
    from .line  import Line
    # Test line-plane intersection
    line = Line(pd_pair=((0, 0, 1), (0, 1, -0.01)))
    plane = Plane(pn_pair=((0, 0, 0), (0, 0, 2)))
    lpi = plane.intersection(line)
    assert(lpi.x == 0 and lpi.y == 100 and lpi.z == 0)
