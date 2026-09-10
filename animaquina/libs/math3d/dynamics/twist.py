# coding=utf-8
"""
Module implementing the Twist class(es). A twist is generally a
spatial velocity of a rigid body. It has a linear and an angular part;
relating to e(3) and se(3), respectively. Different ways of
"""

__author__ = "Morten Lind"
__copyright__ = "Morten Lind 2013-2021"
__credits__ = ["Morten Lind"]
__license__ = "LGPLv3"
__maintainer__ = "Morten Lind"
__email__ = "morten@lind.fairuse.org"
__status__ = "Development"


import numpy as np
import math3d as m3d


class OrigoTwist:
    """An OrigoTwist instance is a representation of the motion of the
    defining coordinate system at its origo."""

    def __init__(self, *args, **kwargs):
        """Construct an OrigoTwist from arguments. 'kwargs' take
        precedence over given 'args'.

        'kwargs' may contain:

        * 'v_lin' and 'v_ang', each an iterable of three
          floats, either are optional.

        'args' may contain:

        * One iterable of six floats. The first three are taken as
          linear and the last three are taken as angular velocities.

        * Two iterables of three floats. The first is linear and the
          second angular velocities.

        * An OrigoTwist instance (copy constructor).
        """
        if len(args) == 0 and len(kwargs) == 0:
            # Default constructor
            self._v_lin = m3d.FreeVector()
            self._v_ang = m3d.FreeVector()
        elif 'v_lin' in kwargs or 'v_ang' in kwargs:
            self._v_lin = m3d.FreeVector(kwargs.get('v_lin', m3d.FreeVector()))
            self._v_ang = m3d.FreeVector(kwargs.get('v_ang', m3d.FreeVector()))
        elif len(args) == 1 and type(args[0]) == OrigoTwist:
            self._v_lin = args[0].linear
            self._v_ang = args[0].angular
        elif len(args) == 1 and len(args[0]) == 6:
            self._v_lin = m3d.FreeVector(args[0][:3])
            self._v_ang = m3d.FreeVector(args[0][3:])
        elif len(args) == 2 and len(args[0]) == 3 and len(args[1]) == 3:
            self._v_lin = m3d.FreeVector(args[0])
            self._v_ang = m3d.FreeVector(args[1])
        else:
            raise Exception(
                self.__class__.__name__ +
                'Could not construct on given arguments: *args=' +
                str(args) + ' *kwargs=' + str(kwargs))

    def equivalent(self, ref):
        """Compute the eqivalent twist at 'ref'. If 'ref' is a
        transform, compute the transformed equivalent twist at the
        origo of 'ref' and in the orientation of 'ref'. If 'ref' is a
        vector, compute the equivalent twist at 'ref' in the current
        coordinate system.  The new v_ang is the same as the old, but
        possibly reoriented. The new v_lin is the old one, possibly
        reoriented, plus the action of the v_ang acting at the old
        origo. Beware that the constant velocity motion obtained by
        the transformed twist is, at the new origo, only
        instantaneously in accord with the current twist, since it
        introduces a translation of the line of rotation.
        """
        if type(ref) == m3d.PositionVector:
            # 'ref' is given in current coordinates, and represents
            # the point at which the equivalent twist should be
            # found.
            va_n = self._v_ang
            vl_n = self._v_lin - ref.cross(va_n)
            return OrigoTwist(v_lin=vl_n, v_ang=va_n)
        elif type(ref) == m3d.Transform:
            # 'ref' is a transformation to the new coordinate system,
            # at the origo of whom the equivalent twist is sought, in
            # new coordinates.
            m = self._v_ang
            va_n = ref.orient * self._v_ang
            vl_n = ref.orient * self._v_lin + ref.pos.cross(va_n)
            return OrigoTwist(v_lin=vl_n, v_ang=va_n)

    def displacement(self, delta_t):
        """Compute the displacement resulting from applying the twist
        for time 'delta_t'. The returned transform will be given in the current
        coordinates and represent the moved coordinate system."""
        return m3d.Transform(delta_t * self.array)

    def __rmul__(self, left):
        """Handle a left-operator."""
        if np.isscalar(left):
            return OrigoTwist(left * self.array)
        elif type(left) in (m3d.Transform, m3d.PositionVector):
            return self.equivalent(left)
        elif type(left) == m3d.Orientation:
            # Perform a reorientation of the twist, as observed from a
            # coordinate system with the orientation given in 'left'
            vl_n = left * self._v_lin
            va_n = left * self._v_ang
            return OrigoTwist(v_lin=vl_n, v_ang=va_n)
        else:
            return NotImplemented

    # Angular property
    def get_angular(self):
        """Get the angular part."""
        return self._v_ang

    def set_angular(self, new_v_ang):
        """Set the angular part."""
        self._v_ang = m3d.FreeVector(new_v_ang)

    angular = ang = property(get_angular, set_angular)

    # Raw array data access
    def get_array(self):
        return np.append(self._v_lin._data, self._v_ang._data)

    array = property(get_array)

    # Linear property
    def get_linear(self):
        """Get the linear part."""
        return self._v_lin

    def set_linear(self, new_v_lin):
        """Set the linear part."""
        self._v_lin = m3d.FreeVector(new_v_lin)

    linear = lin = property(get_linear, set_linear)

    def __mul__(self, right):
        """Handle right operation."""
        if np.isscalar(right):
            return OrigoTwist(right * self.array)
        else:
            return NotImplemented

    def __add__(self, v_add):
        """Add two twists. Note that they are percieved as belonging
        to the same origo in the same coordinate system!.
        """
        return OrigoTwist(v_lin=self._v_lin+v_add._v_lin,
                           v_ang=self._v_ang+v_add._v_ang)

    def __sub__(self, v_sub):
        """Subtract two twists. Note that they are percieved as
        belonging to the same origo in the same coordinate system!.
        """
        return OrigoTwist(v_lin=self._v_lin-v_sub._v_lin,
                           v_ang=self._v_ang-v_sub._v_ang)

    def __neg__(self):
        """Return the negative twist."""
        return OrigoTwist(v_lin=-self._v_lin, v_ang=-self._v_ang.data)

    # def __copy__(self):
    #     """Copy method for creating a copy of this Vector."""
    #     return (self)

    # def __deepcopy__(self, memo):
    #     return self.__copy__()


    def __repr__(self):
        """String represenstation of the twist."""
        return ('<{} lin=[{:.3f}, {:.3f}, {:.3f}] ang=[{:.3f}, {:.3f}, {:.3f}]>'
                .format(*([self.__class__.__name__] +
                          self._v_lin.list + self._v_ang.list)))
