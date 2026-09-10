# coding=utf-8
"""
Module implementing the Wrench class(es). A wrench is a spatial vector
composed of a force and a moment acting at a point. The two components
transform in a different way than two separate vectors.
"""

__author__ = "Morten Lind"
__copyright__ = "Morten Lind 2013"
__credits__ = ["Morten Lind"]
__license__ = "LGPLv3"
__maintainer__ = "Morten Lind"
__email__ = "morten@lind.fairuse.org"
__status__ = "Production"


import numpy as np
import math3d as m3d

class OrigoWrench(object):
    """An OrigoWrench is a wrench, i.e. a force and a moment vector,
    which acts around the origo of the defining coordinate
    system. When transformed, it changes to the origo of the target
    coordinate system.
    """

    def __init__(self, *args, **kwargs):
        """Construct an OrigoWrench from arguments. 'kwargs' take
        precedence over given 'args'.

        'kwargs' may contain:

        * 'f' and 'm', either are optional, each an iterable of three
          floats.

        'args' may contain:

        * One iterable of six floats. The first three are taken as the
          force, and the last three as the moment.

        * Two iterables of three floats. The first gives the force and
          the second the moment.

        * One OrigoTwist instance (copy constructor).
        """
        if len(args) == 0 and len(kwargs) == 0:
            # Default constructor
            self._force = m3d.Vector()
            self._moment = m3d.Vector()
        elif 'f' in kwargs or 'm' in kwargs:
            self._force = m3d.Vector(kwargs.get('f', m3d.Vector()))
            self._moment = m3d.Vector(kwargs.get('m', m3d.Vector()))
        elif len(args) == 1 and len(args[0]) == 6:
            self._force = m3d.Vector(args[0][:3])
            self._moment = m3d.Vector(args[0][3:])
        elif len(args) == 1 and type(args[0]) == OrigoWrench:
            self._force = args[0].force
            self._moment = args[0].moment
        elif len(args) == 2 and len(args[0]) == 3 and len(args[1]) == 3:
            self._force = m3d.Vector(args[0])
            self._moment = m3d.Vector(args[1])
        else:
            raise Exception(
                self.__class__.__name__ +
                'Could not construct on given arguments: *args=' +
                str(args) + ' *kwargs=' + str(kwargs))

    def get_array(self):
        """Return a simple 6-array holding the values of the force and moment
        appended, in that order.
        """
        return np.append(self._force.array, self._moment.array)

    def set_array(self, array):
        """Set the wrench by six values in the iterable 'array'. The first
        three values must specify force, the last three must specify
        moment
        """
        if len(array) != 6:
            raise Exception(
                self.__class__.__name__ +
                'Setting the value by the "array" property needs exactly'
                + ' six values.({} were given)'.format(len(array)))
        self._force = m3d.Vector(array[:3])
        self._moment = m3d.Vector(array[3:])

    array = property(get_array, set_array)

    def equivalent(self, ref):
        """Compute the eqivalent wrench at 'ref'. If 'ref' is a
        transform, compute the transformed equivalent wrench at the
        origo of 'ref' and in the orientation of 'ref'. If 'ref' is a
        vector, compute the equivalent wrench at 'ref' in the current
        coordinate system.  The new force is the same as the old, but
        possibly reoriented. The new moment is the old one, possibly
        reoriented, plus the action of the force acting at the old
        origo.
        """
        if type(ref) == m3d.Vector:
            # 'ref' is given in current coordinates, and represents
            # the point at which the equivalent wrench should be
            # found.
            f_n = self._force
            m_n = self._moment - ref.cross(f_n)
            return OrigoWrench(f=f_n, m=m_n)
        elif type(ref) == m3d.Transform:
            # 'ref' is a transformation to the new coordinate system,
            # at the origo of whom the equivalent wrench is sought, in
            # new coordinates.
            m = self._moment
            f_n = ref.orient * self._force
            m_n = ref.orient * self._moment + ref.pos.cross(f_n)
            return OrigoWrench(f=f_n, m=m_n)

    def __rmul__(self, left):
        """Handle left operator."""
        if type(left) in [m3d.Transform, m3d.Vector]:
            return self.equivalent(left)
        if type(left) == m3d.Orientation:
            # Perform a reorientation of the wrench, as observed from
            # a coordinate system with the orientation transform given
            # in 'left'
            f_n = left * self._force
            m_n = left * self._moment
            return OrigoWrench(f=f_n, m=m_n)

    # Moment property
    def get_moment(self):
        """Get the moment part."""
        return self._moment.copy()
    def set_moment(self, new_moment):
        """Set the moment part."""
        self._moment = m3d.Vector(new_moment)
    moment = property(get_moment, set_moment)

    # Force property
    def get_force(self):
        """Get the force part."""
        return self._force.copy()
    def set_force(self, new_force):
        """Set the force part."""
        self._force = m3d.Vector(new_force)
    force = property(get_force, set_force)

    def __add__(self, w_add):
        """Add two wrenches. Note that they are percieved as belonging
        to the same origo in the same coordinate system!.
        """
        return OrigoWrench(f=self._force+w_add._force,
                           m=self._moment+w_add._moment)

    def __sub__(self, w_sub):
        """Subtract two wrenches. Note that they are percieved as
        belonging to the same origo in the same coordinate system!.
        """
        return OrigoWrench(f=self._force-w_sub._force,
                           m=self._moment-w_sub._moment)

    def __neg__(self):
        """Return the negative wrench."""
        return OrigoWrench(f=-self._force, m=-self._moment.data)

    def __repr__(self):
        """String represenstation of the wrench."""
        return ('<{} f=[{:.3f}, {:.3f}, {:.3f}] m=[{:.3f}, {:.3f}, {:.3f}]>'
                .format(*([self.__class__.__name__] +
                          self._force.list + self._moment.list)))


class FootedWrench(object):
    """A FootedWrench is a wrench that contains, in addition to the
    force-moment vectors, a position vector, the 'foot point', for
    holding the position of action of the wrench. Under coordinate
    changes, the force and moment vectors then transforms as free
    vectors and the foot point transforms as a position vector."""
    def __init__(self, *args, **kwargs):
        raise NotImplemented
