# coding=utf-8

"""
Module implementing the Quaternion class.
"""

__author__ = "Morten Lind"
__copyright__ = "Morten Lind 2012-2021"
__credits__ = ["Morten Lind"]
__license__ = "LGPLv3"
__maintainer__ = "Morten Lind"
__email__ = "morten@lind.fairuse.org"
__status__ = "Production"

import numpy as np


from . import utils
from .vector import FreeVector, Vector
from .orientation import Orientation, RotationVector


class Quaternion(object):

    class Error(Exception):
        """Exception class."""

        def __init__(self, message):
            self.message = message
            Exception.__init__(self, self.message)

        def __repr__(self):
            return self.message

    def __init__(self, *args, **kwargs):
        if len(args) == 0:
            self._s = 1
            self._v = FreeVector(0, 0, 0)
        if len(args) == 1:
            if type(args[0]) in [Quaternion]:
                self._s = args[0]._s
                self._v = args[0]._v.copy()
        elif len(args) == 2:
            if(utils.is_num_type(args[0]) and
               type(args[1]) in (Vector, FreeVector)):
                # Interpret as s, v
                self._s = utils.flt(args[0])
                self._v = FreeVector(args[1])
        elif len(args) == 4:
            self._s = args[0]
            self._v = FreeVector(args[1:])

    def __getattr__(self, name):
        if name == 's':
            return self._s
        elif name == 'x':
            return self._v.x
        elif name == 'y':
            return self._v.y
        elif name == 'z':
            return self._v.z
        else:
            raise AttributeError(('Attribute "{}" not found in class "{}".')
                                 .format(name, self.__class__.__name__))

    def __setattr__(self, name, val):
        if name in ['s', 'x', 'y', 'z']:
            raise AttributeError(('Not allowed to set attribute "{}" directly in ' +
                                  'class "{}"').format(name, self.__class__.__name__))
        else:
            object.__setattr__(self, name, val)

    def __getitem__(self, index):
        if index == 0:
            return self._s
        else:
            return self._v[index-1]

    def __repr__(self):
        return '<{}: [{:.5f}, ({:.5f}, {:.5f}, {:.5f})]>'.format(
            self.__class__.__name__, self._s, *self._v._data)

    def __copy__(self):
        """Copy method for creating a copy of this Quaternion."""
        return self.__class__(self)

    def __deepcopy__(self, memo):
        return self.__copy__()

    def copy(self, other=None):
        """Copy data from 'other' to self. If no argument given,
        i.e. 'other==None', return a copy of this Quaternion."""
        if other is None:
            return self.__class__(self)
        else:
            self._s = other._s
            self._v = other._v.copy()

    def __eq__(self, other):
        return np.isclose(self._s, other._s) and self._v == other._v

    def get_vector_part(self):
        """Return a copy of the vector part of the versor."""
        return self._v.copy()

    vector_part = property(get_vector_part)

    def get_scalar_part(self):
        """Return the scalar part of the versor."""
        return self._s

    scalar_part = property(get_scalar_part)

    def __mul__(self, other):
        """Multiplication of this with an 'other' Quaternion or Versor, or
        with a scalar.
        """
        if isinstance(other, Quaternion):
            return self.__class__(self._s*other._s - self._v*other._v,
                                  self._v.cross(other._v)
                                  + self._s*other._v + other._s*self._v)
        elif utils.is_num_type(other):
            return Quaternion(other*self._s, other*self._v)
        else:
            return NotImplemented

    def __rmul__(self, rother):
        # """Right-multiply by Versor or scalar. """
        if utils.is_num_type(rother):
            return self.__class__(rother * self._s, rother * self._v)
        else:
            return NotImplemented

    def __imul__(self, other):
        """In-place multiply."""
        new_quat = self * other
        self._s = new_quat._s
        self._v = new_quat._v
        return self

    def __ipow__(self, x):
        """In-place exponentiation of this versor to the power of
        'x'."""
        # if abs(1 - abs(self._s)) < utils.sqrt_eps:
        #     self._s = utils.flt(1)
        #     self._v = FreeVector(0, 0, 0)
        # else:
        #     theta = np.arccos(self._s)
        #     sintheta = np.sin(theta)
        #     logv = theta / sintheta * self._v
        #     alpha = x * logv.length
        #     v = logv.normalized
        #     self._s = np.cos(alpha)
        #     self._v = np.sin(alpha) * v
        theta = x * np.arccos(self._s/self.norm)
        if np.isclose(self._v.length, 0):
            n = FreeVector(0, 0, 0)
        else:
            n = self._v.normalized
        norm = self.norm ** x
        self._s = norm * np.cos(theta)
        self._v = norm * n * np.sin(theta)
        return self

    def __pow__(self, x):
        """Return this quaternion to the power of 'x'."""
        q = self.copy()
        q **= x
        return q

    def negate(self):
        """In-place negation, i.e. flip between small- and large-norm
        representation of the same rotation.
        """
        self._s = -self._s
        self._v = -self._v

    def __neg__(self):
        """Return the negative of this versor."""
        q = self.__class__(self)
        q.negate()
        return q

    def get_norm(self):
        """Return the usual quaternion norm."""
        return np.sqrt(self.norm_squared)

    norm = property(get_norm)

    def get_norm_squared(self):
        """Return the square of the usual quaternion norm."""
        return self._s**2 + self._v.length_squared

    norm_squared = property(get_norm_squared)

    def conjugate(self):
        """In-place conjugation of this quaternion."""
        self._v = -self._v

    def get_conjugated(self):
        """Return a quaternion which is the conjugated of this
        quaternion.
        """
        qc = self.copy()
        qc.conjugate()
        return qc

    conjugated = property(get_conjugated)

    def normalize(self):
        """Normalize this versor. """
        n = self.norm
        if abs(n) < 1e-10:
            self._s = utils.flt(1)
            self._v = FreeVector(0.0, 0.0, 0.0)
        else:
            ninv = 1.0 / n
            self._s *= ninv
            self._v *= ninv

    def get_normalized(self):
        """Return a normalised version of this quaternion. """
        q = self.copy()
        q.normalize()
        return q

    normalized = property(get_normalized)

    def invert(self):
        """In-place inversion of this quaternion. """
        # n2 = self.norm_squared
        self.conjugate()
        self *= 1 / self.norm_squared
        # self *= 1 / n2

    def get_inverse(self):
        """Return an inverse of this quaternion."""
        qi = self.copy()
        qi.invert()
        return qi

    inverse = property(get_inverse)

    def get_array(self):
        """Return an ndarray with the fundamental data. The layout is as
        described by the list property.
        """
        return np.array(self.list)

    array = property(get_array)

    def get_list(self):
        """Return the fundamental data as a list. The scalar part is placed in
        the first element, at index 0, and the vector data at the
        remainder, slice [1:].
        """
        return [self._s]+self._v.list

    list = property(get_list)

    def get_matrix(self):
        """Return a 4x4 matrix representation of the Quaternion. See
        http://en.wikipedia.org/wiki/Quaternion#Matrix_representations.
        """
        a, b, c, d = self._s, self._v.x, self._v.y, self._v.z
        return np.array([[a, b, c, d],
                         [-b, a, -d, c],
                         [-c, d, a, -b],
                         [-d, -c, b, a]])

    matrix = property(get_matrix)


class Versor(Quaternion):
    """Versor class for representing unit quaternions."""

    class Error(Exception):
        """Exception class."""

        def __init__(self, message):
            self.message = message
            Exception.__init__(self, self.message)

        def __repr__(self):
            return self.message

    def __init__(self, *args, **kwargs):
        """Create a versor. Args may be () for default
        constructor; (Orientation) for createing a quaternion
        representing the given orientation; (Versor) for a copy
        constructor, (s,x,y,z) or (s,FreeVector) for the direct versor
        data; (FreeVector) for creating the equivalent to a rotation
        vector; or (FreeVector, angle) for creating the equivalent of axis
        angle. A named option 'norm_warn' is supported as a kwargs and
        defaults to True. If set to false, nomalization is performed
        tacitly.
        """
        norm_warn = kwargs.get('norm_warn', True)
        self._s = utils.flt(0.0)
        self._v = FreeVector(1, 0, 0)
        if len(args) == 0:
            # Default constructor
            pass
        elif len(args) == 1:
            # Try with orientation or versor
            if type(args[0]) == Orientation:
                self.orientation = args[0]
            # Try with rotation vector
            elif type(args[0]) in (Vector, FreeVector, RotationVector):
                self.log = args[0]
            # Copy constructor
            elif type(args[0]) == Versor:
                self._s = args[0]._s
                self._v = FreeVector(args[0])
            elif type(args[0]) in (list, tuple, np.ndarray):
                raise utils.Error(
                    'A Versor can not be constructed on a list, ' +
                    'a tuple, or an np.ndarray. Was given "{}".'
                    .format(args[0]))
            else:
                raise utils.Error(
                    'Unknown argument given for Versor constructor: ' +
                    '"{}".'.format(args[0]))
        elif len(args) == 2:
            # Test for (axis, angle) and (s, v) determined by order
            if(utils.is_num_type(args[0]) and
               type(args[1]) in (Vector, FreeVector)):
                # Interpret as s, v
                self._s = utils.flt(args[0])
                self._v = FreeVector(args[1])
            elif (utils.is_num_type(args[1]) and
                  type(args[0]) in (Vector, FreeVector)):
                # Interpret as axis-angle
                axis = FreeVector(args[0])
                ang = args[1]
                self.axis_angle = (axis, ang)
            else:
                raise utils.Error(
                    'Unknown arguments given for Versor constructor' +
                    ': "{}".'.format(args[0]))
        elif len(args) == 3 and np.all(np.isreal(args)):
            # Assume three components of a rotation vector
            self.rotation_vector = args
        elif len(args) == 4 and np.all(np.isreal(args)):
            # Assume numbers for s, x, y, and z
            self._s = utils.flt(args[0])
            self._v = FreeVector(args[1:])
        else:
            raise utils.Error(
                'Creating on type {} is not supported'
                .format(str(type(args))))
        err = np.abs(self.norm - 1.0)
        if err > utils.eps:
            if norm_warn and err > 1.0e-4:
                print(('Versor.__init__ : Warning : Arguments '
                       'did not constitute a versor ' +
                       '(error={:.2e}). Normalizing!')
                      .format(self.norm-1))
            self.normalize()

    def __mul__(self, other):
        """Multiplication is interpreted by either transforming
        (rotating) a Vector, ordinary versor multiplication, or
        multiplication by scalar."""
        if type(other) in (Vector, FreeVector):
            # Do a rotation of the vector
            return (self * Versor(0, other) * self.inverse)._v.copy()
        elif type(other) == Versor:
            # Ordinary quaternion multiplication
            return Quaternion.__mul__(self, other)
            # return Versor(self._s * other._s - self._v * other._v,
            #               self._v.cross(other._v) +
            #               self._s * other._v + other._s * self._v)
        elif type(other) == Orientation:
            return self * other.versor
        # elif utils.is_num_type(other):
        #     return Versor(other * self._s, other * self._v)
        else:
            return NotImplemented

    def __rmul__(self, rother):
        # """Right-multiply by number. """
        # if utils.is_num_type(rother):
        #     return Versor(rother * self._s, rother * self._v)
        # else:
        return NotImplemented

    def __imul__(self, other):
        """In-place multiply."""
        if other in (Orientation, Versor):
            new_versor = self * other
            self._s = new_versor._s
            self._v = new_versor._v
        else:
            return NotImplemented
        return self

    # Inversion is a simple conjugation
    invert = Quaternion.conjugate

    def get_ang_norm(self, shortest=True):
        """Return the angular norm, i.e. the angular rotation, of this
        versor. If 'shortest' is True, the default, the shortest
        norm is returned, i.e. the minimal geodesic path length
        from the unit element to this unit versor.
        """
        if shortest is None:
            return 2*np.arccos(self._s)
        else:
            if shortest:
                # Return the long rotation angle
                return 2*min(np.arccos(-self._s), np.arccos(self._s))
            else:
                # Return the long rotation angle
                return 2*max(np.arccos(-self._s), np.arccos(self._s))

    ang_norm = property(get_ang_norm)

    def ang_dist(self, other, shortest=True):
        """Compute the rotation angle distance to the 'other' versor. If
        'shortest' is True, the default, the shortest distance is
        returned, i.e. the minimal geodesic path length from the
        'other' versor to this versor. 'shortest'
        may be False, in which case the longest angular distance is
        returned. If 'shortest' is None, the natural angular distance
        is computed.
        """
        return (self.conjugated * other).get_ang_norm(shortest)

    def dist_squared(self, other):
        """Compute the square of the usual quaternion metric distance to the
        'other' versor.
        """
        return (self._s - other._s)**2 + (self._v - other._v).length_squared

    def dist(self, other):
        """Compute the usual quaternion metric distance to the
        'other' quaternion."""
        return np.sqrt(self.dist_squared(other))

    def get_axis_angle(self):
        """Return an '(axis, angle)' pair representing the orientation of this
        versor. References:
        https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation#Recovering_the_axis-angle_representation
        https://en.wikipedia.org/wiki/Axis%E2%80%93angle_representation#Unit_quaternions
        """
        vlen = self._v.length
        if np.isclose(vlen, 0.0):
            return (FreeVector(), 0.0)
        else:
            angle = 2 * np.arctan2(vlen, self._s)
            return (self._v/vlen, angle)

    def set_axis_angle(self, axisangle):
        """Set this versor to the equivalent of the given axis
        and angle given in the ordered pair 'axisangle'."""
        axis, angle = axisangle
        if type(axis) not in (Vector, FreeVector):
            axis = FreeVector(axis)
        angle = utils.flt(angle)
        sa = np.sin(0.5 * angle)
        ca = np.cos(0.5 * angle)
        axis.normalize()
        self._s = ca
        self._v._data[:] = (sa * axis)._data

    axis_angle = property(get_axis_angle, set_axis_angle)

    def get_logarithm(self):
        """Return a rotation vector representing the rotation of this
        versor."""
        axis, angle = self.axis_angle
        return RotationVector((angle * axis)._data)

    def set_logarithm(self, rot_vec):
        """Set this versor to the equivalent of the given
        rotation vector 'rot_vec'."""
        if type(rot_vec) != RotationVector:
            rot_vec = RotationVector(rot_vec)
        angle = rot_vec.length
        if angle > utils.eps:
            axis = rot_vec.normalized
        else:
            # Select arbitrary x-direction as axis and set angle to zero
            axis = Vector.e1
            angle = 0.0
        self.axis_angle = (axis, angle)

    logarithm = log = property(get_logarithm, set_logarithm)
    rotation_vector = rot_vec = logarithm

    def get_orientation(self):
        """Return an Orientation object representing the same rotation as this
        versor. The method is taken from
        http://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation.
        """
        self.normalize()
        s = self._s
        v = self._v
        x = v.x
        y = v.y
        z = v.z
        x2 = x**2
        y2 = y**2
        z2 = z**2
        return Orientation(np.array([
            [1 - 2 * (y2 + z2), 2 * x * y - 2 * s * z, 2 * s * y + 2 * x * z],
            [2 * x * y + 2 * s * z, 1 - 2 * (x2 + z2), -2 * s * x + 2 * y * z],
            [-2 * s * y + 2 * x * z, 2 * s * x + 2 * y * z, 1 - 2 * (x2 + y2)]
        ]))

    def set_orientation(self, orient, positive=True):
        """Set this versor to represent the given orientation matrix in
        'orient'. The used method should be robust;
        cf. http://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation.
        The mentioned method from wikipedia has problems with certain
        orientations, like the identity. Therfore another robust
        method from
        http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/index.htm
        is used; the one from 'Angel'.
        """
        M = orient._data
        tr = M.trace() + 1.0
        if tr > 1e-10:
            s = 0.5 / np.sqrt(tr)
            self._s = 0.25 / s
            self._v.x = s * (M[2, 1] - M[1, 2])
            self._v.y = s * (M[0, 2] - M[2, 0])
            self._v.z = s * (M[1, 0] - M[0, 1])
        else:
            diag = M.diagonal()
            u = diag.argmax()
            v = (u + 1) % 3
            w = (v + 1) % 3
            r = np.sqrt(1 + M[u, u] - M[v, v] - M[w, w])
            if abs(r) < 1e-10:
                self._s = utils.flt(1.0)
                self._v = FreeVector(0, 0, 0)
            else:
                tworinv = 1.0 / (2 * r)
                self._s = utils.flt((M[w, v] - M[v, w]) * tworinv)
                self._v[u] = 0.5 * r
                self._v[v] = (M[u, v] + M[v, u]) * tworinv
                self._v[w] = (M[w, u] + M[u, w]) * tworinv
        if positive and self._s < 0:
            self.negate()
        self.normalize()

    orientation = property(get_orientation, set_orientation)


# Backwards compatibility
UnitQuaternion = Versor


def _test():
    print('Here should come a norm-warning:')
    Versor(1, 2, 3, 4, norm_warn=True)
    print('Here should be *no* norm-warning:')
    Versor(1, 2, 3, 4, norm_warn=False)
    print('Done')
    print('Testing power operator on Versor')
    assert(np.allclose((Versor(FreeVector(1, 0, 0))**2).log,
                       np.array([2, 0, 0])))
    print('Test for closeness of long and short rotation vectors (logarithm).')
    q = Versor(FreeVector(0, 0, -1/2 * np.pi))
    p = Versor(FreeVector(0, 0, 3/2 * np.pi))
    assert(np.allclose(q.rotation_vector, p.log)
           or np.allclose(q.log, (-p).log))
    print('Test multiplicative inverse')
    q = Quaternion(1, 2, 3, 4)
    assert(q * q.inverse == Quaternion())
    print('Testing versor transformation of vector')
    assert(Versor(FreeVector(0, 0, 3/2 * np.pi)) * FreeVector(1, 0, 0)
           == FreeVector(0, -1, 0))
