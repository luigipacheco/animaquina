# coding=utf-8

"""
Module implementing the Vector class.
"""

__author__ = "Morten Lind"
__copyright__ = "Morten Lind 2012-2021"
__credits__ = ["Morten Lind"]
__license__ = "LGPLv3"
__maintainer__ = "Morten Lind"
__email__ = "morten@lind.fairuse.org"
__status__ = "Production"


import typing

import numpy as np

from . import utils


class _UnitVectors(type):

    @property
    def e0(self):
        return FreeVector(1, 0, 0)
    ex = e0

    @property
    def e1(self):
        return FreeVector(0, 1, 0)
    ey = e1

    @property
    def e2(self):
        return FreeVector(0, 0, 1)
    ez = e2

    @property
    def unit_vectors(self):
        return [Vector.ex, Vector.ey, Vector.ez]


class _Vector4(object, metaclass=_UnitVectors):
    """A Vector is a 3D vector (member of R3) with standard Euclidian
    operations."""

    @classmethod
    def canCreateOn(cls, *arg):
        if type(arg) == cls:
            return True
        elif utils.is_sequence(arg):
            if len(arg) <= 3 and utils.is_num_types(arg):
                return True
            elif len(arg) == 1:
                return cls.canCreateOn(*arg[0])
            else:
                return False
        else:
            return False

    def __init__(self, *args, **kwargs):
        """Constructor for Vector. If optional keyword argument
        'position' evaluates to True, or is not given, the vector is
        represented as a position vector. Otherwise it is represented
        as a free vector."""
        if len(args) == 0:
            self._data = np.array([0, 0, 0], dtype=utils.flt)
        elif len(args) == 3 and utils.is_num_types(args):
            self._data = np.array(args, dtype=utils.flt)
        elif len(args) == 2 and utils.is_num_types(args):
            self._data = np.array((args[0], args[1], 0), dtype=utils.flt)
        elif len(args) == 1:
            arg = args[0]
            if utils.is_three_sequence(arg):
                self._data = np.array(arg, dtype=utils.flt)
            elif utils.is_sequence(arg) and len(arg) == 2:
                self._data = np.array((arg[0], arg[1], 0), dtype=utils.flt)
            elif type(arg) == Vector:
                self._data = arg.array
            elif hasattr(arg, "x") and hasattr(arg, "y") and hasattr(arg, "z"):
                self._data = np.array([arg.x, arg.y, arg.z])
            else:
                raise utils.Error(
                    ('__init__ : could not create vector on argument ' +
                     ': "{}" of type "{}"')
                    .format(str(args[0]), str(type(args[0]))))
        else:
            raise utils.Error('__init__ : could not create vector on ' +
                              'argument : "{}" of type "{}"'
                              .format(str(args[0]), str(type(args[0]))))
        self._is_position = kwargs.get('position', 1)

    def __copy__(self):
        """Copy method for creating a copy of this Vector."""
        return self.__class__(self)

    def __deepcopy__(self, memo):
        return self.__copy__()

    def copy(self, other=None):
        """Copy data from 'other' to self. If no argument given,
        i.e. 'other==None', return a copy of this Vector."""
        if other is None:
            return self.__class__(self)
        else:
            self._data[:] = other._data

    def __getattr__(self, name):
        if name == 'x':
            return self._data[0]
        elif name == 'y':
            return self._data[1]
        elif name == 'z':
            return self._data[2]
        else:
            raise AttributeError('Attribute "{}" not found in Vector'
                                 .format(name))

    def __setattr__(self, name, val):
        if name == 'x':
            self._data[0] = val
        elif name == 'y':
            self._data[1] = val
        elif name == 'z':
            self._data[2] = val
        # elif name == '_data':
        #     # Important for initialization? Or would
        #     # object.__setattr__ take care of it?
        #     self.__dict__[name] = val
        elif name == 'pos':
            if type(val) == Vector:
                self._data[:] = val.array
            elif utils.is_three_sequence(val):
                self._data[:] = np.array(val)
        else:
            object.__setattr__(self, name, val)

    # These pose some semantic problems with numpy array multiplication.
    # def __len__(self):
    #     return 3

    # def __iter__(self):
    #     return iter(self._data)

    def __getitem__(self, n):
        return self._data[n]

    def __setitem__(self, n, val):
        self._data[n] = val

    def __eq__(self, other):
        if isinstance(other, Vector):
            return np.allclose(self._data, other._data)
        else:
            return NotImplemented

    def __repr__(self):
        return ('<Vector4: ({:.5f}, {:.5f}, {:.5f})>'
                .format(*self._data))

    def __str__(self):
        return self.__repr__()

    def get_is_position(self):
        """If the vector is a position vector, the default, then it
        transforms differently than a real vector.
        """
        return self._is_position

    is_position = property(get_is_position)

    def __or__(self, v: 'Vector',
               normalize: bool = True):
        """Get the projection vector of this vector along a direction given by
        another vector, 'v'. If 'normalize' is True, the default, 'v'
        is first normalized. It is not mathematically clear, what the
        meaning of projections of and onto position vectors is, so use
        with care!
        """
        if type(v) == np.ndarray and v.shape == (3,):
            v = FreeVector(v)
        if not isinstance(v, Vector):
            return NotImplemented
        if normalize:
            v_hat = v.normalized
        else:
            v_hat = v
        return FreeVector((self * v_hat) * v_hat)

    projected_along = __or__

    def __xor__(self, v: typing.Union['Vector', np.ndarray],
                normalize: bool = True):
        """Return a new vector which is this vector with it's component along
        'v' removed. If 'normalize' is True, the default, a normalized
        version of 'v' is used. It is not mathematically clear, what
        the meaning of projections of and onto position vectors is, so
        use with care!
        """
        return self - self.projected_along(v, normalize)

    removed_along = __xor__

    def angle(self, other, minimal=False):
        """Return the angle (radians) to the 'other' vector. This is the
        absolute, positive angle unless minimal is True, in which case
        the minimal of the two angles between the vectors is returned.
        """
        costheta = (self * other) / (self.length * other.length)
        if costheta > 1:
            costheta = 1
        elif costheta < -1:
            costheta = -1
        ang = np.arccos(costheta)
        if minimal:
            return min(ang, np.pi-ang)
        else:
            return ang

    def signed_angle_to(self, other, ref_vec=None):
        """With default reference rotation vector as Z-axis (if
        'ref_vec' == None), compute the signed angle of rotation from
        self to 'other'.
        """
        theta = self.angle(other)
        xprod = self & other
        if ref_vec is not None:
            if xprod * ref_vec < 0:
                theta = -theta
        else:
            if xprod.z < 0:
                theta = -theta
        return theta

    def signed_angle(self, other, ref_vec=None):
        utils._deprecation_warning('signed_angle_to')
        return self.signed_angle_to(other, ref_vec=None)

    def get_length(self):
        """Return the Euclidean length."""
        return np.sqrt(self.length_squared)

    length = property(get_length)

    def get_length_squared(self):
        """Return the square of the standard Euclidean length."""
        return self._data @ self._data

    length_squared = property(get_length_squared)

    def normalize(self):
        """In-place normalization of this Vector."""
        length = self.length
        if length != 1.0:
            self._data = self._data / length

    def get_normalized(self):
        """Return a normalized Vector with same direction as this
        one.
        """
        nv = self.__class__(self)
        nv.normalize()
        return nv

    normalized = property(get_normalized)

    def dist(self, other):
        """Compute the Euclidean distance between points given by self
        and 'other'."""
        if isinstance(other, Vector):
            return np.linalg.norm(self._data - other._data)
        else:
            return NotImplemented

    def dist_squared(self, other):
        """Compute the square of the Euclidean distance between points given by self
        and 'other'."""
        utils._deprecation_warning('dist()**2')
        if isinstance(other, Vector):
            return self.dist(other) ** 2
        else:
            return NotImplemented

    def get_cross_operator(self):
        """Return the cross product operator for this Vector. I.e. the
        skew-symmetric operator cross_op, such that cross_op * u == v
        x u, for any vector u."""
        # cross_op = np.zeros((3,3))
        # cross_op[0, 1] = -self._data[2]
        # cross_op[0, 2] = self._data[1]
        # cross_op[1, 0] = self._data[2]
        # cross_op[1, 2] = -self._data[0]
        # cross_op[2, 0] = -self._data[1]
        # cross_op[2, 1] = self._data[0]
        # return cross_op
        x, y, z = self._data
        return np.array(
            (0, -z, y), (z, 0, -x), (-y, x, 0))

    cross_operator = skew = property(get_cross_operator)

    def get_alt_cross_operator(self):
        """Return the cross product operator for this Vector. I.e. the
        skew-symmetric operator cross_op, such that cross_op * u == v
        x u, for any vector u."""
        cross_op = np.zeros((3, 3))
        # cross_op[0, 1] = -self._data[2]
        cross_op[0, 2] = self._data[1]
        cross_op[1, 0] = self._data[2]
        # cross_op[1, 2] = -self._data[0]
        # cross_op[2, 0] = -self._data[1]
        cross_op[2, 1] = self._data[0]
        return cross_op - cross_op.T

    alt_cross_operator = property(get_alt_cross_operator)

    def __and__(self, other):
        """Return the cross product with 'other'."""
        # If other is a Vector, take out data
        if isinstance(other, Vector):
            other = other._data
        # Check that other now is an ndarray of correct shape
        if type(other) != np.ndarray or other.shape != (3,):
            return NotImplemented
        return FreeVector(np.cross(self._data, other))

    cross = __and__

    def get_array(self):
        """Return a copy of the ndarray which is the fundamental data
        of the Vector."""
        return self._data.copy()

    def set_array(self, array, check=True):
        """Set the vector by three values in the iterable 'array'.
        """
        if check and len(array) != 3:
            raise utils.Error(
                self.__class__.__name__ +
                'Setting the value by the "array" property needs exactly'
                + ' three values. ({} were given)'.format(len(array)))
        self._data[:] = array

    array = property(get_array, set_array)

    def get_array_ref(self):
        """Return a reference to the (3,) ndarray, which is the
        fundamental data of the Orientation. Caution: Use this method
        only for optimization, since it eliminates copying, and be
        sure not to compromize the data.
        """
        return self._data

    array_ref = property(get_array_ref)

    def get_list(self):
        """Return the fundamental data of the Vector as a list."""
        return self._data.tolist()
    list = property(get_list)

    def get_matrix(self):
        """Property for getting a single-column np-matrix with the data
        from the vector."""
        return np.matrix(self._data).T

    matrix = property(get_matrix)

    def get_column(self):
        """Property for getting a single-column array with the data
        from the vector."""
        return self._data.reshape((3, 1))

    column = property(get_column)

    def __sub__(self, other):
        """Subtract another vector from this. The semantics regarding
        free and position vectors should be: If this is free, and
        other is a position, or opposite, the new should be
        position. If both are free or both are positions, the new
        should be free."""
        if type(self) == Vector:
            if isinstance(other, Vector):
                return Vector(self._data - other._data)
            elif type(other) == np.ndarray and other.shape == (3,):
                # Other is given as a triplet in ndarray
                return Vector(self._data - other)
            else:
                return NotImplemented
        elif isinstance(other, Vector):
            if type(self) == PositionVector:
                if type(other) == FreeVector:
                    return PositionVector(self._data - other._data)
                elif type(other) == PositionVector:
                    return FreeVector(self._data - other._data)
                else:
                    return NotImplemented
            elif type(self) == FreeVector:
                if type(other) == FreeVector:
                    return FreeVector(self._data - other._data)
                else:
                    # raise utils.Error('Can only subtract a free vector from a free vector.')
                    return NotImplemented
            else:
                return NotImplemented
        # elif utils.is_sequence(other):
        #     # Assume a sequence of objects that may be multiplied
        #     return [self - o for o in other]
        else:
            return NotImplemented

    def __isub__(self, other):
        # if type(other) == Vector:
        #     self._data -= other._data
        # elif type(other) == np.ndarray and other.shape == (3,):
        #     # Other is given as a triplet in ndarray
        #     self._data -= other
        # else:
        #     return NotImplemented
        self._data[:] = (self - other)._data
        return self

    def __mul__(self, other):
        """Multiplication with an 'other' Vector (inner product) or
        with a scalar."""
        if isinstance(other, Vector):
            return self._data @ other._data
        elif utils.is_num_type(other):
            return self.__class__(self._data * other)
        elif type(other) == np.ndarray and other.shape == (3,):
            # Other is given as a triplet in ndarray
            return self.__class__(self._data * other)
        # elif utils.is_sequence(other):
        #     # Assume a sequence of objects that may be multiplied
        #     # WARNING: v * [1,2,3] == [1*v, 2*v, 3*v] !
        #     return [self * o for o in other]
        else:
            return NotImplemented

    def __imul__(self, other):
        """In-place multiplication with a scalar, 'other'. """
        if utils.is_num_type(other):
            self._data *= other
        else:
            return NotImplemented
            # raise utils.Error('__imul__ : Could not multiply by non-number')
        return self

    def __rmul__(self, other):
        """Right multiplication with a scalar, 'other'. """
        if utils.is_num_type(other):
            return self.__class__(other * self._data)
        else:
            raise utils.Error('__rmul__ : Could not multiply by non-number')

    def __truediv__(self, other):
        """Division with a scalar, 'other'. """
        if utils.is_num_type(other):
            if np.isclose(other, 0.0):
                raise ZeroDivisionError(f'In division of vector by scalar. Divisor: {other}')
            return self.__class__((1.0 / other) * self._data)
        else:
            raise utils.Error('__truediv__ : Could not divide by non-number')
    __div__ = __truediv__

    def __add__(self, other):
        """Return the sum of this and the 'other' vector."""
        if type(self) == Vector:
            if type(other) == Vector:
                return Vector(self._data + other._data)
            elif type(other) == np.ndarray and other.shape == (3,):
                # Other is given as a triplet in ndarray
                return self.__class__(self._data + other)
            else:
                return NotImplemented
        elif isinstance(other, Vector):
            if((type(self) == PositionVector and type(other) == FreeVector) or
               (type(self) == FreeVector and type(other) == PositionVector)):
                return PositionVector(self._data + other._data)
            elif type(self) == FreeVector and type(other) == FreeVector:
                return FreeVector(self._data + other._data)
            else:
                return NotImplemented
        else:
            return NotImplemented
            # raise utils.Error('__add__ : Could not add non-vector')

    def __iadd__(self, other):
        """In-place add the 'other' vector to this vector."""
        # if type(other) == Vector:
        #     self._data += other._data
        # elif type(other) == np.ndarray and other.shape == (3,):
        #     # Other is given as a triplet in ndarray
        #     self._data += other
        # else:
        #     return NotImplemented
        #     # raise utils.Error('__iadd__ : Could not add non-vector')
        # return self
        self._data[:] = (self + other)._data
        return self

    def __neg__(self):
        return self.__class__(-self._data)

    @classmethod
    def new_random_unit_vector(cls):
        """Generator for random vectors uniformly sampled on S2. Use the
        Muller's algorithm from "A note on a method for generating
        points uniformly on n-dimensional spheres", Communications of
        the ACM, Volume 2, Issue 4, April 1959, pp 19-20.
        """
        v = cls(np.random.normal(size=3))
        v.normalize()
        return v


class Vector(_Vector4):

    def __repr__(self):
        return ('<Vector: ({:.5f}, {:.5f}, {:.5f})>'
                .format(*self._data))


class FreeVector(Vector):

    def __repr__(self):
        return ('<FreeVec: ({:.5f}, {:.5f}, {:.5f})>'
                .format(*self._data))


class PositionVector(Vector):

    def __repr__(self):
        return ('<PosVec: ({:.5f}, {:.5f}, {:.5f})>'
                .format(*self._data))


def _test_construction():
    print((Vector.canCreateOn(1, 2, 3),
           Vector.canCreateOn((1, 2, 3)),
           Vector.canCreateOn(1, 2)))


def _test_signed_angle():
    v = FreeVector(1, 2, 3)
    u = FreeVector(3, 1, 2)
    print(v.signed_angle_to(u))
    return True


def _test_rops():
    """Test that rop on other is called when NotImplemented is
    returned from the Vector object."""
    class A(object):
        def __rmul__(self, other):
            print('rmul from A')
            return other * 2.0
    v = Vector(1, 2, 3)
    v_origin = v.copy()
    a = A()
    if v*a != 2*v:
        return False
    v *= a
    if v != 2 * v_origin:
        return False
    u = Vector(3, 1, 2)
    print(v.signed_angle_to(u))
    return True


def _test_projections_and_cross_product():
    v0 = FreeVector([1, 1, 3])
    print(f'v0: {v0}')
    v1 = FreeVector([1, 1, 0])
    print(f'v1: {v1}')
    p0 = PositionVector([1, 2, 1])
    print(f'p0: {p0}')
    v0prj = v0 | v1
    print(f'Projected v0 along v1 "|": {v0prj}')
    assert((v0prj - v1).length < utils.eps)
    v0prj = v0.projected_along(v1.normalized, normalize=False)
    assert((v0prj - v1).length < utils.eps)
    v0orth = v0 ^ v1
    print(f'Removed v1 component from v0 "^": {v0orth}')
    assert(abs(v0orth * v1) < utils.eps)
    # Also test cross operator
    vx = v0 & v1
    print(f'Cross product "v0 & v1": {vx}')
    assert((vx | v0).length < utils.eps and (vx | v0).length < utils.eps)
    vxp = v0 & p0
    print(f'Cross product "v0 & p0": {vxp}')
    a = np.array([0,1,0])
    pxa = p0 & a
    print(f'Cross product "p0 & {a}": {pxa}')


def _test_free_pos_vectors():
    v = FreeVector(1, 2, 3)
    v1 = FreeVector(0, 1, 2)
    p = PositionVector(1, 2, 3)
    p1 = PositionVector(1, 1, 1)
    print('p + v', p + v)
    print('v + p', v + p)
    print('v + v', v + v)
    print('p - p1', p - p1)
    print('p - v', p - v)
    print('v - v1', v - v1)
    try:
        print('p + p', p + p)
    except TypeError:
        print('TypeError correctly raised when attempting "p + p".')
    try:
        print('v - p', v - p)
    except TypeError:
        print('TypeError correctly raised when attempting "v - p".')



# def _test_vectorized_operations():
#     # Test multiplication of a list
#     v = Vector(1, 0, 0)
#     vs = [Vector(1, 0, 0), Vector(0, 1, 0)]
#     rms = v * vs
#     assert(rms[0] == v * vs[0])
#     assert(rms[1] == v * vs[1])
#     ras = v + vs
#     assert(ras[0] == v + vs[0])
#     assert(ras[1] == v + vs[1])
#     rss = v + vs
#     assert(rss[0] == v + vs[0])
#     assert(rss[1] == v + vs[1])
