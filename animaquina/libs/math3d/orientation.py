# coding=utf-8

"""
Module implementing the Orientation class. The orientation is
represented internally by an orthogonal 3x3 matrix.
"""

__author__ = "Morten Lind"
__copyright__ = "Morten Lind 2012-2021"
__credits__ = ["Morten Lind"]
__license__ = "LGPLv3"
__maintainer__ = "Morten Lind"
__email__ = "morten@lind.fairuse.org"
__status__ = "Production"

import numpy as np

# Circular dependencies prevents direct import of Versor, hence
# global addressing.
import math3d as m3d

from . import utils
from .vector import Vector, FreeVector


class RotationVector:  # (np.ndarray):
    class Error(Exception):
        def __init__(self, message):
            self.message = message
            Exception.__init__(self, self.message)

        def __repr__(self):
            return self.__class__.__qualname__ + ': ' + self.message

    def __init__(self, *args, **kwargs):
        self._data = np.zeros(3)
        if len(args) > 0:
            # args take precedence
            if len(args) == 1:
                # Take three numbers
                if type(args[0]) in (Vector, FreeVector, RotationVector):
                    self._data[:] = args[0]._data
                elif len(args[0]) == 3:
                    self._data[:] = args[0]
                else:
                    raise self.Error('Given one positional argument which ' +
                                     f'was not of length three: {args[0]}')
            elif len(args) == 3:
                self._data[:] = args
            else:
                raise self.Error(f'Can not create on {len(args)} ' +
                                 'positional arguments')

    def __repr__(self):
        return ('<RotVec: ({:.5f}, {:.5f}, {:.5f})>'
                .format(*self._data))

    def copy(self):
        return self.__class__(self._data.copy())

    def get_length_squared(self):
        """Return the square of the standard Euclidean length."""
        return self._data @ self._data

    length_squared = property(get_length_squared)

    def get_length(self):
        """Return the Euclidean length."""
        return np.sqrt(self.length_squared)

    length = property(get_length)

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
        """Return a reference to the (3,)-ndarray, which is the
        fundamental data of the RotationVector.  Caution: Use this method
        only for optimization, since it eliminates copying, and be
        sure not to compromize the data.
        """
        return self._data

    array_ref = property(get_array_ref)


class Orientation(object):
    """An Orientation is a member of SO(3) which can be used either to
    perform a rotational transformation, or for keeping an orientation
    in 3D.
    """

    def __create_on_sequence(self, seq):
        if type(seq) in (list, tuple):
            seq = np.array(seq)
        if type(seq) != np.ndarray:
            raise utils.Error('Creating on a sequence requires numpy array, ' +
                              'list or tuple')
        if seq.shape in ((9,), (3, 3)):
            self._data = seq.copy()
            # // Ensure the right shape.
            self._data.shape = (3, 3)
        elif seq.shape == (3,):
            self._data = np.identity(3)
            self.rotation_vector = seq
        else:
            raise utils.Error('Creating on a numpy array requires shape ' +
                              '(3,), (9,) or (3,3)!')

    def __init__(self, *args):
        """Create an orientation on either of the following arguments:

        * An Orientation.

        * A RotationVector

        * A Versor

        * Three FreeVectors or numpy arrays of shape (3,) interpreted as
          columns of the matrix.

        * One FreeVector, numpy array, list, or tuple of shape (3,)
          interpreted as a rotation vector.

        * A numpy array, list, or tuple of shape (3,3) or (9,) for
          giving direct matrix data; using row major order.
        """
        if len(args) == 1:
            arg = args[0]
            if type(arg) == Orientation:
                self._data = arg.array
            elif type(arg) == m3d.Versor:
                self._data = arg.orientation._data
            elif type(arg) in [FreeVector, RotationVector]:
                if type(arg) == FreeVector:
                    utils._deprecation_warning('RotationVector')
                # Interpret as a rotation vector
                self._data = np.identity(3)
                self.rotation_vector = arg
            elif utils.is_sequence(arg):
                self.__create_on_sequence(arg)
            else:
                raise utils.Error(
                    'Creating on type {} is not supported'
                    .format(str(type(arg))))
        elif len(args) == 3:
            if np.all(np.array([type(a) == FreeVector for a in args])):
                array_args = (a._data for a in args)
            elif np.all(np.array([type(a) == np.ndarray for a in args])):
                array_args = args
            else:
                raise utils.Error(
                    'Creating on three arguments requires three vectors ' +
                    'or three numpy arrays of shape (3,)!')
            # Stack the vector data vertically and transpose to get
            # them into columns.
            self._data = np.transpose(np.vstack((va for va in array_args)))
        elif len(args) == 0:
            self._data = np.identity(3)
        # Always ensure that we use utils.flt as fundamental type
        self._data = self._data.astype(utils.flt)

    def __copy__(self):
        """Copy method for creating a copy of this Orientation."""
        return Orientation(self)

    def __deepcopy__(self, memo):
        return self.__copy__()

    def copy(self, other=None):
        """Copy data from 'other' to self. If no argument given,
        i.e. 'other==None', return a copy of this Orientation
        """
        if other is None:
            return Orientation(self)
        else:
            self._data[:, :] = other._data

    def get_vec_x(self):
        """Return the x-direction of the moving coordinate system in
        base reference as a FreeVector.
        """
        return FreeVector(self._data[:, 0])

    vec_x = property(get_vec_x)

    def get_col_x(self):
        """Return the x-direction of the moving coordinate system in
        base reference as an array."""
        return self._data[:, 0]

    col_x = property(get_col_x)

    def get_vec_y(self):
        """Return the y-direction of the moving coordinate system in
        base reference as a FreeVector.
        """
        return FreeVector(self._data[:, 1])

    vec_y = property(get_vec_y)

    def get_col_y(self):
        """Return the y-direction of the moving coordinate system in
        base reference as an array.
        """
        return self._data[:, 1]

    col_y = property(get_col_y)

    def get_vec_z(self):
        """Return the z-direction of the moving coordinate system in
        base reference as a FreeVector.
        """
        return FreeVector(self._data[:, 2])

    vec_z = property(get_vec_z)

    def get_col_z(self):
        """Return the z-direction of the moving coordinate system in
        base reference as an array.
        """
        return self._data[:, 2]

    col_z = property(get_col_z)

    def __getitem__(self, index):
        """Return the 'index''th axis vector (column) of this orientation
        matrix.
        """
        return self._data[:, index]

    def __eq__(self, other):
        if type(other) == Orientation:
            return np.sum((self._data-other._data)**2) < utils.eps
        else:
            return NotImplemented
            # raise utils.Error('Could not compare to non-Orientation!')

    def get_repr_error(self):
        """Compute and return the square root of the sum of squared
        dot products of the axis vectors, as a representation of the
        error of the orientation matrix.
        """
        vec_x = self.vec_x
        vec_y = self.vec_y
        vec_z = self.vec_z
        sq_sum = (vec_x * vec_y)**2
        sq_sum += (vec_y * vec_z)**2
        sq_sum += (vec_z * vec_x)**2
        sq_sum += np.abs(1 - vec_x * vec_x)
        sq_sum += np.abs(1 - vec_y * vec_y)
        sq_sum += np.abs(1 - vec_z * vec_z)
        return np.sqrt(sq_sum)

    repr_error = property(get_repr_error)

    def orthonormalize(self):
        """Perform an in-place Gram-Schmidt."""
        # Trust the x-vector but normalize
        self._data[:, 0] /= np.linalg.norm(self._data[:, 0])
        # Form y-vector by removing x-projection and normalizing
        self._data[:, 1] -= (self._data[:, 1] @
                                   self._data[:, 0]) * self._data[:, 0]
        self._data[:, 1] /= np.linalg.norm(self._data[:, 1])
        # Form z-vector by cross product #  and normalization
        self._data[:, 2] = np.cross(self._data[:, 0], self._data[:, 1])
        # self._data[:, 2] /= np.linalg.norm(self._data[:, 2])

    # def renormalize(self):
    #     """Correct the axis vectors by a Gram-Schmidt procedure."""
    #     colx = self._data[:,0]
    #     coly = self._data

    def from_xy(self, x_vec, y_vec):
        """Reset this orientation to the one that conforms with the
        given x and y directions.
        """
        if type(x_vec) != FreeVector:
            x_vec = FreeVector(x_vec)
        if type(y_vec) != FreeVector:
            y_vec = FreeVector(y_vec)
        x_vec = x_vec.normalized
        y_vec = y_vec.normalized
        if np.abs(x_vec * y_vec) > utils.eps:
            print('Warning: Orthonormalizing y_vec on x_vec!')
            y_vec -= (x_vec * y_vec) * x_vec
            y_vec.normalize()
        self._data[:, 0] = x_vec._data
        self._data[:, 1] = y_vec._data
        self._data[:, 2] = x_vec.cross(y_vec)._data

    def from_xz(self, x_vec, z_vec):
        """Reset this orientation to the one that conforms with the
        given x and z directions.
        """
        if type(x_vec) != FreeVector:
            x_vec = FreeVector(x_vec)
        if type(z_vec) != FreeVector:
            z_vec = FreeVector(z_vec)
        x_vec = x_vec.normalized
        z_vec = z_vec.normalized
        if np.abs(x_vec * z_vec) > utils.eps:
            print('Warning: Orthonormalizing z_vec on x_vec!')
            z_vec -= (x_vec * z_vec) * x_vec
            z_vec.normalize()
        self._data[:, 0] = x_vec._data
        self._data[:, 1] = z_vec.cross(x_vec)._data
        self._data[:, 2] = z_vec._data

    def from_yz(self, y_vec, z_vec):
        """Reset this orientation to the one that conforms with the
        given x and z directions.
        """
        if type(y_vec) != FreeVector:
            y_vec = FreeVector(y_vec)
        if type(z_vec) != FreeVector:
            z_vec = FreeVector(z_vec)
        y_vec = y_vec.normalized
        z_vec = z_vec.normalized
        if np.abs(y_vec * z_vec) > utils.eps:
            print('Warning: Orthonormalizing z_vec on x_vec!')
            z_vec -= (y_vec * z_vec) * y_vec
            z_vec.normalize()
        self._data[:, 0] = y_vec.cross(z_vec)._data
        self._data[:, 1] = y_vec._data
        self._data[:, 2] = z_vec._data

    def get_versor(self):
        """Return a versor representing this orientation."""
        return m3d.Versor(self)

    # Deprecated
    get_unit_quaternion = get_versor
    get_quaternion = get_unit_quaternion

    def set_versor(self, vers):
        """Set the orientation to that of the versor given in
        'vers'.
        """
        self._data[:, :] = vers.orientation._data

    # Deprecated
    set_unit_quaternion = set_versor
    set_quaternion = set_unit_quaternion

    versor = property(get_versor, set_versor)

    # Deprecated
    unit_quaternion = versor
    quaternion = unit_quaternion

    def get_logarithm(self):
        """Return a rotation vector representing this orientation. This is
        essentially the logarithm of the rotation matrix.
        """
        return self.versor.rotation_vector

    # Deprecated
    get_rotation_vector = get_logarithm

    def set_logarithm(self, rot_vec):
        """Set this Orientation to represent the one given in a rotation
        vector in 'rot_vec'. 'rot_vec' must be a FreeVector or an
        numpy array of shape (3,).
        """
        if type(rot_vec) not in (FreeVector, RotationVector):
            rot_vec = RotationVector(rot_vec)
        angle = rot_vec.length
        if np.isclose(angle, 0.0):
            self._data = np.identity(3)
        else:
            axis = FreeVector(rot_vec._data / angle)
            self.axis_angle = (axis, angle)

    # Deprecated
    set_rotation_vector = set_logarithm

    logarithm = log = rotation_vector = rot_vec = rv = property(get_logarithm,
                                                                set_logarithm)

    def get_axis_angle(self):
        """Return an (axis,angle) pair representing the equivalent
        orientation."""
        return m3d.Versor(self).axis_angle

    def set_axis_angle(self, ax_ang):
        """Set this orientation to the equivalent to rotation of 'angle'
        around 'axis'. Only the direction of the 'axis' is used, so it
        may have any length greater than zero.
        """
        axis, angle = ax_ang
        if type(axis) in (Vector, FreeVector):
            axis = axis.array
        # Force normalization
        axis /= np.linalg.norm(axis)
        x = axis[0]
        y = axis[1]
        z = axis[2]
        ca = np.cos(angle)
        sa = np.sin(angle)
        self._data[:, :] = np.array((
            (ca + (1 - ca) * x**2,
             (1 - ca) * x * y - sa * z,
             (1 - ca) * x * z + sa * y),
            ((1 - ca) * x * y + sa * z,
             ca + (1 - ca) * y**2,
             (1 - ca) * y * z - sa * x),
            ((1 - ca) * x * z - sa * y,
             (1 - ca) * y * z + sa * x,
             ca + (1 - ca) * z**2)))

    axis_angle = ax_ang = aa = property(get_axis_angle, set_axis_angle)

    def set_to_x_rotation(self, angle):
        """Replace this orientation by that of a rotation around x."""
        ca = np.cos(angle)
        sa = np.sin(angle)
        self._data[:, :] = np.array(((1, 0, 0), (0, ca, -sa), (0, sa, ca)))

    def set_to_y_rotation(self, angle):
        """Replace this orientation by that of a rotation around y."""
        ca = np.cos(angle)
        sa = np.sin(angle)
        self._data[:, :] = np.array(((ca, 0, sa), (0, 1, 0), (-sa, 0, ca)))

    def set_to_z_rotation(self, angle):
        """Replace this orientation by that of a rotation around z."""
        ca = np.cos(angle)
        sa = np.sin(angle)
        self._data[:, :] = np.array(((ca, -sa, 0), (sa, ca, 0), (0, 0, 1)))

    def rotate_t(self, axis, angle):
        """In-place rotation of this orientation angle radians in axis
        perceived in the transformed reference system.
        """
        o = Orientation()
        o.axis_angle = (axis, angle)
        self.copy(self * o)

    rotate = rotateT = rotate_t

    def rotate_b(self, axis, angle):
        """In-place rotation of this orientation angle radians in axis
        perceived in the base reference system.  Arguments:
        axis -- the axis to rotate about (unit vector with direction).
        angle -- the angle in radians to rotate about the axis.
        """
        o = Orientation()
        o.axis_angle = (axis, angle)
        self.copy(o * self)

    rotateB = rotate_b

    def rotate_xb(self, angle):
        """In-place rotation of this oriantation by a rotation around
        x axis in the base reference system. (Inefficient!)
        """
        self.rotate_b(Vector.e0, angle)

    rotateXB = rotate_xb

    def rotate_yb(self, angle):
        """In-place rotation of this oriantation by a rotation around
        y axis in the base reference system. (Inefficient!)
        """
        self.rotate_b(Vector.e1, angle)

    rotateYB = rotate_yb

    def rotate_zb(self, angle):
        """In-place rotation of this oriantation by a rotation around
        z axis in the base reference system. (Inefficient!)
        """
        self.rotate_b(Vector.e2, angle)

    rotateZB = rotate_zb

    def rotate_xt(self, angle):
        """In-place rotation of this oriantation by a rotation around
        x axis in the transformed reference system. (Inefficient!)
        """
        self.rotate_t(Vector.e0, angle)

    rotate_x = rotateX = rotateXT = rotate_xt

    def rotate_yt(self, angle):
        """In-place rotation of this oriantation by a rotation around
        y axis in the transformed reference system. (Inefficient!)
        """
        self.rotate_t(Vector.e1, angle)

    rotate_y = rotateY = rotateYT = rotate_yt

    def rotate_zt(self, angle):
        """In-place rotation of this oriantation by a rotation around
        z axis in the transformed reference system. (Inefficient!)
        """
        self.rotate_t(Vector.e2, angle)

    rotate_z = rotateZ = rotateZT = rotate_zt

    def __repr__(self):
        # return '<Orientation: \n' + repr(self._data) + '>'
        rvr = self.rot_vec.array_ref
        return f'<Orientation: RV({rvr[0]:.3f}, {rvr[1]:.3f}, {rvr[2]:.3f})>'

    def __str__(self):
        return self.__repr__()

    def get_ang_norm(self):
        """Return the angular norm, i.e. the angular rotation, of
        this orientation."""
        return self.axis_angle[1]

    ang_norm = property(get_ang_norm)

    def ang_dist(self, other):
        """Return the orientation distance (the angle of rotation) to
        the 'other' orientation.
        """
        return (self.inverse * other).versor.ang_norm

    def invert(self):
        """In-place inversion of this orientation."""
        self._data[:, :] = self._data.T

    def get_inverse(self):
        """Return an inverse of this orientation as a rotation."""
        o = Orientation(self._data)
        o.invert()
        return o

    inverse = property(get_inverse)

    def __matmul__(self, other):
        if type(other) == FreeVector:
            return FreeVector(self._data @ other._data)
        elif type(other) == Orientation:
            return Orientation(self._data @ other._data)
        elif type(other) == m3d.Versor:
            return self * other.orientation
        elif type(other) == RotationVector:
            return RotationVector(self._data @ other._data)
        elif isinstance(other, np.ndarray):
            if other.shape == (3,):
                return self._data @ other
            elif other.shape == (6,):
                # Individually transfor two appended free vectors,
                # e.g. twists or wrenches
                return (self._data @ other.reshape((3, 2), order='F')
                        ).reshape(6, order='F')
        # elif utils.is_sequence(other):
        #     # Assume a sequence of objects that may be multiplied
        #     return [self * o for o in other]
        else:
            return NotImplemented
            # raise utils.Error('Multiplication by something other than'
            #                  + 'Orientation, FreeVector, or a sequence '
            #                  + 'of these, is not allowed!')

    __mul__ = __matmul__

    def get_matrix(self):
        """Return an np-matrix with the data from the orientation.
        """
        return np.matrix(self._data)

    matrix = property(get_matrix)

    def get_array(self):
        """Return a copy of the ndarray which is the fundamental data
        of the Orientation.
        """
        return self._data.copy()

    def set_array(self, array, check=True):
        """Set the orientation matrix data by the nine values in the doubly
        iterable 'array'.
        """
        if check:
            if len(array) != 3:
                raise utils.Error(
                    'Setting the value by the "array" property needs exactly'
                    + ' three rows. ({} were given)'.format(len(array)))
            for row in array:
                if len(row) != 3:
                    raise utils.Error(
                        'Setting the values of a row in the "array" property '
                        + 'needs exactly three values.({} were given)'
                        .format(len(row)))
        self._data[:] = array
        if check:
            self.orthonormalize()

    array = property(get_array, set_array)

    def get_array_ref(self):
        """Return a reference to the (3,3) ndarray, which is the
        fundamental data of the Orientation.  Caution: Use this method
        only for optimization, since it eliminates copying, and be
        sure not to compromize the data.
        """
        return self._data

    array_ref = property(get_array_ref)

    def get_list(self):
        """Return the fundamental data of the Orientation as a
        list.
        """
        return self._data.tolist()

    list = property(get_list)

    @classmethod
    def new_from_xy(cls, x_vector, y_vector):
        """Factory for a new orientation with given x- and
        y-direction.
        """
        o = Orientation()
        o.from_xy(x_vector, y_vector)
        return o

    @classmethod
    def new_from_xz(cls, x_vector, z_vector):
        """Factory for a new orientation with given x- and
        z-direction.
        """
        o = Orientation()
        o.from_xz(x_vector, z_vector)
        return o

    @classmethod
    def new_from_yz(cls, y_vector, z_vector):
        """Factory for a new orientation with given y- and
        z-direction.
        """
        o = Orientation()
        o.from_yz(y_vector, z_vector)
        return o

    @classmethod
    def new_rot_x(cls, angle):
        """Factory for a new orientation which is a rotation in the
        signed angle 'angle' around the x-direction.
        """
        o = Orientation()
        o.set_to_x_rotation(angle)
        return o

    @classmethod
    def new_rot_y(cls, angle):
        """Factory for a new orientation which is a rotation in the
        signed angle 'angle' around the y-direction.
        """
        o = Orientation()
        o.set_to_y_rotation(angle)
        return o

    @classmethod
    def new_rot_z(cls, angle):
        """Factory for a new orientation which is a rotation in the
        signed angle 'angle' around the z-direction.
        """
        o = Orientation()
        o.set_to_z_rotation(angle)
        return o

    # // The Euler encoding map is used to easily apply rotations
    # according the the 'xyzXYZ' encoding scheme for Euler angles
    _euler_encoding_map = {'x': rotate_xb, 'y': rotate_yb, 'z': rotate_zb,
                           'X': rotate_xt, 'Y': rotate_yt, 'Z': rotate_zt}

    @classmethod
    def new_from_vec_to_vec(cls, from_vec, to_vec):
        """Factory for a new orientation which is the minimal rotation which
        rotates 'from_vec' to 'to_vec'. Technically the axis-angle
        that can be computed for the two vectors.
        """
        # Ensure that the vectors are m3d unit vectors
        from_vec = FreeVector(from_vec).normalized
        to_vec = FreeVector(to_vec).normalized
        # Angle between the vectords
        angle = from_vec.angle(to_vec)
        # Consider cases
        if angle <= utils.sqrt_eps:
            # // Identity
            return Orientation()
        elif angle < np.pi - utils.sqrt_eps:
            # // Regular, minimal rotation
            return Orientation(angle * from_vec.cross(to_vec).normalized)
        else:
            # // Find any suitable rotation axis
            x_angle = FreeVector.ex.angle(from_vec)
            if x_angle > 1e-3 and x_angle < np.pi - 1.0e-3:
                return Orientation(angle *
                                   FreeVector.ex.cross(from_vec).normalized)
            y_angle = FreeVector.ey.angle(from_vec)
            if y_angle > 1e-3 and y_angle < np.pi - 1.0e-3:
                return Orientation(angle *
                                   FreeVector.ey.cross(from_vec).normalized)
            z_angle = FreeVector.ez.angle(from_vec)
            if z_angle > 1e-3 and z_angle < np.pi - 1.0e-3:
                return Orientation(angle *
                                   FreeVector.ez.cross(from_vec).normalized)

    new_vec_to_vec = new_from_vec_to_vec

    @classmethod
    def new_from_euler(cls, angles, encoding='ZYX'):
        """Factory for generating a new orientation from Euler or
        Tait-Bryan angles. 'angles' must be a sequence of three real
        numbers giving the Euler or Tait-Bryan angles. 'encoding' must
        be three characters, all from the set 'xyzXYZ'. The encoding
        denotes the sequence of axes to rotate around and the case of
        the characters in the encoding string tells if it should be
        intrinsic or extrinsic axes for the rotation; all rotations
        must be either intrinsic or extrinsic. Here the notation is
        adopted from 'http://en.wikipedia.org/wiki/Euler_angles'. A
        lower case character, e.g. 'x', denotes a rotation around the
        extrinsic axis, i.e. the given axis of the initial coordinate
        system. An upper-case character, e.g. 'X', denotes a rotation
        around the axis in the intrinsic, i.e. moved, coordinate
        system at that particular instance of the sequence. A
        classical example of proper Euler angles are alpha-beta-gamma
        angles consisting of all intrinsic rotations, first alpha
        around the inital z-axis, then beta around rotated x-axis, and
        finally gamma around rotated z-axis; this is encoded by
        'ZXZ'. Note that proper Euler angles always address two
        different axes, the same (intrinsic) axis is used for the
        first and third rotation. Tait-Bryan angles address three
        different axes and classical examples are roll-pitch-yaw,
        which are encoded as 'ZYX', or yaw-pitch-roll, encoded by
        'XYZ'. Any sequence of intrinsic rotations may be converted to
        a corresponding sequence of extrinsic rotations by reversing
        the angle sequence; e.g. arguments ((alpha,beta,gamma), 'ZYX')
        gives the same rotation as ((gamma,beta,alpha), 'xyz').
        """
        enc = encoding
        # All rotations must either be intrinsic or extrinsic
        if not (enc.upper() == enc or enc.lower() == enc):
            raise utils.Error(
                'Rotation encoding must either be all intrinsic or extrinsic!')
        o = Orientation()
        for r, a in zip(encoding, angles):
            cls._euler_encoding_map[r](o, a)
        return o

    new_euler = new_from_euler

    def to_euler(self, encoding):
        """The Euler angles 'encoding' follow the documentation for
        the factory method 'new_euler'. The routine is taken from Ken
        Shoemake's chapter 'Euler Angle Conversion' in 'Graphics Gems
        IV', Academic Press, 1994, ISBN 0-12-336155-9.
        """
        enc = encoding
        # All rotations must either be intrinsic or extrinsic
        if enc.upper() == enc:
            intrinsic = True
        elif enc.lower() == enc:
            intrinsic = False
        else:
            raise utils.Error(
                'Rotation encoding must either be all intrinsic or extrinsic!')
        lenc = enc.lower()
        repetition = lenc[0] == lenc[2]
        if intrinsic:
            parity = lenc[1:] not in ('yx', 'zy', 'xz')
        else:
            parity = lenc[:2] not in ('xy', 'yz', 'zx')
        inner = lenc[2] if intrinsic else lenc[0]
        i = 'xyz'.index(inner)
        j = (i + 1 + parity) % 3
        k = (i + 2 - parity) % 3
        # h = k if repetition else i
        m = self._data
        if repetition:
            sy = np.sqrt(m[i, j]**2 + m[i, k]**2)
            if sy > utils.eps:
                ax = np.arctan2(m[i, j], m[i, k])
                ay = np.arctan2(sy, m[i, i])
                az = np.arctan2(m[j, i], -m[k, i])
            else:
                ax = np.arctan2(-m[j, k], m[j, j])
                ay = np.arctan2(sy, m[i, i])
                az = 0.0
        else:  # not repetition
            cy = np.sqrt(m[i, i]**2 + m[j, i]**2)
            if cy > utils.eps:
                ax = np.arctan2(m[k, j], m[k, k])
                ay = np.arctan2(-m[k, i], cy)
                az = np.arctan2(m[j, i], m[i, i])
            else:
                ax = np.arctan2(-m[j, k], m[j, j])
                ay = np.arctan2(-m[k, i], cy)
                az = 0.0
        if parity:
            ax, ay, az = -ax, -ay, -az
        if intrinsic:
            ax, az = az, ax
        return np.array((ax, ay, az))

    @classmethod
    def new_from_axis_angle(self, axis, angle):
        """Create and return a new orientation that rotates the given 'angle'
        (scalar) around the given 'axis' (Vector).
        """
        if type(axis) != FreeVector:
            axis = FreeVector(axis)
        oaa = Orientation()
        oaa.axis_angle = (axis, angle)
        return oaa

    new_axis_angle = new_from_axis_angle

    @classmethod
    def new_from_rotation_vector(self, rotation_vector):
        """Create and return a new orientation that represents the given
        'rotation_vector', given as a sequence of three numbers.
        """
        return Orientation(rotation_vector)

    new_rotation_vector = new_from_rotation_vector


def _test():
    o = Orientation()
    r = Orientation()
    o.from_xy(Vector(1, 1, 0), FreeVector(-1, 1, 0))
    r.set_to_z_rotation(np.pi / 2)
    ro = r * o
    print(ro.ang_dist(r))
    print(ro.axis_angle)


def _test_to_euler():
    ang = (0.1, 0.5, 0.2)
    enc = 'ZYZ'
    o = Orientation.new_euler(ang, enc)
    o_ang = o.to_euler(enc)
    oo = Orientation.new_euler(o_ang, enc)
    oo_ang = oo.to_euler(enc)
    print(np.allclose(o_ang, oo_ang))


def _test_from_nn():
    o = Orientation.new_from_xy((1, 0, 0), (1, 1, 0))
    if o.vec_y != FreeVector.ey or o.vec_x != FreeVector.ex:
        print('Test-Error in new_from_xy')
    o = Orientation.new_from_xz((1, 0, 0), (1, 0, 1))
    if o.vec_z != FreeVector.ez or o.vec_x != FreeVector.ex:
        print('Test-Error in new_from_xz')


def _test_array_property():
    o = Orientation()
    try:
        o.array = [[1, 1, 1], [0, 0, 0], [2]]
    except utils.Error as e:
        print(e)
    o.array = np.arange(9).reshape((3, 3))
    print(o.array)
    print(o.repr_error)
    o.orthonormalize()
    print(o.repr_error)


# def _test_vectorized_multiplication():
#     # Test multiplication of a list
#     o = Orientation.new_rot_z(np.pi/2)
#     vs = [Vector(1, 0, 0), FreeVector(0, 1, 0)]
#     rs = o * vs
#     assert(rs[0] == o * vs[0])
#     assert(rs[1] == o * vs[1])
