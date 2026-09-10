# coding=utf-8

"""
Module implementing a 3D homogenous Transform class. The transform is
represented internally by associated orientation and a vector objects.
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
from .vector import Vector, FreeVector, PositionVector
from .orientation import Orientation, RotationVector


class PoseVector:  # (np.ndarray):

    class Error(Exception):
        def __init__(self, message):
            self.message = message
            Exception.__init__(self, self.message)

        def __repr__(self):
            return self.__class__.__qualname__ + ': ' + self.message

    def __init__(self, *args, **kw_args):
        self._data = np.zeros(6)
        if len(args) > 0:
            # args take precedence
            if len(args) == 1:
                # Copy constructor
                if type(args[0]) == PoseVector:
                    self._data[:] = args[0].array
                # Use a Transform object
                elif type(args[0]) == Transform:
                    self._data[:] = args[0].pose_vector.array
                # Use six numbers
                elif len(args[0]) == 6:
                    self._data[:] = args[0]
                else:
                    raise self.Error('Could not construct on one positional '
                                     + 'argument {args[0]}')
            elif len(args) == 2:
                if len(args[0]) != 3 or len(args[1]) != 3:
                    raise self.Error('Given two positional argument which ' +
                                     f'were not of length three: "{args[0]}" , "{args[1]}"')
                else:
                    # Separate position and orientation
                    utils._deprecation_warning(
                        'Constructing pose vector on two 3-arrays in '
                        + 'positional arguments. Use keyword arguments '
                        + '"pos" and "rot" instead')
                    self._data[:3] = args[0]
                    self._data[3:] = args[1]
            elif len(args) == 6:
                self._data[:] = args
        elif 'pos' in kw_args or 'rot' in kw_args:
            if 'pos' in kw_args:
                self._data[:3] = PositionVector(kw_args['pos']).array
            if 'rot' in kw_args:
                self._data[3:] = RotationVector(kw_args['rot']).array
        elif len(args) == 0:
            # Default constructor
            pass
        else:
            raise self.Error(f'Can not create on {args} or {kw_args}')
        self._pos = PositionVector()
        self._pos._data = self._data[:3]
        self._rot = RotationVector()
        self._rot._data = self._data[3:]

    def copy(self):
        return self.__class__(self._data.copy())

    def __repr__(self):
        # return (f'<PoseVec:\n{repr(self.rot_vec)}\n{repr(self.pos)}\n>')
        rvr = self.rot_vec.array_ref
        pr = self.pos.array_ref
        return (f'<PoseVec: P({pr[0]:.3f}, {pr[1]:.3f}, {pr[2]:.3f})' +
                f' RV({rvr[0]:.3f}, {rvr[1]:.3f}, {rvr[2]:.3f})')
    
    def __eq__(self, other):
        if type(other) == PoseVector:
            return np.sum((self._data - other._data) ** 2) < utils.eps
        else:
            return NotImplemented

    def __mul__(self, other):
        if np.isscalar(other):
            return self.__class__(other * self._data)

    def __rmul__(self, other):
        if np.isscalar(other):
            return self.__class__(other * self._data)

    def __imul__(self, other):
        if np.isscalar(other):
            self._data *= other

    def __truediv__(self, other):
        if np.isscalar(other):
            return self.__class__(self._data / other)

    def __idiv__(self, other):
        if np.isscalar(other):
            self._data /= other

    def get_array(self):
        """Return a copy of the ndarray which is the fundamental data
        of the PoseVector."""
        return self._data.copy()

    def set_array(self, array, check=True):
        """Set the vector by six values in the iterable 'array'.
        """
        if check and len(array) != 6:
            raise self.Error(
                'Setting the value by the "array" property needs exactly'
                + ' three values. ({} were given)'.format(len(array)))
        self._data[:] = array

    array = property(get_array, set_array)

    def get_rotation_vector(self) -> RotationVector:
        return self._rot.copy()

    rotation_vector = rot_vec = rv = property(get_rotation_vector)

    def get_orientation(self):
        return Orientation(self._rot)

    orientation = orient = ori = o = property(get_orientation)

    def get_position(self):
        return self._pos.copy()

    position = pos = p = property(get_position)

    def get_exponential(self):
        return Transform(self)

    exponential = exp = property(get_exponential)
    transform = transf = tr = exponential


class Transform(object):
    """A Transform is a member of SE(3), represented as a homogenous
    transformation matrix. It uses an Orientation in member '_o'
    (accessible through 'orient') to represent the orientation part
    and a Vector in member '_v' (accessible through 'pos') to
    represent the position part.
    """

    # A set of acceptable multi-value types for entering data.
    __value_types = (np.ndarray, list, tuple)

    class Error(Exception):
        """Exception class."""

        def __init__(self, message):
            self.message = message
            Exception.__init__(self, self.message)

        def __repr__(self):
            return self.message

    def __create_on_sequence(self, arg):
        """Called from init when a single argument of sequence type was given
        the constructor.
        """
        # if len(arg) == 1 and utils.is_sequence(arg[0]):
        #     self.__createOnSequence(arg[0])
        if type(arg) in (tuple, list):
            self.__create_on_sequence(np.array(arg))
        elif type(arg) == np.ndarray and arg.shape in ((4, 4), (3, 4)):
            self._o = Orientation(arg[:3, :3])
            self._v = PositionVector(arg[:3, 3])
        elif type(arg) == np.ndarray and arg.shape == (6,):
            # Assume a pose vector of 3 position vector and 3 rotation
            # vector components
            self._v = PositionVector(arg[:3])
            self._o = Orientation(arg[3:])
        else:
            raise self.Error(
                'Could not create Transform on arguments : "' + str(arg) + '"')

    def __init__(self, *args):
        """A Transform is a homogeneous transform on SE(3), internally
        represented by an Orientation and a Vector. A Transform can be
        constructed on:

        * A Transform.

        * A PoseVector

        * A numpy array, list or tuple of shape (4,4) or (3,4) giving
          direct data; as [orient | pos].

        * A --''-- of shape (6,) giving a pose vector; concatenated
          position and rotation vector.

        * Two --''--; the first for orientation and the second for
          position.

        * Four --''--; the first three for orientation and the fourth
          for position.

        * Twelve numbers, the first nine used for orientation and the
          last three for position.

        * An ordered pair of Orientation and PositionVector.
        """
        if len(args) == 0:
            self._v = PositionVector()
            self._o = Orientation()
        elif len(args) == 1:
            arg = args[0]
            if type(arg) == Transform or (
                    hasattr(arg, 'pos') and hasattr(arg, 'orient')):
                self._v = PositionVector(arg.pos)
                self._o = Orientation(arg.orient)
            elif type(arg) == PoseVector:
                self._v = arg.position
                self._o = arg.orientation
            else:
                self.__create_on_sequence(arg)
        elif len(args) == 2:
            self._o = Orientation(args[0])
            self._v = PositionVector(args[1])
        elif len(args) == 4:
            self._o = Orientation(args[:3])
            self._v = PositionVector(args[3])
        elif len(args) == 12:
            self._o = Orientation(args[:9])
            self._v = PositionVector(args[9:])
        else:
            raise self.Error(
                'Could not create Transform on arguments : ' +
                '"{}"'.format(str(args)))
        # Guard against reference to data.
        self._from_ov(self._o, self._v)

    def _from_ov(self, o, v):
        self._data = np.identity(4, dtype=utils.flt)
        # First take over the data from Orientation and Vector
        self._data[:3, :3] = o._data
        self._data[:3, 3] = v._data
        # Then share data with Orientation and Vector.
        self._o._data = self._data[:3, :3]
        self._v._data = self._data[:3, 3]

    def get_pos(self):
        """Return a reference (Beware!) to the position object."""
        return self._v

    def set_pos(self, new_pos):
        """Set the position."""
        if type(new_pos) in self.__value_types:
            self._data[:3, 3] = new_pos
        elif type(new_pos) in [Vector, PositionVector]:
            self._data[:3, 3] = new_pos._data
        else:
            raise self.Error('Trying to set "pos" by an object of ' +
                             'type "{}". '.format(str(type(new_pos))) +
                             'Needs tuple, list, ndarray, or Vector.')
        self._v._data = self._data[:3, 3]

    pos = p = property(get_pos, set_pos)

    def get_orient(self):
        """Return a reference (Beware!) to the orientation object."""
        return self._o

    def set_orient(self, new_orient):
        """Set the orientation."""
        if type(new_orient) in self.__value_types:
            self._data[:3, :3] = new_orient
        elif type(new_orient) == Orientation:
            self._data[:3, :3] = new_orient._data
        else:
            raise self.Error('Trying to set "orient" by an object of ' +
                             'type "{}". '.format(str(type(new_orient))) +
                             'Needs tuple, list, ndarray, or Orientation.')
        self._o._data = self._data[:3, :3]

    orientation = orient = ori = o = property(get_orient, set_orient)

    def __copy__(self):
        """Copy method for creating a (deep) copy of this
        Transform.
        """
        return Transform(self)

    def __deepcopy__(self, memo):
        return self.__copy__()

    def copy(self, other=None):
        """Copy data from 'other' to self. If no argument given,
        i.e. 'other==None', return a copy of this Transform.
        """
        if other is None:
            return Transform(self)
        else:
            self._data[:, :] = other._data

    def __repr__(self):
        # return (f'<Transform:\n{repr(self.orient)}\n{repr(self.pos)}\n>')
        rvr = self.orient.rot_vec.array_ref
        pr = self.pos.array_ref
        return (f'<Transform: RV({rvr[0]:.3f}, {rvr[1]:.3f}, {rvr[2]:.3f})' +
                f' P({pr[0]:.3f}, {pr[1]:.3f}, {pr[2]:.3f})>')
    
    def __str__(self):
        return self.__repr__()

    def __eq__(self, other):
        if type(other) == Transform:
            return np.sum((self._data - other._data) ** 2) < utils.eps
        else:
            return NotImplemented

    def from_xyp(self, vec_x, vec_y, origo):
        """Make this transform correspond to the orientation given by the
        given 'vec_x' and 'vec_y' directions and translation given by
        'origo'.
        """
        self._o.from_xy(vec_x, vec_y)
        self._v = origo
        self._from_ov(self._o, self._v)

    def from_xzp(self, vec_x, vec_z, origo):
        """Make this transform correspond to the orientation given by the
        given 'vec_x' and 'vec_z' directions and translation given by
        'p'.
        """
        self._o.from_xz(vec_x, vec_z)
        self._v = origo
        self._from_ov(self._o, self._v)

    def from_yzp(self, vec_y, vec_z, origo):
        """Make this transform correspond to the orientation given by the
        given 'vec_y' and 'vec_z' directions and translation given by
        'p'.
        """
        self._o.from_yz(vec_y, vec_z)
        self._v = origo
        self._from_ov(self._o, self._v)

    def dist_squared(self, other):
        """Return the square of the metric distance, as the unweighted sum of
        linear and angular distance, to the 'other' transform. Note
        that the units and scale among linear and angular
        representations matters heavily.
        """
        return self._v.dist_squared(other._v) + self._o.ang_dist(other._o) ** 2

    def dist(self, other):
        """Return the metric distance, as unweighted combined linear and
        angular distance, to the 'other' transform. Note that the
        units and scale among linear and angular representations
        matters heavily.
        """
        return np.sqrt(self.dist_squared(other))

    def get_inverse(self):
        """Return an inverse of this Transform."""
        return Transform(np.linalg.inv(self._data))

    inverse = property(get_inverse)

    def invert(self):
        """In-place invert this Transform."""
        self._data[:, :] = np.linalg.inv(self._data)

    def __matmul__(self, other):
        """Multiplication of self with another Transform or operate on a
        Vector given by 'other'. 'other' may be a Transform, a
        FreeVector, a PositionVector, a PoseVector, a RotationVector,
        an 3-ndarray, or an 3xN-ndarray.
        """
        if type(other) == Transform:
            return Transform(self._data @ other._data)
        elif isinstance(other, Vector):
            if type(other) in [Vector, FreeVector]:
                return FreeVector(self._data[:3, :3] @ other._data)
            elif type(other) == PositionVector:
                return PositionVector(self._data[:3, :3] @ other._data
                                      + self._data[:3, 3])
            else:
                return NotImplemented
        elif type(other) == PoseVector:
            return (self * other.transform).pose_vector
        elif type(other) == RotationVector():
            return self._o * other
        elif (type(other) == np.ndarray and
              len(other.shape) in (1, 2) and
              other.shape[0] == 3):
            return (np.matmul(self._o._data, other).T + self._v._data).T
        # elif utils.is_sequence(other):
        #     # Assume a sequence of objects that may be multiplied
        #     return [self * o for o in other]
        else:
            return NotImplemented

    __mul__ = __matmul__


    def trf_vec_arr(self,
                    vec_array: np.ndarray,
                    rows: bool = True,
                    positions: bool = True) -> np.ndarray:
        """Transforms the vectors in 'vec_array'. If 'rows' is True, the
        array is considered an array of row vectors, Nx3, otherwise as
        an array of column vectors, 3xN. If 'positions' is True, the
        vectors are transformed as PositionVectors, otherwise as
        FreeVectors. The returned, transformed vector array has the
        same organization as the incoming.
        """
        if rows:
            # Work on array of column vectors
            vec_array = vec_array.T
        res = self._data[:3, :3] @ vec_array
        if positions:
            res += self._data[:3, 3:4]
        if rows:
            return res.T
        else:
            return res

    def get_logarithm(self) -> PoseVector:
        """Get the logarithm of the transform, i.e. in pose vector representation '(x, y, z, rx, ry,
        rz)'.
        """
        return PoseVector(np.append(self._v._data, self._o.rotation_vector._data))

    logarithm = log = pose_vector = pv = property(get_logarithm)

    def get_structured_array(self):
        """Return a tuple pair of an 3x3 orientation array and position as
        3-array.
        """
        return (self._data[:3, :3], self._data[:3, 3])

    structured_array = property(get_structured_array)

    def get_list_3x3_3(self):
        """Return a list with separate orientation and position in list
        form.
        """
        return [self._data[:3, :3].tolist(), self._data[:3, 3].tolist()]

    list_3x3_3 = property(get_list_3x3_3)

    def get_matrix(self):
        """Property for getting a (4,4) np-matrix with the data from the
        transform.
        """
        return np.matrix(self._data)

    matrix = property(get_matrix)

    def get_array(self):
        """Return a copy of the (4,4) ndarray which is the fundamental
        data of the Transform. Caution: Use this method only for
        optimization, since it eliminates copying, and be sure not to
        compromize the data.
        """
        return self._data.copy()

    array = property(get_array)

    def get_array_ref(self):
        """Return a reference to the (4,4) ndarray, which is the
        fundamental data of the transform.
        """
        return self._data

    array_ref = property(get_array_ref)

    def get_list_4x4(self):
        """Return the fundamental data of the Transform as a list."""
        return self._data.tolist()
    list_4x4 = property(get_list_4x4)

    @classmethod
    def new_from_xyp(self, vec_x, vec_y, origo):
        """Create a transform corresponding to the orientation given by the
        given 'vec_x' and 'vec_y' directions and translation given by
        'origo'.
        """
        t = Transform()
        t.from_xyp(vec_x, vec_y, origo)
        return t

    @classmethod
    def new_from_xzp(self, vec_x, vec_z, origo):
        """Create a transform corresponding to the orientation given by the
        given 'vec_x' and 'vec_z' directions and translation given by
        'origo'.
        """
        t = Transform()
        t.from_xzp(vec_x, vec_z, origo)
        return t

    @classmethod
    def new_from_yzp(self, vec_y, vec_z, origo):
        """Create a transform corresponding to the orientation given by the
        given 'vec_y' and 'vec_z' directions and translation given by
        'origo'.
        """
        t = Transform()
        t.from_yzp(vec_y, vec_z, origo)
        return t

    @classmethod
    def new_from_point_sets(
            self,
            ApTs: typing.Union[np.ndarray, tuple[PositionVector]],
            BpTs: typing.Union[np.ndarray, tuple[PositionVector]]):
        """Create a transform inferred from the same set of minimum three
        points, observed in two reference systems, 'A' and 'B'. Points
        are stored as row vector, hence they are denoted 'XpTs'. The
        resulting transform is denoted AB, which transforms from
        reference B to referende A. The method uses Kabsch algorithm: https://en.wikipedia.org/wiki/Kabsch_algorithm
        """
        if type(ApTs[0]) == PositionVector:
            ApTs = np.array([Ap.array for Ap in ApTs])
        if type(BpTs[0]) == PositionVector:
            BpTs = np.array([Bp.array for Bp in BpTs])
        # Calculate centered point set
        ApTc = ApTs.mean(axis=0)
        ApTcs = ApTs - ApTc
        BpTc = BpTs.mean(axis=0)
        BpTcs = BpTs - BpTc
        U, S, VT = np.linalg.svd(ApTcs.T @ BpTcs)
        # Orientation of B in A reference
        AoB = U @ VT
        # Correct for reflection
        if np.linalg.det(AoB) < 0:
            AoB = U @ np.diag([1.0, 1.0, -1]) @ VT
        # Calculate origo of B in A reference
        ApB = ApTc - AoB @ BpTc
        return Transform(AoB, ApB)


def _test():
    cy = FreeVector(1, 1, 0)
    cx = FreeVector(2, 3, 0)
    cz = FreeVector.e2
    p = PositionVector(1, 2, 3)
    t = Transform.new_from_xzp(cx, cz, p)
    t = Transform.new_from_yzp(cy, cz, p)
    print(t * cx)
    it = t.inverse
    print(t * it)


def _test_transf_vectors():
    px = PositionVector.e0
    py = PositionVector.e1
    pz = PositionVector.e2
    pp = PositionVector(1, 2, 3)
    t = Transform(Orientation.new_rot_z(np.pi), PositionVector(1, 0, 0))
    tpx = t * px
    assert(tpx == Vector(0, 0, 0))
    assert(np.all(t * px.array == tpx.array))
    tpy = t * py
    assert(tpy == Vector(1, -1, 0))
    tpz = t * pz
    assert(tpz == Vector(1, 0, 1))
    tpp = t * pp
    assert(tpp == Vector(0, -2, 3))
    pstack = np.vstack([px.array, py.array, pz.array, pp.array]).T
    tpstack = t * pstack
    stacktp = np.vstack([tpx.array, tpy.array, tpz.array, tpp.array]).T
    assert(np.all(tpstack == stacktp))


def _test_from_point_sets():
    BpTs = np.array([[1, 0, 1],
                     [1, 1, 0],
                     [0, 0, 1],
                     # [2, 0, 0]
                     ])
    AB_nom = Transform(Orientation.new_euler([0.1, 0.5, 1.2]),
                       PositionVector(1, 2, 3))
    ApTs = (AB_nom.orient.array @ BpTs.T).T + AB_nom.pos.array
    AB = Transform.new_from_point_sets(ApTs, BpTs)
    print(f'Orientation error: {AB.orient.ang_dist(AB_nom.orient)} rad')
    print(f'Position error: {AB.pos.dist(AB_nom.pos)} [length units]')


def _test_free_pos_vectors_transform():
    t = Transform(Orientation.new_rot_z(np.pi), PositionVector(1, 0, 0))
    fv = FreeVector(1, 2, 3)
    pv = PositionVector(1, 2, 3)
    print(t * fv)
    print(t * pv)


def _test_pose_vector():
    # Test == operator
    assert(PoseVector(1, 2, 3, 4, 5, 6) == PoseVector(1, 2, 3, 4, 5, 6))
    assert(PoseVector() != PoseVector(1, 2, 3, 4, 5, 6))
    # Test subtraction, which must throw an exception
    try:
        PoseVector() - PoseVector()
    except TypeError:
        print('Taking difference of pose vectors raised TypeError as expected.')
    # Test transformation
    t0 = Transform(Orientation.new_rot_z(np.pi), PositionVector(1, 0, 0))
    t1 = Transform(Orientation.new_rot_x(np.pi / 2), PositionVector(0, 0, 3))
    assert((t0 * t1).pose_vector == t0 * t1.pose_vector)


def _test_trf_vec_arr():
    # global prs, pcs, t, tprs, tpcs
    prs = vrs = np.arange(15).reshape((-1, 3))
    print(f'Row vectors:\n{prs}')
    pcs = vcs = vrs.T
    t = Transform([0, 0, np.pi/2], [1, 0, 0])
    print(f'Transform: {t}')
    tprs = t.trf_vec_arr(prs, rows=True, positions=True)
    tpcs = t.trf_vec_arr(pcs, rows=False, positions=True)
    print(f'Transformed row position vectors:\n{tprs}')
    assert(np.all(tprs.T == tpcs))
    assert(np.all(np.isclose(tprs[0], [0, 0, 2])))
    tvrs = t.trf_vec_arr(vrs, rows=True, positions=False)
    tvcs = t.trf_vec_arr(vcs, rows=False, positions=False)
    print(f'Transformed row free vectors:\n{tvrs}')
    assert(np.all(np.isclose(tvrs.T, tvcs)))

# def _test_vectorized_multiplication():
#     # Test multiplication of a list
#     t = Transform(Orientation.new_rot_z(np.pi/2), Vector(1, 0, 0))
#     vs = [Vector(1, 0, 0), Vector(0, 1, 0)]
#     rs = t * vs
#     assert(rs[0] == t * vs[0])
#     assert(rs[1] == t * vs[1])
