# coding=utf-8

"""
Module implementing facilities for orientation computation.
"""

__author__ = "Morten Lind"
__copyright__ = "Morten Lind 2012-2017"
__credits__ = ["Morten Lind"]
__license__ = "LGPLv3"
__maintainer__ = "Morten Lind"
__email__ = "morten@lind.fairuse.org"
__status__ = "Development"


from .vector import Vector
from .orientation import Orientation

class OrientationComputer:
    """From a given orientation, compute various target orientations
    fulfilling some constraints and objectives.
    """

    @classmethod
    def align_z(cls, o_org, z_tgt, x_tgt=None):
        """Given an original orientation 'o_org', compute a target
        orientation, 'o_tgt' which is close to 'o_org', and where the
        z-direction is aligned (.*.==1) with z_tgt. If 'x_tgt' is
        given, then the x-direction is chosen mainly parallel to
        'x_tgt'. This is especially handy for computing orientations
        for a sensor which have an observation direction, z, and a
        major-direction, x. The computation is equivalent to two
        mobile frame rotations, where first the original z-axis is
        rotated minimally to the target z-axis, and then the
        intermediate x-axis is rotated minimally for being closest to
        parallel with the target x-axis.
        """
        # Make sure z_tgt is a m3d unit vector
        z_tgt = Vector(z_tgt).normalized
        if x_tgt is None and y_tgt is None:
            # No additional requirements, compute the minimal rotation
            # of the original z-direction. 
            r_z2z = Orientation.new_vec_to_vec(o_org.vec_z, z_tgt)
            return r_z2z * o_org
        if x_tgt is not None:
            # Make sure that x_tgt is a unit Vector which has no
            # projection on z_tgt
            x_tgt = Vector(x_tgt)
            x_tgt = (x_tgt - x_tgt.projection(z_tgt)).normalized
            # Form the positive and negative orientations, based on x_tgt or -x_tgt
            o_p = Orientation.new_from_xz(x_tgt, z_tgt)
            o_n = Orientation.new_from_xz(-x_tgt, z_tgt)
            # Select based on minimal rotation from the original orientation
            if o_org.ang_dist(o_p) < o_org.ang_dist(o_n):
                return o_p
            else:
                return o_n


def _test():
    o_init = Orientation()
    z_tgt = Vector([1,1,0]).normalized
    x_tgt = Vector([0,0.5,1]).normalized
    o_tgt=OrientationComputer.align_z(o_init, z_tgt, x_tgt)
    # The most important condition
    assert(o_tgt.vec_z * z_tgt == 1.0)
    # ToDo: Test requirement on x_tgt
