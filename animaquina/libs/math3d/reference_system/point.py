"""
Module for class Point in the ReferenceSystem class.
"""

__author__ = "Morten Lind"
__copyright__ = "Morten Lind 2012"
__credits__ = ["Morten Lind"]
__license__ = "LGPLv3"
__maintainer__ = "Morten Lind"
__email__ = "morten@lind.fairuse.org"
__status__ = "Development"

import numpy as np

from ..vector import Vector
#from .ref_sys_object import RefSysObject

class Point(object): 
    """A point in the reference system is identified by a label name,
    and a vector with reference to a specific reference frame in the
    reference system. The vector representing the point is to be
    considered a position vector, and must transform as such. The
    reference system will be able to express the point in any
    registered frame of reference."""

    def __init__(self, name, root_frame=None, pos_vec=None, by_ref=True):
        """ Create a new point with given 'name', defined in
        'root_frame' and with position vector 'pos_vec'. 'root_frame'
        and 'pos_vec' may be None at creation time. If None, the
        position vector will be initialized to the zero vector. At
        time of registering in a reference system instance, the
        reference frame must be filled in to make sense. If 'by_ref'
        is True, the default, then the given object is stored by
        reference; which is practical for external, implicit update."""
        self._name = name
        self._root_frame = root_frame
        if pos_vec is None:
            pos_vec = Vector()
        self._by_ref = by_ref
        if by_ref:
            self._pos_vec = pos_vec
        else:
            self._pos_vec = pos_vec.copy()

    @property
    def pos_vec(self):
        """Give access to the position vector in the natural frame of
        reference."""
        return self._pos_vec

    def __repr__(self):
        return (
            'Point: "{self._name}" in frame "{self._root_frame}" ' 
            'with vector {self._pos_vec}').format(self=self)
