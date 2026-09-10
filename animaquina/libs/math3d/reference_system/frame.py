"""
Module for class Frame in the ReferenceSystem class.
"""

__author__ = "Morten Lind"
__copyright__ = "Morten Lind 2012"
__credits__ = ["Morten Lind"]
__license__ = "LGPLv3"
__maintainer__ = "Morten Lind"
__email__ = "morten@lind.fairuse.org"
__status__ = "Development"

import numpy as np

from ..transform import Transform

class Frame(object):
    """A frame in the reference system is identified by a label name,
    a root frame, and the transform which represents the frame in its
    root."""
    
    def __init__(self, name, root_frame=None, xform=None, by_ref=True):
        """Initialize a frame by a 'name', a 'root_frame' (defaults to
        None) and a transform (defaults to the identity). If 'xform'
        is not of class m3d.Transform, it is supposed to evaluate to
        one by access to an attribute 'xform'. If 'by_ref' is True,
        the default, then the given object is stored by reference;
        which is practical for external, implicit update. In case of
        by value semantic (if 'by_ref==False') the given xform object
        must have a valid '.copy()' method."""
        self._name = name
        self._root_frame = root_frame
        ## The transform from this to root coordinates, i.e. 'this in root'
        if xform is None:
            xform = Transform()
        if type(xform) is not Transform:
            self._volatile = True
        else:
            self._volatile = False
        self._by_ref = by_ref
        if by_ref:
            self._xform = xform
        else:
            self._xform = xform.copy()

    @property
    def xform(self):
        """Give access to the fundamental transform which represents
        this frame in its root frame."""
        if self._volatile:
            return self._xform.xform
        else:
            return self._xform

    @property
    def name(self):
        """The name of this frame."""
        return self._name

    @property
    def root_frame(self):
        return self._root_frame

    def __repr__(self):
        if self._name == 'world':
            return 'Frame: "world"'
        else:
            return (
                'Frame: "{self.name}" in frame "{self.root_frame.name}" with pose vector '
                '{self.xform.pose_vector}'
                ).format(self=self)

