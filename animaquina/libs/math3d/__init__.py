"""
Module for top level imports in PyMath3D.
"""

__author__ = "Morten Lind"
__copyright__ = "Morten Lind 2012-2021"
__credits__ = ["Morten Lind"]
__license__ = "LGPLv3"
__maintainer__ = "Morten Lind"
__email__ = "morten@lind.fairuse.org"
__status__ = "Production"

__version__ = "4.0-dev"


from .utils import set_precision
from .quaternion import Quaternion, Versor
from .orientation import Orientation, RotationVector
from .orientation_computer import OrientationComputer
from .vector import Vector, FreeVector, PositionVector
from .transform import Transform, PoseVector
