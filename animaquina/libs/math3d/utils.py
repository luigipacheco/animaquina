# coding=utf-8

"""
Utility function and definitions library for PyMath3D.
"""

__author__ = "Morten Lind"
__copyright__ = "Morten Lind 2009-2021"
__credits__ = ["Morten Lind"]
__license__ = "LGPLv3"
__maintainer__ = "Morten Lind"
__email__ = "morten@lind.fairuse.org"
__status__ = "Production"


import numbers
import inspect
import collections

import numpy as np


def _deprecation_warning(msg):
    f = inspect.stack()[1]
    # print(f)
    print(('math3d: {} @ {} in {}:\n\tA deprecated method was invoked. ')
          .format(f[1], f[2], f[3]) +
          'Suggestion for replacement: "{:s}"'.format(msg))


def set_precision(prec):
    """Set epsilon and float type"""
    global sqrt_eps, eps, _eps, flt
    if prec == 16:
        eps = 10 * np.finfo(np.float16).resolution
        flt = np.float16
    elif prec == 32:
        eps = 10 * np.finfo(np.float32).resolution
        flt = np.float32
    elif prec == 64:
        eps = 10 * np.finfo(np.float64).resolution
        flt = np.float64
    else:
        raise Error('Supported precision (int): 16, 32, 64.')
    sqrt_eps = np.sqrt(eps)

set_precision(64)

def is_sequence(obj):
    """Test if "obj" is a sequence."""
    return isinstance(obj, collections.abc.Iterable)


def is_three_sequence(obj):
    """Test if "obj" is of a sequence type and three long."""
    return isinstance(obj, collections.abc.Iterable) and len(obj) == 3


# Standard numeric types
_number_bases = (np.number, numbers.Number)


def is_num_type(val):
    """Test if "val" is of a number type."""
    return isinstance(val, _number_bases)


def is_num_types(lst):
    """Test if every item in "lst" is of a number type."""
    return np.all([(lambda x: isinstance(x, _number_bases))(li) for li in lst])


class Error(Exception):
    """Exception class."""
    def __init__(self, message):
        self.message = message
        Exception.__init__(self, self.message)

    def __repr__(self):
        return self.message
