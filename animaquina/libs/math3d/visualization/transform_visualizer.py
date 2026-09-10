# coding=utf-8

"""
Rudimentary visualization of transforms using Matplotlib.
"""

__author__ = "Morten Lind"
__copyright__ = "Morten Lind 2021"
__credits__ = ["Morten Lind"]
__license__ = "LGPLv3"
__maintainer__ = "Morten Lind"
__email__ = "morten@lind.fairuse.org"
__status__ = "Development"


import numpy as np
import matplotlib.pyplot as plt
import math3d as m3d


class TransformVisualizer:

    def __init__(self,
                 uvec_length: float = 1.0,
                 plot_identity: bool = False,
                 identity_label: str = None):
        self._uvec_length = uvec_length
        self._plot_identity = plot_identity
        self._identity_label = identity_label
        self.clear()

    def clear(self):
        self._fig = plt.figure()
        self._ax = self._fig.add_subplot(111, projection='3d')
        self._points = np.array([[0, 0, 0]])
        if self._plot_identity:
            self.plot(m3d.Transform(),
                      uvec_length=self._uvec_length,
                      label=self._identity_label)

    def plot(self,
             t: m3d.Transform,
             uvec_length: float = None,
             label: str = None):
        if uvec_length is None:
            uvec_length = self._uvec_length
        t = m3d.Transform(t)
        p = t.pos
        parr = p.array
        o = t.orient
        for uvec, col in zip((o.vec_x.array, o.vec_y.array, o.vec_z.array),
                             'rgb'):
            print(uvec_length, uvec)
            tip = parr + uvec_length * uvec
            points = np.vstack((parr, tip))
            self._points = np.vstack((self._points, points))
            self._ax.plot(*points.T, color=col)
        if label is not None:
            self._ax.text(*(parr + uvec_length * uvec), ' ' + label)

    def show(self, block=False):
        plt.plot(*np.transpose([
            3*[self._points.min()],
            3*[self._points.max()]]), alpha=0)
        plt.show(block=block)


def _test():
    tv = TransformVisualizer(plot_identity=True, identity_label='Base')
    # tv.plot(m3d.Transform(), label='Base')
    tv.plot(m3d.Transform((np.pi/3, 0, 0), (1, 0, 2)),
            uvec_length=0.5,
            label='Transformed')
    tv.show()
