from abc import ABCMeta, abstractmethod

import numpy as np
from basic_math import rotation_2d_about_z


class CollisionBase:
    """
    The followings must be inherited.:
    * _calc_vertices
    * set_offset
    * _pseudo_check
    * _detail_check
    """

    __metaclass__ = ABCMeta

    def __init__(self, center, rad):
        self._center = np.array(center)
        self._radian = rad
        self.R = rotation_2d_about_z(self._radian)
        self.inv_R = rotation_2d_about_z(-self._radian)

        # vertices: [[x, ...], [y, ...]]
        self.vertices = None
        self.offsets = None

    @abstractmethod
    def _calc_vertices(self):
        pass

    @abstractmethod
    def set_offset(self, robot_radius):
        pass

    def check(self, xy):
        if self._pseudo_check(xy):
            return self._detail_check(xy)
        return False

    @abstractmethod
    def _pseudo_check(xy):
        pass

    @abstractmethod
    def _detail_check(xy):
        pass


class CollisionBox(CollisionBase):
    def __init__(self, center, rad, width, length):
        """
                       | y, length(size_y)
                       |
           ------------- x, width(size_x)
        """
        super(CollisionBox, self).__init__(center, rad)
        self._half_width = width / 2.0
        self._half_length = length / 2.0
        self._calc_vertices()

        self.minx, self.maxx, self.miny, self.maxy = (None, None, None, None)
        self.offset_W = None
        self.offset_L = None

    def _calc_vertices(self):
        hW = self._half_width
        hL = self._half_length
        xy = np.array([[-hW, -hW, hW, hW], [hL, -hL, -hL, hL]])
        vertices = np.matmul(self.R, xy)
        vertices[0] += self._center[0]
        vertices[1] += self._center[1]
        self.vertices = vertices

    def set_offset(self, robot_radius):
        self.offset_W = W = self._half_width + robot_radius
        self.offset_L = L = self._half_length + robot_radius
        xy = np.array([[-W, -W, W, W], [L, -L, -L, L]])
        offsets = np.matmul(self.R, xy)
        offsets[0] += self._center[0]
        self.minx = min(offsets[0])
        self.maxx = max(offsets[0])
        offsets[1] += self._center[1]
        self.miny = min(offsets[1])
        self.maxy = max(offsets[1])
        self.offsets = offsets.copy()

    def _pseudo_check(self, xy):
        x, y = xy
        if (self.minx <= x) and (x <= self.maxx):
            return (self.miny <= y) and (y <= self.maxy)
        return False

    def _detail_check(self, xy):
        local_xy = np.matmul(self.inv_R, xy - self._center)
        x, y = local_xy
        return (abs(x) <= self.offset_W) and (abs(y) <= self.offset_L)
