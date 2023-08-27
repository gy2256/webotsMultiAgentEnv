
import numpy as np
from scipy import interpolate


class Spline2D:

    def __init__(self, x, y, kind="cubic"):
        self.s = self.__calc_s(x, y)
        self.sx = interpolate.interp1d(self.s, x, kind=kind)
        self.sy = interpolate.interp1d(self.s, y, kind=kind)

    def __calc_s(self, x, y):
        self.ds = np.hypot(np.diff(x), np.diff(y))
        s = [0.0]
        s.extend(np.cumsum(self.ds))
        return s

    def calc_position(self, s):
        x = self.sx(s)
        y = self.sy(s)
        return x, y
