import numpy as np
from colour import Color
from data_shape import IDX
import matplotlib.pyplot as plt


"""
raw:
    [[Cr x y manip],
     [Cr x y manip], ...]
"""

class ColorGradient:
    def __init__(self, resolution=100):
        low_color = Color("red")
        high_color = Color("blue")

        self.size = resolution
        self.colors = list(low_color.range_to(high_color, self.size))

    def get(self, value):
        """value: 0.0 ~ 1.0"""
        i = int(value * self.size)
        if i >= self.size:
            i = self.size - 1
        r, g, b = self.colors[i].rgb
        return (r, g, b, 1.0)


def scale_remapping(value, max_value, min_value, max_target, min_target):
    gain = (value - min_value) / (max_value - min_value)
    return min_target + gain * (max_target - min_target)


def filtering(raw):
    return raw[:, [IDX["Y"], IDX["TCP_X"], IDX["TCP_Y"], IDX["M"]]]


def get_colors(ms):
    max_manip = np.max(ms)
    min_manip = np.min(ms)
    grad = ColorGradient()
    return np.array(
        [grad.get(scale_remapping(m, max_manip, min_manip, 1, 0)) for m in ms]
    )


def scatter_2d(ax, raw, xyMinMax, title, is_irm=True, dot_size=1):
    """
    raw:
        [[Cr x y manip],
         [Cr x y manip], ...]
         deg m m -

    Example:
        fig, (ax1, ax2) = plt.subplots(nrows=1, ncols=2)
        xyminmax = [-0.8, 0.8, -0.8, 0.8]
        scatter_2d(ax1, filtering(rm), xyminmax)
        scatter_2d(ax2, filtering(irm), xyminmax)
        fig.tight_layout()
        plt.show()
    """
    Crs, xs, ys, ms = np.transpose(raw)
    colors = get_colors(ms)

    ax.scatter(xs, ys, c=colors, s=dot_size)

    max_Cr = np.max(Crs)
    min_Cr = np.min(Crs)
    ax.set_title("%s ($C_{r}$ range: %.1f~%.1f deg)" % (title, min_Cr, max_Cr))
    ax.set_xlabel("x(m)", fontsize=15)
    ax.set_ylabel("y(m)", fontsize=15)
    ax.axis(xyMinMax)
    ax.grid(True)

    # robot and object
    if is_irm:  # object
        marker_x = [1.0, -0.5, -0.5, -0.5]
        marker_y = [0.0, 0.0, -0.5, 0.5]
    else:  # robot
        marker_x = [1.0, -0.5, -0.5, 1.0]
        marker_y = [0.0, 0.5, -0.5, 0.0]
    arrow_size = max(xyMinMax) * 0.1
    arrow_x = np.array(marker_x) * arrow_size
    arrow_y = np.array(marker_y) * arrow_size
    ax.plot(arrow_x, arrow_y)


def scatter_3d(ax, raw, xyMinMax, title, is_irm=True, dot_size=2):
    """
    raw:
        [[Cr x y manip],
         [Cr x y manip], ...]
         deg m m -

    Example:
        fig = plt.figure()
        ax1 = fig.add_subplot(121, projection='3d')
        ax2 = fig.add_subplot(122, projection='3d')
        xyminmax = [-0.8, 0.8, -0.8, 0.8]
        scatter_3d(ax1, filtering(rm), xyminmax)
        scatter_3d(ax2, filtering(irm), xyminmax)
        plt.show()
    """
    Crs, xs, ys, ms = np.transpose(raw)
    colors = get_colors(ms)

    ax.scatter(xs, ys, zs=Crs, c=colors, s=dot_size)
    ax.set_xlabel("x(m)", fontsize=15)
    ax.set_ylabel("y(m)", fontsize=15)
    ax.set_zlabel("$C_{r}$(deg)", fontsize=15)
    ax.axis(xyMinMax)

    max_Cr = np.max(Crs)
    min_Cr = np.min(Crs)
    ax.set_title("%s ($C_{r}$ range: %.1f~%.1f deg)" % (title, min_Cr, max_Cr))

    # robot and object
    if is_irm:  # object
        marker_x = [1.0, -0.5, -0.5, -0.5]
        marker_y = [0.0, 0.0, -0.5, 0.5]
    else:  # robot
        marker_x = [1.0, -0.5, -0.5, 1.0]
        marker_y = [0.0, 0.5, -0.5, 0.0]
    arrow_size = max(xyMinMax) * 0.1
    arrow_x = np.array(marker_x) * arrow_size
    arrow_y = np.array(marker_y) * arrow_size

    int_Crs = set(np.array(Crs, dtype=int))
    for c in int_Crs:
        arrow_z = np.array([c, c, c, c])
        ax.plot(arrow_x, arrow_y, arrow_z)
