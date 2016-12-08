from __future__ import division, print_function

import numpy as np
import math
import six
from six.moves import range
from scipy.interpolate import CubicSpline

from Point import Point

SAMPLES = 26


def linear(points, scale_uniform=False):
    # Rescale time to [0.0, 1.0]
    min_time = points[0].t
    max_time = points[-1].t
    resampled = [Point(p.x, p.y, (p.t - min_time) / (max_time - min_time)) for p in points]

    # Interpolate between points
    interpolated = []
    for i in range(0, SAMPLES):
        effective_time = i / (SAMPLES - 1)
        for j in range(len(resampled)):
            if resampled[j + 1].t < effective_time:
                continue

            x = resampled[j].x + (effective_time - resampled[j].t) \
                                 * ((resampled[j + 1].x - resampled[j].x) / (resampled[j + 1].t - resampled[j].t))
            y = resampled[j].y + (effective_time - resampled[j].t) \
                                 * ((resampled[j + 1].y - resampled[j].y) / (resampled[j + 1].t - resampled[j].t))

            interpolated.append(Point(x, y, effective_time))
            break

    # Remap points physically
    x_mean = 0
    y_mean = 0
    for p in interpolated:
        x_mean += p.x
        y_mean += p.y
    x_mean /= len(interpolated)
    y_mean /= len(interpolated)
    x_max = max([p.x for p in points])
    y_max = max([p.y for p in points])
    x_min = min([p.x for p in points])
    y_min = min([p.y for p in points])

    remapped = []
    if scale_uniform:
        scale = max(x_max - x_min, y_max - y_min)
        for p in interpolated:
            if scale != 0:
                x = 2 * (p.x - x_mean) / scale
                y = 2 * (p.y - y_mean) / scale
            else:
                x = x_max
                y = y_max
            remapped.append(Point(x, y, p.t))
    else:
        for p in interpolated:
            if x_max != x_min:
                x = 2 * (p.x - x_mean) / (x_max - x_min)
            else:
                x = x_max
            if y_max != y_min:
                y = 2 * (p.y - y_mean) / (y_max - y_min)
            else:
                y = y_max
            remapped.append(Point(x, y, p.t))

    return remapped


def linear_time_invariant(points, scale_uniform=False):
    def distance(x1, y1, x2, y2):
        return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

    # Rescale time to [0.0, 1.0]
    min_time = points[0].t
    max_time = points[-1].t
    resampled = [Point(p.x, p.y, (p.t - min_time) / (max_time - min_time)) for p in points]

    total_distance = 0.0
    for i in range(0, len(resampled) - 1):
        total_distance += distance(points[i].x, points[i].y, points[i + 1].x, points[i + 1].y)

    interpolated = []
    for i in range(0, SAMPLES):
        target_distance = total_distance * (i / (SAMPLES - 1))
        accumulated_distance = 0.0
        for j in range(len(resampled) - 1):
            local_distance = distance(points[j].x, points[j].y, points[j + 1].x, points[j + 1].y)
            if accumulated_distance + local_distance >= target_distance:
                if local_distance == 0:
                    # The trajectory did not move, so we can just take the first point
                    x = points[j].x
                    y = points[j].y
                else:
                    location = (target_distance - accumulated_distance) / local_distance
                    if location > 1.0:
                        # Floating point precision problems
                        assert (location <= 1.0001)
                        location = 1.0

                    x = location * (points[j + 1].x - points[j].x) + points[j].x
                    y = location * (points[j + 1].y - points[j].y) + points[j].y
                interpolated.append(Point(x, y, target_distance))
                break
            else:
                accumulated_distance += local_distance

    # Remap points physically
    x_mean = 0
    y_mean = 0
    for p in interpolated:
        x_mean += p.x
        y_mean += p.y
    x_mean /= len(interpolated)
    y_mean /= len(interpolated)
    x_max = max([p.x for p in points])
    y_max = max([p.y for p in points])
    x_min = min([p.x for p in points])
    y_min = min([p.y for p in points])

    remapped = []
    if scale_uniform:
        scale = max(x_max - x_min, y_max - y_min)
        for p in interpolated:
            if scale != 0:
                x = 2 * (p.x - x_mean) / scale
                y = 2 * (p.y - y_mean) / scale
            else:
                x = x_max
                y = y_max
            remapped.append(Point(x, y, p.t))
    else:
        for p in interpolated:
            if x_max != x_min:
                x = 2 * (p.x - x_mean) / (x_max - x_min)
            else:
                x = x_max
            if y_max != y_min:
                y = 2 * (p.y - y_mean) / (y_max - y_min)
            else:
                y = y_max
            remapped.append(Point(x, y, p.t))

    return remapped


def cubic_spline(points, scale_uniform=False, bc_type="not-a-knot"):
    # Rescale time to [0.0, 1.0]
    min_time = points[0].t
    max_time = points[-1].t
    resampled = [Point(p.x, p.y, (p.t - min_time) / (max_time - min_time)) for p in points]

    # Interpolate SAMPLES points using cubic splines
    t = np.linspace(0, 1, len(resampled))
    xSpline = CubicSpline(t, [p.x for p in resampled], bc_type=bc_type)
    ySpline = CubicSpline(t, [p.y for p in resampled], bc_type=bc_type)

    interpolated = []
    for i in range(0, SAMPLES):
        effective_time = i / (SAMPLES - 1)
        interpolated.append(Point(xSpline(effective_time), ySpline(effective_time), effective_time))

    # Remap points physically
    x_mean = 0
    y_mean = 0
    for p in interpolated:
        x_mean += p.x
        y_mean += p.y
    x_mean /= len(interpolated)
    y_mean /= len(interpolated)
    x_max = max([p.x for p in points])
    y_max = max([p.y for p in points])
    x_min = min([p.x for p in points])
    y_min = min([p.y for p in points])

    remapped = []
    if scale_uniform:
        scale = max(x_max - x_min, y_max - y_min)
        for p in interpolated:
            if scale != 0:
                x = 2 * (p.x - x_mean) / scale
                y = 2 * (p.y - y_mean) / scale
            else:
                x = x_max
                y = y_max
            remapped.append(Point(x, y, p.t))
    else:
        for p in interpolated:
            if x_max != x_min:
                x = 2 * (p.x - x_mean) / (x_max - x_min)
            else:
                x = x_max
            if y_max != y_min:
                y = 2 * (p.y - y_mean) / (y_max - y_min)
            else:
                y = y_max
            remapped.append(Point(x, y, p.t))

    return remapped


interpolation_strategies = {
    "default": linear_time_invariant,
    "linear": linear,
    "linear_time_invariant": linear_time_invariant,
    "linear_scale_uniform": lambda p: linear(p, True),
    "linear_time_invariant_scale_uniform": lambda p: linear_time_invariant(p, True),
    "cubic_spline": cubic_spline,
    "cubic_spline_scale_uniform": lambda p: cubic_spline(p, True),
    "none": lambda p: p,
}


def normalizeNumpyArray(npArray, strategy="default"):
    if isinstance(strategy, six.string_types):
        strategy = interpolation_strategies[strategy]

    trajectory = [Point(x, y, t) for x, y, t in npArray]
    normalizedTrajectory = strategy(trajectory)
    normalizedNumpyTrajectory = np.array(
        [[p.x for p in normalizedTrajectory], [p.y for p in normalizedTrajectory]]).T
    return normalizedNumpyTrajectory
