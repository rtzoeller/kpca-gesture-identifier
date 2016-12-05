from __future__ import division, print_function

import numpy as np
import six
from six.moves import range

from Point import Point

SAMPLES = 26


def linear(points):
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
    "default": linear,
    "linear": linear,
}


def normalizeNumpyArray(npArray, strategy="default"):
    if isinstance(strategy, six.string_types):
        strategy = interpolation_strategies[strategy]

    trajectory = [Point(x, y, t) for x, y, t in npArray]
    normalizedTrajectory = strategy(trajectory)
    normalizedNumpyTrajectory = np.array(
        [[p.x for p in normalizedTrajectory], [p.y for p in normalizedTrajectory]]).T
    return normalizedNumpyTrajectory
