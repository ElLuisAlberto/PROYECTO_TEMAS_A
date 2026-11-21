"""Integrate a base-frame Cartesian twist for local inverse kinematics."""
import math


def integrate_twist(current, linear, angular, duration):
    vector = tuple(value * duration for value in angular)
    angle = math.hypot(*vector)
    if angle < 1e-12:
        delta = ((1., 0., 0.), (0., 1., 0.), (0., 0., 1.))
    else:
        x, y, z = (value / angle for value in vector)
        c, s, k = math.cos(angle), math.sin(angle), 1.0 - math.cos(angle)
        delta = ((c+x*x*k, x*y*k-z*s, x*z*k+y*s),
                 (y*x*k+z*s, c+y*y*k, y*z*k-x*s),
                 (z*x*k-y*s, z*y*k+x*s, c+z*z*k))
    result = [[0.0] * 4 for _ in range(4)]
    for i in range(3):
        for j in range(3):
            result[i][j] = sum(delta[i][k] * current[k][j] for k in range(3))
        result[i][3] = current[i][3] + linear[i] * duration
    result[3][3] = 1.0
    return result
