"""Uniform cubic B-spline shared with native PathTrajectory; no waypoint stops."""
import numpy as np


def samples(points, subdivisions=10):
    points = np.asarray(points, dtype=float)
    padded = np.vstack((points[:1], points[:1], points, points[-1:], points[-1:]))
    result = []
    for i in range(len(padded)-3):
        a,b,c,d = padded[i:i+4]
        for u in np.linspace(0,1,subdivisions,endpoint=False):
            result.append(((1-u)**3*a+(3*u**3-6*u*u+4)*b+
                           (-3*u**3+3*u*u+3*u+1)*c+u**3*d)/6)
    return np.vstack((result, points[-1]))
