"""
matrix_computations.py

Camera matrix and rotation matrix utility functions.

This module provides:
- Camera intrinsic matrix computation
- Quaternion-to-rotation-matrix conversion

Used throughout the drone image processing pipeline for:
- Homography estimation
- Image rectification
- Camera projection operations
"""

import numpy as np
from math import tan, radians


# ===================================================================
# CAMERA INTRINSIC MATRIX
# ===================================================================
def compute_K(fov_deg, width, height):
    """
    Compute camera intrinsic matrix K.

    Parameters
    ----------
    fov_deg : float
        Vertical field of view in degrees.

    width : int
        Image width in pixels.

    height : int
        Image height in pixels.

    Returns
    -------
    numpy.ndarray
        3x3 intrinsic camera matrix.

    Notes
    -----
    PyBullet uses vertical field of view (FOV).
    Assumes square pixels (fx = fy).
    """

    # ---------------------------------------------------------------
    # Convert field of view from degrees to radians
    # ---------------------------------------------------------------
    fov_rad = radians(fov_deg)



    # ---------------------------------------------------------------
    # Compute focal lengths
    # ---------------------------------------------------------------
    fy = height / (2 * tan(fov_rad / 2))

    # Assume square pixels
    fx = fy



    # ---------------------------------------------------------------
    # Principal point (image center)
    # ---------------------------------------------------------------
    cx = width / 2
    cy = height / 2



    # ---------------------------------------------------------------
    # Construct intrinsic matrix
    # ---------------------------------------------------------------
    K = np.array([
        [fx, 0,  cx],
        [0,  fy, cy],
        [0,   0,  1]
    ], dtype=np.float64)



    return K



# ===================================================================
# QUATERNION TO ROTATION MATRIX
# ===================================================================
# Optional utility function for converting quaternion orientation
# data into a 3x3 rotation matrix.
def compute_R(data):
    """
    Compute 3x3 rotation matrix from quaternion.

    Parameters
    ----------
    data : dict
        Dictionary containing quaternion values under:
        data["quaternion"]

    Returns
    -------
    numpy.ndarray
        3x3 rotation matrix.
    """

    # ---------------------------------------------------------------
    # Extract quaternion values
    # ---------------------------------------------------------------
    Q = data["quaternion"]

    q0 = Q[0]
    q1 = Q[1]
    q2 = Q[2]
    q3 = Q[3]



    # ---------------------------------------------------------------
    # First row of rotation matrix
    # ---------------------------------------------------------------
    r00 = 2 * (q0 * q0 + q1 * q1) - 1
    r01 = 2 * (q1 * q2 - q0 * q3)
    r02 = 2 * (q1 * q3 + q0 * q2)



    # ---------------------------------------------------------------
    # Second row of rotation matrix
    # ---------------------------------------------------------------
    r10 = 2 * (q1 * q2 + q0 * q3)
    r11 = 2 * (q0 * q0 + q2 * q2) - 1
    r12 = 2 * (q2 * q3 - q0 * q1)



    # ---------------------------------------------------------------
    # Third row of rotation matrix
    # ---------------------------------------------------------------
    r20 = 2 * (q1 * q3 - q0 * q2)
    r21 = 2 * (q2 * q3 + q0 * q1)
    r22 = 2 * (q0 * q0 + q3 * q3) - 1



    # ---------------------------------------------------------------
    # Construct rotation matrix
    # ---------------------------------------------------------------
    R = np.array([
        [r00, r01, r02],
        [r10, r11, r12],
        [r20, r21, r22]
    ])



    return R