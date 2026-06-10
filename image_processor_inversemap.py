"""
inverse_homography_warp.py

Inverse image warping using rotational homography.

This script:
- Loads a sequence of drone camera frames
- Reads corresponding camera metadata
- Computes a rotational homography matrix
- Performs inverse pixel mapping
- Saves rectified output images

Purpose:
--------
Used for stabilizing or rectifying aerial imagery by removing
camera rotation effects through inverse homography mapping.

The implementation manually performs inverse warping in order
to compare results against OpenCV's built-in warpPerspective().
"""

import cv2
import json
import numpy as np
from math import tan
from matrix_computations import compute_K


# ===================================================================
# PROCESS IMAGE SEQUENCE
# ===================================================================
# Frames captured every 120 simulation steps
for frame in range(1200, 2500, 120):


    # ---------------------------------------------------------------
    # Load image frame
    # ---------------------------------------------------------------
    img_test = cv2.imread(
        f"dataset/images/frame_{frame:06d}.png"
    )


    # Convert image to grayscale
    gray = cv2.cvtColor(
        img_test,
        cv2.COLOR_BGR2GRAY
    )



    # ===============================================================
    # LOAD METADATA
    # ===============================================================
    with open(
        f"dataset/metadata/meta_frame_{frame:06d}.json",
        "r"
    ) as f:

        data = json.load(f)



    # ===============================================================
    # EXTRACT CAMERA PARAMETERS
    # ===============================================================

    # Camera field of view
    fov = data["fov"]

    # Image resolution
    width = data["width"]
    height = data["height"]


    # ---------------------------------------------------------------
    # Compute camera intrinsic matrix
    # ---------------------------------------------------------------
    K = compute_K(fov, width, height)


    # ---------------------------------------------------------------
    # Camera rotation matrix
    # ---------------------------------------------------------------
    R = np.array(data["R_cam"])



    # ===============================================================
    # CREATE OUTPUT IMAGE
    # ===============================================================
    final_img = np.zeros_like(gray)



    # ===============================================================
    # COMPUTE HOMOGRAPHY MATRIX
    # ===============================================================
    # Pure rotational homography:
    #
    # H = K * R^T * K^-1
    #
    # R^T is used because we are undoing the camera rotation.
    H = K @ R.T @ np.linalg.inv(K)



    # ===============================================================
    # INVERSE IMAGE WARPING
    # ===============================================================
    # For each pixel in the output image:
    # 1. Map backwards into the source image
    # 2. Sample source intensity
    # 3. Assign intensity to output pixel
    for y in range(height):

        for x in range(width):


            # -------------------------------------------------------
            # Output image coordinate
            # -------------------------------------------------------
            coord = np.array([x, y, 1])



            # -------------------------------------------------------
            # Back-project into source image
            # -------------------------------------------------------
            src = np.linalg.inv(H) @ coord



            # -------------------------------------------------------
            # Prevent division by very small homogeneous scale
            # -------------------------------------------------------
            if abs(src[2]) < 1e-6:
                continue



            # -------------------------------------------------------
            # Normalize homogeneous coordinates
            # -------------------------------------------------------
            src /= src[2]



            # -------------------------------------------------------
            # Convert to integer pixel coordinates
            # -------------------------------------------------------
            src_x = int(round(src[0]))
            src_y = int(round(src[1]))



            # -------------------------------------------------------
            # Bounds checking
            # -------------------------------------------------------
            if 0 <= src_x < width and 0 <= src_y < height:

                # Copy source pixel intensity
                final_img[y, x] = gray[src_y, src_x]



    # ===============================================================
    # SAVE RESULT IMAGE
    # ===============================================================
    cv2.imwrite(
        f"dataset/result_images/frame_{frame:06d}.png",
        final_img
    )



# ===================================================================
# OPTIONAL: OpenCV COMPARISON TESTS
# ===================================================================

# -------------------------------------------------------------------
# OpenCV built-in warpPerspective() comparison
# -------------------------------------------------------------------
# result = cv2.warpPerspective(img_test, H, (width, height))


# -------------------------------------------------------------------
# Display original image
# -------------------------------------------------------------------
# cv2.imshow('Display Window', gray)
# cv2.waitKey(0)
# cv2.destroyAllWindows()


# -------------------------------------------------------------------
# Display manually warped image
# -------------------------------------------------------------------
# cv2.imshow('Display Window', final_img)
# cv2.waitKey(0)
# cv2.destroyAllWindows()


# -------------------------------------------------------------------
# Display OpenCV warpPerspective() result
# -------------------------------------------------------------------
# cv2.imshow('Display Window', result)
# cv2.waitKey(0)
# cv2.destroyAllWindows()