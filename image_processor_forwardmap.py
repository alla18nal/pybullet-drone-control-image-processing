"""
forward_map_test.py

Image warping experiment using forward homography mapping.

This script:
- Loads a rendered drone image and camera metadata
- Computes the camera intrinsic matrix
- Applies a rotational homography transformation
- Forward-projects pixels into a new image frame
- Displays the warped output image

Purpose:
--------
Used for comparing forward mapping against inverse mapping
during aerial image rectification experiments.

NOTE:
-----
Forward mapping may produce holes or sparse regions in the
output image because multiple source pixels may map unevenly
into the destination frame.
"""

import cv2
import json
import numpy as np
from math import tan
from matrix_computations import compute_K, compute_R


# ===================================================================
# IMAGE LOADING
# ===================================================================
# Manually select which frame to render by changing the frame number
img_test = cv2.imread(
    "dataset/images/frame_004080.png"
)


# -------------------------------------------------------------------
# Convert image to grayscale
# -------------------------------------------------------------------
gray = cv2.cvtColor(img_test, cv2.COLOR_BGR2GRAY)



# ===================================================================
# LOAD CAMERA METADATA
# ===================================================================
with open(
    "dataset/metadata/meta_frame_004080.json",
    "r"
) as f:

    data = json.load(f)

    # Print metadata for debugging
    print(data)



# ===================================================================
# EXTRACT CAMERA PARAMETERS
# ===================================================================

# Camera field of view
fov = data["fov"]

# Image dimensions
width = data["width"]
height = data["height"]


# -------------------------------------------------------------------
# Compute camera intrinsic matrix
# -------------------------------------------------------------------
K = compute_K(fov, width, height)


# -------------------------------------------------------------------
# Camera rotation matrix
# -------------------------------------------------------------------
R = np.array(data["R_cam"])



# ===================================================================
# OUTPUT IMAGE BUFFER
# ===================================================================
# Initialize empty image for warped result
final_img = np.zeros_like(gray)



# ===================================================================
# FORWARD IMAGE MAPPING
# ===================================================================
# For every pixel in the source image:
# 1. Convert pixel to homogeneous coordinates
# 2. Apply rotational homography transformation
# 3. Reproject into image plane
# 4. Copy intensity into transformed position
for y in range(height):

    for x in range(width):

        # -----------------------------------------------------------
        # Source pixel intensity
        # -----------------------------------------------------------
        intensity = gray[y, x]


        # -----------------------------------------------------------
        # Homogeneous image coordinate
        # -----------------------------------------------------------
        coord = np.array([x, y, 1])


        # -----------------------------------------------------------
        # Apply forward rotational homography
        # -----------------------------------------------------------
        new_coord = (
            K
            @ np.linalg.inv(R)
            @ np.linalg.inv(K)
            @ coord
        )


        # -----------------------------------------------------------
        # Normalize homogeneous coordinates
        # -----------------------------------------------------------
        new_coord /= new_coord[2]


        # Extract transformed pixel coordinates
        new_x = int(new_coord[0])
        new_y = int(new_coord[1])


        # -----------------------------------------------------------
        # Bounds check
        # -----------------------------------------------------------
        if 0 <= new_x < width and 0 <= new_y < height:

            # Assign transformed pixel value
            final_img[new_y, new_x] = intensity



# ===================================================================
# DISPLAY RESULT
# ===================================================================

# Optional original image display
# cv2.imshow('Display Window', gray)

# Display transformed image
cv2.imshow('Display Window', final_img)

# Wait until key press
cv2.waitKey(0)

# Close all OpenCV windows
cv2.destroyAllWindows()