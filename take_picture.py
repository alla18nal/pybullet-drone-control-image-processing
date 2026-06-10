import threading
import time
import numpy as np
import json
import pybullet as p
import cv2


def take_picture(frame_counter, cam_pos, R):
    """
    Captures a rendered image from a simulated camera in PyBullet,
    saves both the RGB image and its associated metadata.
    """

    # Define camera orientation vectors:
    # up_vector defines "up" direction in world space
    up_vector = np.array([0, 1, 0])

    # down_vector defines the forward direction of the camera in its local frame
    down_vector = np.array([0, 0, -1])

    # Rotate the camera's forward direction into world space using rotation matrix R
    # This determines where the camera is looking at
    camera_target = cam_pos + R @ down_vector

    # Build the view matrix (camera extrinsics)
    # This transforms world coordinates into camera coordinates
    view_matrix = p.computeViewMatrix(
        cameraEyePosition=cam_pos,           # camera position in world space
        cameraTargetPosition=camera_target,  # point the camera looks at
        cameraUpVector=up_vector             # defines camera roll orientation
    )

    # Image properties
    width = 160
    height = 120
    fov = 60  # field of view in degrees
    aspect = width / height
    near = 0.01  # near clipping plane
    far = 100    # far clipping plane

    # Build projection matrix (camera intrinsics approximation)
    # Converts 3D view space into 2D image plane
    projection_matrix = p.computeProjectionMatrixFOV(
        fov=fov,
        aspect=aspect,
        nearVal=near,
        farVal=far
    )

    # Render the image from the simulated camera
    img = p.getCameraImage(
        width=width,
        height=height,
        viewMatrix=view_matrix,
        projectionMatrix=projection_matrix
    )

    # Extract RGB buffer from PyBullet output
    rgb = img[2]

    # Reshape flat buffer into (H, W, RGBA)
    rgb = np.reshape(rgb, (height, width, 4))

    # Remove alpha channel (RGBA -> RGB)
    rgb = rgb[:, :, :3]

    # Flip vertically because PyBullet image origin is bottom-left
    rgb = cv2.flip(rgb, 0)

    # Save image to disk with zero-padded frame index
    filename = f"dataset/images/frame_{frame_counter:06d}.png"

    # Save associated metadata (camera pose + parameters)
    meta_filename = f"dataset/metadata/meta_frame_{frame_counter:06d}.json"

    # Write image file
    cv2.imwrite(filename, rgb)

    # Store metadata for later reconstruction / training
    metadata = {
        "position": cam_pos.tolist(),  # camera world position
        "fov": fov,                    # field of view used for rendering
        "width": width,                # image width
        "height": height,              # image height
        "R_cam": R.tolist(),           # camera rotation matrix
    }

    # Write metadata JSON
    with open(meta_filename, "w") as f:
        json.dump(metadata, f, indent=4)

    # Small sleep to avoid overwhelming CPU (important in tight loops/threads)
    time.sleep(0.001)


def start_camera(frame_counter, cam_pos, R):
    """
    Launches a separate thread to capture a single camera frame
    without blocking the main simulation loop.
    """

    # Create a daemon thread so it exits automatically with the program
    t = threading.Thread(
        target=take_picture,
        args=(frame_counter, cam_pos, R,),
        daemon=True
    )

    # Start asynchronous capture
    t.start()

    return t