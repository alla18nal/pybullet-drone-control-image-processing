"""
3d_keyboard.py

Keyboard-controlled quadcopter simulation using PyBullet.

This script:
- Initializes the PyBullet physics environment
- Controls the drone using keyboard inputs
- Applies motor thrust forces to simulate quadcopter motion
- Captures aerial images from a virtual onboard camera
- Advances the simulation in real time
"""

import time
from drone_4motor_keyboard import Drone
import pybullet as p
import pybullet_data
import numpy as np
import take_picture as pic


# -------------------------------------------------------------------
# Initialize PyBullet physics simulation
# -------------------------------------------------------------------
p.connect(p.GUI)

# Set gravitational acceleration (m/s^2)
p.setGravity(0, 0, -9.81)

# Disable automatic real-time stepping
p.setRealTimeSimulation(0)

# Load PyBullet default assets
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Create ground plane
plane = p.loadURDF("plane.urdf")


# -------------------------------------------------------------------
# Configure debug visualization camera
# -------------------------------------------------------------------
p.resetDebugVisualizerCamera(
    cameraDistance=30,
    cameraYaw=90,
    cameraPitch=-35,
    cameraTargetPosition=[0, 0, 0]
)


# -------------------------------------------------------------------
# Load drone body into the simulation
# -------------------------------------------------------------------
# NOTE:
# This URDF currently represents a simplified drone body.
# Can later be replaced with a more detailed physical model.
drone_id = p.loadURDF("drone.urdf", [0, 0, 1])


# -------------------------------------------------------------------
# Create drone controller object
# -------------------------------------------------------------------
drone = Drone()


# -------------------------------------------------------------------
# Simulation timestep
# -------------------------------------------------------------------
dt = 1 / 240


# -------------------------------------------------------------------
# User startup instructions
# -------------------------------------------------------------------
print("If you want to start the simulation please turn on the motors with the key O")

key = input()

# Turn motors on
drone.motors_on([key])

print("Press G to quit")


# -------------------------------------------------------------------
# Camera reference vectors
# -------------------------------------------------------------------
# Up direction of camera frame
up_vector = np.array([0, 1, 0])

# Downward viewing direction
down_vector = np.array([0, 0, -1])


# -------------------------------------------------------------------
# Global frame counter used for image capture
# -------------------------------------------------------------------
frame_counter = 0


# ===================================================================
# MAIN SIMULATION LOOP
# ===================================================================
while True:

    # ---------------------------------------------------------------
    # Measure loop execution start time
    # ---------------------------------------------------------------
    t0 = time.time()


    # ---------------------------------------------------------------
    # Read keyboard events
    # ---------------------------------------------------------------
    keys = p.getKeyboardEvents()

    # Store currently pressed control keys
    pressed = []

    for k in ['w', 'a', 's', 'd', 'z', 'x', 'q', 'e', 'r']:

        if ord(k) in keys and keys[ord(k)] & p.KEY_IS_DOWN:
            pressed.append(k)



    # ---------------------------------------------------------------
    # Exit simulation
    # ---------------------------------------------------------------
    if ord('g') in keys and keys[ord('g')] & p.KEY_IS_DOWN:
        break



    # ---------------------------------------------------------------
    # Reset drone state
    # ---------------------------------------------------------------
    if ord('r') in keys and keys[ord('r')] & p.KEY_IS_DOWN:

        # Reset motor forces
        for i in range(4):
            drone.force[i] = 0

            # Reset drone position and orientation
            p.resetBasePositionAndOrientation(
                drone_id,
                [0, 0, 1],
                [0, 0, 0, 1]
            )



    # ---------------------------------------------------------------
    # Update drone controls
    # ---------------------------------------------------------------

    # Vertical thrust control
    drone.hover(pressed)

    # Roll, pitch, and yaw control
    drone.apply_torque(pressed, dt)



    # ---------------------------------------------------------------
    # Retrieve drone pose from simulation
    # ---------------------------------------------------------------
    pos, orn = p.getBasePositionAndOrientation(drone_id)

    # Convert quaternion orientation into rotation matrix
    R = np.array(
        p.getMatrixFromQuaternion(orn)
    ).reshape(3, 3)



    # ---------------------------------------------------------------
    # Define virtual onboard camera position
    # ---------------------------------------------------------------
    # Camera positioned slightly below the drone body
    cam_pos = np.array([
        pos[0],
        pos[1],
        pos[2] - 1
    ])



    # ---------------------------------------------------------------
    # Capture aerial images periodically
    # ---------------------------------------------------------------
    # Starts after stabilization period
    # Captures one frame every 120 simulation frames
    if frame_counter > 1000 and frame_counter % 120 == 0:

        pic.start_camera(
            frame_counter,
            cam_pos,
            R
        )



    # ---------------------------------------------------------------
    # Optional follow-camera visualization
    # ---------------------------------------------------------------
    # p.resetDebugVisualizerCamera(
    #     cameraDistance=5,
    #     cameraYaw=0,
    #     cameraPitch=-30,
    #     cameraTargetPosition=pos
    # )



    # ---------------------------------------------------------------
    # Apply motor thrust forces
    # ---------------------------------------------------------------
    for i in range(4):

        p.applyExternalForce(
            objectUniqueId=drone_id,

            # Apply to base link
            linkIndex=-1,

            # Upward thrust generated by motor
            forceObj=[0, 0, drone.motors[i].thrust],

            # Motor position relative to drone body
            posObj=drone.motors[i].get_pos(),

            # Force applied in drone local frame
            flags=p.LINK_FRAME
        )



    # ---------------------------------------------------------------
    # Advance simulation
    # ---------------------------------------------------------------
    frame_counter += 1

    p.stepSimulation()

    # Synchronize simulation with real time
    time.sleep(dt)