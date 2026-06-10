"""
3d_joystick.py

Main simulation loop for the quadcopter drone.
This script:
- Connects to the PyBullet physics engine
- Receives joystick commands through a TCP socket
- Updates drone thrust and torques
- Applies forces to the drone body
- Runs the physics simulation in real time
"""

import time
from drone_4motor_joystick import Drone
import pybullet as p
import pybullet_data
import numpy as np
import socket
import take_picture as pic
import input_reader as ir


# -------------------------------------------------------------------
# Global frame counter used for optional image capture
# -------------------------------------------------------------------
frame_counter = 0


# -------------------------------------------------------------------
# TCP socket connection to joystick input bridge (Windows side)
# -------------------------------------------------------------------
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect(("172.22.96.1", 12345))

print("Connected to Windows bridge")


# -------------------------------------------------------------------
# PyBullet simulation setup
# -------------------------------------------------------------------
p.connect(p.GUI)

# Set gravity acceleration (m/s^2)
p.setGravity(0, 0, -9.81)

# Disable real-time simulation for manual stepping
p.setRealTimeSimulation(0)

# Load built-in PyBullet assets
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Create ground plane
plane = p.loadURDF("plane.urdf")


# -------------------------------------------------------------------
# Configure simulation camera
# -------------------------------------------------------------------
p.resetDebugVisualizerCamera(
    cameraDistance=10,
    cameraYaw=90,
    cameraPitch=-35,
    cameraTargetPosition=[0, 0, 0]
)

# Optional rendering optimization
# p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)

# Improve simulation stability
p.setPhysicsEngineParameter(numSolverIterations=50)


# -------------------------------------------------------------------
# Load drone model
# -------------------------------------------------------------------
# NOTE:
# Currently using a simple URDF body.
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
# Startup instructions
# -------------------------------------------------------------------
print("If you want to start the simulation please turn on the motors with the key O")

key = input()
drone.motors_on(key)

print("Press G to quit")
print("Press R to restart from 0 position")


# -------------------------------------------------------------------
# Start asynchronous joystick TCP reader. Since my connection is Windows --> Ubuntu I require a bridge file 
# on Windows. However, if you have an available USB port access, just plug in the connection to the receiver.py file.
# -------------------------------------------------------------------
ir.start_tcp_reader(sock)


# ===================================================================
# MAIN SIMULATION LOOP
# ===================================================================
while True:

    # ---------------------------------------------------------------
    # Read latest joystick control packet
    # control_data structure:
    # [hover_input, pitch_roll_keys, yaw_input, dt]
    # ---------------------------------------------------------------
    control_data = ir.latest_joystick

    # print(control_data)

    if control_data:

        # Control vertical thrust
        drone.hover(control_data[0])

        # Apply rotational torques
        drone.apply_torque(
            control_data[1],
            control_data[2],
            control_data[3]
        )

    # Debug outputs
    # print("FORCE:", drone.force)
    # print("THRUST:", [m.thrust for m in drone.motors])



    # ---------------------------------------------------------------
    # Keyboard input handling for resetting
    # ---------------------------------------------------------------
    keys = p.getKeyboardEvents()

    # Reset drone position and thrust values
    if ord('r') in keys and keys[ord('r')] & p.KEY_IS_DOWN:

        for i in range(4):
            drone.force[i] = 0

            p.resetBasePositionAndOrientation(
                drone_id,
                [0, 0, 1],
                [0, 0, 0, 1]
            )



    # ---------------------------------------------------------------
    # Get drone pose from simulation
    # ---------------------------------------------------------------
    pos, orn = p.getBasePositionAndOrientation(drone_id)

    # Convert quaternion orientation to rotation matrix
    R = np.array(
        p.getMatrixFromQuaternion(orn)
    ).reshape(3, 3)



    # ---------------------------------------------------------------
    # Optional onboard camera capture
    # ---------------------------------------------------------------
    # if frame_counter > 1000 and frame_counter % 120 == 0:
    #     pic.start_camera(frame_counter, cam_pos, R)



    # ---------------------------------------------------------------
    # Make debug camera follow the drone
    # ---------------------------------------------------------------
    p.resetDebugVisualizerCamera(
        cameraDistance=5,
        cameraYaw=0,
        cameraPitch=-30,
        cameraTargetPosition=pos
    )



    # ---------------------------------------------------------------
    # Apply motor thrust forces
    # ---------------------------------------------------------------
    for i in range(4):

        p.applyExternalForce(
            objectUniqueId=drone_id,

            # Apply to base link
            linkIndex=-1,

            # Upward thrust force
            forceObj=[0, 0, drone.motors[i].thrust],

            # Motor position relative to drone frame
            posObj=drone.motors[i].get_pos(),

            # Force defined in local drone frame
            flags=p.LINK_FRAME
        )



    # ---------------------------------------------------------------
    # Apply yaw torque around Z-axis
    # ---------------------------------------------------------------
    p.applyExternalTorque(
        objectUniqueId=drone_id,
        linkIndex=-1,

        # [roll, pitch, yaw]
        torqueObj=[0, 0, drone.yaw],

        # Torque applied in world frame
        flags=p.WORLD_FRAME
    )



    # ---------------------------------------------------------------
    # Advance physics simulation
    # ---------------------------------------------------------------
    p.stepSimulation()

    # Increment rendered frame counter
    frame_counter += 1

    # Match simulation speed to real time
    time.sleep(dt)