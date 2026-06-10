"""
tcp_test_client.py

Simple TCP client for receiving joystick data from the
Windows communication bridge.

This script:
- Connects to the TCP server
- Receives joystick values sent from Arduino hardware
- Parses incoming control packets
- Prints joystick values for debugging/testing

Purpose:
--------
Used for validating communication between:
- Arduino joystick controller
- Windows socket bridge
- Python drone simulation

Expected packet format:
-----------------------
jx1,jy1,jx2,jy2

Example:
--------
0.12,-0.45,0.03,0.88
"""

import socket


# ===================================================================
# CREATE TCP SOCKET
# ===================================================================
sock = socket.socket(
    socket.AF_INET,
    socket.SOCK_STREAM
)


# ===================================================================
# CONNECT TO WINDOWS SOCKET BRIDGE
# ===================================================================
sock.connect(("172.22.96.1", 12345))

print("Connected to Windows bridge")



# ===================================================================
# MAIN RECEIVE LOOP
# ===================================================================
while True:

    # ---------------------------------------------------------------
    # Receive socket data
    # ---------------------------------------------------------------
    data = sock.recv(1024).decode().strip()



    # ---------------------------------------------------------------
    # Ignore empty packets
    # ---------------------------------------------------------------
    if not data:
        continue



    try:

        # -----------------------------------------------------------
        # Parse joystick values
        # -----------------------------------------------------------
        jx1, jy1, jx2, jy2 = map(
            float,
            data.split(',')
        )



        # -----------------------------------------------------------
        # Print parsed joystick inputs
        # -----------------------------------------------------------
        print(jx1, jy1, jx2, jy2)



        # -----------------------------------------------------------
        # Future integration:
        # These values can later be used directly inside
        # the PyBullet drone simulation.
        # -----------------------------------------------------------



    # ---------------------------------------------------------------
    # Handle malformed packets
    # ---------------------------------------------------------------
    except ValueError:

        print("Bad data:", data)