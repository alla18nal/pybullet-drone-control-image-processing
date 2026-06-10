"""
input_reader.py

TCP joystick input reader for drone simulation.

This module:
- Receives joystick data over a TCP socket
- Runs asynchronously in a background thread
- Continuously updates the latest joystick state

The joystick input originates from:
- An Arduino Uno R3
- A manually connected analog joystick module
- USB serial communication bridge

NOTE:
-----
Different microcontroller boards may also be used if the
communication protocol is configured accordingly.
"""

import threading
import time


# ===================================================================
# GLOBAL STATE VARIABLES
# ===================================================================

# Latest joystick packet:
# [yaw, throttle, pitch, roll]
latest_joystick = [0.0, 0.0, 0.0, 0.0]

# Controls lifetime of background thread
running = True



# ===================================================================
# TCP INPUT READER
# ===================================================================
def tcp_reader(sock):
    """
    Continuously receive joystick values from TCP socket.

    Parameters
    ----------
    sock : socket.socket
        Connected TCP socket object.

    Expected packet format:
    -----------------------
    value1,value2,value3,value4

    Example:
    --------
    0.2,-0.5,0.1,0.0
    """

    global latest_joystick, running


    # ---------------------------------------------------------------
    # Main receive loop
    # ---------------------------------------------------------------
    while running:

        try:

            # -------------------------------------------------------
            # Receive raw socket data
            # -------------------------------------------------------
            data = sock.recv(1024).decode().strip()


            # Ignore empty packets
            if not data:
                continue



            # -------------------------------------------------------
            # Convert CSV string into float values
            # -------------------------------------------------------
            vals = list(map(float, data.split(',')))



            # -------------------------------------------------------
            # Validate packet size
            # -------------------------------------------------------
            if len(vals) == 4:

                # Update global joystick state
                latest_joystick = vals



        # -----------------------------------------------------------
        # Ignore malformed packets or socket errors
        # -----------------------------------------------------------
        except:
            pass



        # -----------------------------------------------------------
        # Small delay prevents unnecessary CPU usage
        # -----------------------------------------------------------
        time.sleep(0.001)



# ===================================================================
# START BACKGROUND TCP THREAD
# ===================================================================
def start_tcp_reader(sock):
    """
    Start asynchronous TCP reader thread.

    Parameters
    ----------
    sock : socket.socket
        Connected TCP socket.

    Returns
    -------
    threading.Thread
        Background daemon thread object.
    """

    # Create daemon thread
    t = threading.Thread(
        target=tcp_reader,
        args=(sock,),
        daemon=True
    )

    # Start thread execution
    t.start()

    return t