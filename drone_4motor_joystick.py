"""
drone_4motor_joystick.py

Drone and motor control classes for quadcopter simulation.

This module defines:
- A Motor class representing a single drone motor
- A Drone class containing quadcopter dynamics and control logic

The drone is controlled using joystick inputs:
- Hover / altitude control
- Pitch control
- Roll control
- Yaw control
"""

import numpy as n


# ===================================================================
# MOTOR CLASS
# ===================================================================
class Motor:

    def __init__(self, x, y, z, sign):
        """
        Initialize a motor object.

        Parameters
        ----------
        x, y, z : float
            Motor position relative to drone center.

        sign : int
            Rotation direction of the propeller.
            1  -> Counterclockwise rotation
           -1  -> Clockwise rotation
        """

        # Indicates whether the motor is active
        self.on_off_state = False

        # Internal thrust value
        self._thrust = 0

        # Position vector in drone body frame
        self.position = n.array([x, y, z])

        # -----------------------------------------------------------
        # Validate motor rotation direction
        # -----------------------------------------------------------
        #  1 -> COUNTERCLOCKWISE
        # -1 -> CLOCKWISE
        if sign == 1 or sign == -1:
            self.direction = sign

        else:
            raise (
                ValueError(
                    "Direction of motor is expressed by 1 or -1"
                )
            )



    # ---------------------------------------------------------------
    # Return motor position as Python list
    # ---------------------------------------------------------------
    def get_pos(self):
        return self.position.tolist()



    # ---------------------------------------------------------------
    # Enable motor
    # ---------------------------------------------------------------
    def on(self):
        self.on_off_state = True



    # ---------------------------------------------------------------
    # Disable motor
    # ---------------------------------------------------------------
    def off(self):
        self.on_off_state = False



    # ---------------------------------------------------------------
    # Getter for thrust property
    # ---------------------------------------------------------------
    @property
    def thrust(self):
        return self._thrust



    # ---------------------------------------------------------------
    # Setter for thrust property
    # ---------------------------------------------------------------
    @thrust.setter
    def thrust(self, p):

        # Prevent thrust updates if motor is disabled
        if self.on_off_state == False:
            raise ValueError(
                "Cannot add thrust to an inactive motor."
            )

        # Clamp thrust to non-negative values
        self._thrust = max(0, p)



# ===================================================================
# DRONE CLASS
# ===================================================================
class Drone:

    def __init__(self):
        """
        Initialize quadcopter configuration.

        Motor layout:

              Front
          M1           M2

          M4           M3
               Rear
        """

        # -----------------------------------------------------------
        # Create four motors
        # -----------------------------------------------------------
        self.motor1 = Motor(-0.15, 0.15, 0.05, 1)
        self.motor2 = Motor(0.15, 0.15, 0.05, -1)
        self.motor3 = Motor(0.15, -0.15, 0.05, -1)
        self.motor4 = Motor(-0.15, -0.15, 0.05, 1)



        # -----------------------------------------------------------
        # Store motors in useful groups
        # -----------------------------------------------------------
        self.motors = [
            self.motor1,
            self.motor2,
            self.motor3,
            self.motor4
        ]

        # Front motors
        self.front_m = [self.motor1, self.motor2]

        # Rear motors
        self.rear_m = [self.motor3, self.motor4]

        # Left motors
        self.left_m = [self.motor1, self.motor4]

        # Right motors
        self.right_m = [self.motor2, self.motor3]



        # -----------------------------------------------------------
        # Yaw torque value
        # -----------------------------------------------------------
        self.yaw = 0


        # -----------------------------------------------------------
        # Base thrust values for each motor
        # -----------------------------------------------------------
        self.force = [0, 0, 0, 0]



    # ---------------------------------------------------------------
    # Turn all motors on
    # ---------------------------------------------------------------

    def motors_on(self, control=None):

        if control == "o":

            for i in range(4):
                self.motors[i].on()



    # ---------------------------------------------------------------
    # Hover / altitude control
    # ---------------------------------------------------------------
    def hover(self, jy=0):
        """
        Adjust vertical thrust using joystick Y-axis.

        Parameters
        ----------
        jy : float
            Vertical joystick input.
        """

        # Vertical thrust scaling factor
        rate = 0.009


        # -----------------------------------------------------------
        # Default hover thrust
        # -----------------------------------------------------------
        # Applied when joystick is centered
        if jy == 0:
            self.force = [3.3, 3.3, 3.3, 3.3]


        # -----------------------------------------------------------
        # Adjust collective thrust
        # -----------------------------------------------------------
        for i in range(4):
            self.force[i] += jy * rate



    # ---------------------------------------------------------------
    # Roll, pitch, and yaw control
    # ---------------------------------------------------------------
    def apply_torque(self, jx1, jy2, jx2):
        """
        Apply rotational control inputs.

        Parameters
        ----------
        jx1 : float
            Yaw joystick input.

        jy2 : float
            Pitch joystick input.

        jx2 : float
            Roll joystick input.
        """

        # -----------------------------------------------------------
        # Set motor thrusts to current hover force
        # -----------------------------------------------------------
        for i in range(4):
            self.motors[i].thrust = self.force[i]


        # Rotational control sensitivity
        rate = 0.004


        # -----------------------------------------------------------
        # Pitch control
        # -----------------------------------------------------------
        # Increase front motors and decrease rear motors
        self.front_m[0].thrust += rate * jy2
        self.front_m[1].thrust += rate * jy2

        self.rear_m[0].thrust -= rate * jy2
        self.rear_m[1].thrust -= rate * jy2



        # -----------------------------------------------------------
        # Roll control
        # -----------------------------------------------------------
        # Increase right motors and decrease left motors
        self.right_m[0].thrust += rate * jx2
        self.right_m[1].thrust += rate * jx2

        self.left_m[0].thrust -= rate * jx2
        self.left_m[1].thrust -= rate * jx2



        # -----------------------------------------------------------
        # Yaw control
        # -----------------------------------------------------------
        # Generates rotational torque around Z-axis
        self.yaw = rate * jx1