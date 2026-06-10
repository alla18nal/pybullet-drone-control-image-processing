"""
drone_4motor_keyboard.py

Core drone and motor control classes for keyboard-controlled
quadcopter simulation.

This module defines:
- A Motor class representing an individual rotor
- A Drone class implementing thrust and torque control

The drone supports:
- Hover control
- Pitch control
- Roll control
- Manual thrust adjustment
"""

import numpy as n


# ===================================================================
# MOTOR CLASS
# ===================================================================
class Motor:

    def __init__(self, x, y, z):
        """
        Initialize motor object.

        Parameters
        ----------
        x, y, z : float
            Position of the motor relative to the drone center.
        """

        # Motor activation state
        self.on_off_state = False

        # Internal thrust value
        self._thrust = 0

        # Position vector in drone body coordinates
        self.position = n.array([x, y, z])



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

        # Prevent thrust updates if motor is inactive
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
        # Create drone motors
        # -----------------------------------------------------------
        # NOTE:
        # Motor positions are currently hardcoded.
        # Future versions may extract geometry directly from URDF.
        self.motor1 = Motor(-0.15, 0.15, 0.05)
        self.motor2 = Motor(0.15, 0.15, 0.05)
        self.motor3 = Motor(0.15, -0.15, 0.05)
        self.motor4 = Motor(-0.15, -0.15, 0.05)



        # -----------------------------------------------------------
        # Store motors in grouped configurations
        # -----------------------------------------------------------
        self.motors = [
            self.motor1,
            self.motor2,
            self.motor3,
            self.motor4
        ]

        # Front pair
        self.front_m = [self.motor1, self.motor2]

        # Rear pair
        self.rear_m = [self.motor3, self.motor4]

        # Left pair
        self.left_m = [self.motor1, self.motor4]

        # Right pair
        self.right_m = [self.motor2, self.motor3]



        # -----------------------------------------------------------
        # Base thrust values
        # -----------------------------------------------------------
        self.force = [0, 0, 0, 0]



    # ---------------------------------------------------------------
    # Turn all motors on
    # ---------------------------------------------------------------
    def motors_on(self, control=None):

        if "o" in control:

            for i in range(4):
                self.motors[i].on()



    # ---------------------------------------------------------------
    # Hover / vertical thrust control
    # ---------------------------------------------------------------
    def hover(self, control=None, dt=0.2):
        """
        Adjust collective thrust.

        Controls:
        ----------
        q : Increase altitude
        e : Decrease altitude
        r : Reset thrust
        """

        # Hover sensitivity coefficient
        rate = 0.1


        # -----------------------------------------------------------
        # Increase upward thrust
        # -----------------------------------------------------------
        if "q" in control:

            for i in range(4):
                self.force[i] += rate * dt


        # -----------------------------------------------------------
        # Decrease upward thrust
        # -----------------------------------------------------------
        elif "e" in control:

            for i in range(4):
                self.force[i] -= rate * dt


        # -----------------------------------------------------------
        # Reset all thrust values
        # -----------------------------------------------------------
        elif "r" in control:

            for i in range(4):
                self.force[i] = 0



    # ---------------------------------------------------------------
    # Roll and pitch control
    # ---------------------------------------------------------------
    def apply_torque(self, keys, dt):
        """
        Apply rotational control to the drone.

        Controls:
        ----------
        w / s : Pitch control
        a / d : Roll control
        """

        # -----------------------------------------------------------
        # Set motor thrusts to current base force values
        # -----------------------------------------------------------
        for i in range(4):
            self.motors[i].thrust = self.force[i]


        # Rotational sensitivity coefficient
        rate = 2



        # -----------------------------------------------------------
        # Pitch backward
        # -----------------------------------------------------------
        if "s" in keys:

            self.front_m[0].thrust += rate * dt
            self.front_m[1].thrust += rate * dt

            self.rear_m[0].thrust -= rate * dt
            self.rear_m[1].thrust -= rate * dt



        # -----------------------------------------------------------
        # Pitch forward
        # -----------------------------------------------------------
        if "w" in keys:

            self.rear_m[0].thrust += rate * dt
            self.rear_m[1].thrust += rate * dt

            self.front_m[0].thrust -= rate * dt
            self.front_m[1].thrust -= rate * dt



        # -----------------------------------------------------------
        # Roll left
        # -----------------------------------------------------------
        if "a" in keys:

            self.right_m[0].thrust += rate * dt
            self.right_m[1].thrust += rate * dt

            self.left_m[0].thrust -= rate * dt
            self.left_m[1].thrust -= rate * dt



        # -----------------------------------------------------------
        # Roll right
        # -----------------------------------------------------------
        if "d" in keys:

            self.left_m[0].thrust += rate * dt
            self.left_m[1].thrust += rate * dt

            self.right_m[0].thrust -= rate * dt
            self.right_m[1].thrust -= rate * dt



    # ---------------------------------------------------------------
    # Optional helper function
    # ---------------------------------------------------------------
    # def update_overall(self):
    #     self.force = [
    #         self.motors[i].thrust
    #         for i in self.motors
    #     ]