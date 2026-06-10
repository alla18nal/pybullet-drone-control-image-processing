# nalbandyan_alla — Project

# Drone Control & Image Processing Project

## Overview

This project contains a drone simulation and image processing pipeline with two drone control modes:

- Keyboard-controlled drone
- Joystick-controlled drone

The simulation allows drone movement, optional image capture, and post-processing of captured images using inverse mapping.

---

# Keyboard Drone Control

Run the keyboard simulation with:

```bash
python 3d_4_keyboard.py
```

Controls:

- `Q` → Move up
- `E` → Move down
- `A` → Roll left
- `D` → Roll right
- `W` → Pitch forward
- `S` → Pitch backward
- `R` → Reset drone
- `G` → Exit simulation

## Taking Pictures

To enable image capture, uncomment the camera code inside the simulation file.

You can change the `frame_counter` value to customize the saved image names.

Image capture functionality is handled by:

```bash
take_picture.py
```

---

# Joystick Drone Control

Run the joystick simulation with:

```bash
python 3d_4_joystick.py
```

Joystick support depends on your own hardware configuration.

Example setup used in this project:

```text
Arduino → Socket Connection → Windows/Ubuntu Bridge
```

Other possible setups include:

- Direct USB joystick connection
- Bluetooth connection with Arduino
- Other custom controller configurations

Make sure your joystick input is configured properly before running the simulation.

---

# Image Processing

After capturing images, run:

```bash
python image_processing_inversemap.py
```

Raw captured images are stored in:

```text
dataset/images
```

Processed images are saved in:

```text
dataset/result_images
```

---

# Requirements

Install the required Python libraries before running the project:

```bash
pip install numpy opencv-python pybullet
```

Additional libraries may be required depending on your hardware or joystick setup.
