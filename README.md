EcoRover – Autonomous Beach Cleaning Robot
EcoRover is an autonomous beach-cleaning robot software prototype. The Raspberry Pi runs computer vision using a trained YOLO model and sends movement/pickup commands to an Arduino Uno. The Arduino controls the motor driver, reads the ultrasonic sensor, and operates a four-servo robotic arm.
Features
Real-time waste detection using YOLO and a USB camera
Waste-class filtering using the trained model's labels
Camera-based target alignment
Ultrasonic distance measurement
Automatic approach and stopping at a configured distance
Four-servo robotic-arm pickup sequence
Serial communication between Raspberry Pi and Arduino
Stop command when no target is detected or camera frames fail
Repository structure
```text
EcoRover/
├── README.md
├── arduino/
│   └── ecorover_controller.ino
└── raspberry-pi/
    ├── autonomous_ecorover.py
    └── requirements.txt
```
Software requirements
Raspberry Pi OS 64-bit / Python 3
Arduino IDE
Trained YOLO weights named `best.pt` (place beside `autonomous_ecorover.py`)
Python packages in `raspberry-pi/requirements.txt`
Install Python packages:
```bash
python3 -m pip install -r requirements.txt
```
Run the Raspberry Pi program from the `raspberry-pi` directory:
```bash
python3 autonomous_ecorover.py
```
Configuration
Edit the constants at the top of `raspberry-pi/autonomous_ecorover.py` to set the model path, camera index, serial port, and exact waste class names used by your YOLO model.
The Arduino sketch contains pin assignments and example four-servo poses. Those values are code configuration examples and must be matched to the actual hardware. The Cytron MDD3A sketch assumes PWM/PWM control mode. The arm poses are illustrative and require calibration for the physical arm.
Operating principle
The camera captures the beach scene.
YOLO identifies objects belonging to the configured waste classes.
The Pi checks the target's horizontal image position and sends steering commands.
The Arduino reports the front ultrasonic distance.
The Pi approaches while the target is aligned and stops at the configured distance.
The Pi sends `PICK`; the Arduino runs the four-servo reach, grip, lift, and home sequence.
Limitations
YOLO must be trained with suitable beach-waste images and the class names must match the configuration.
A single front ultrasonic sensor measures distance in its field of view; it does not classify objects or provide complete obstacle avoidance.
Camera centering is a simple alignment method and does not estimate full 3D target position.
Example arm poses and motor directions may need adjustment for the actual prototype.
This repository provides the complete software workflow, but has not been validated on the user's assembled hardware.
