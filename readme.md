# Door Sensor System

This Python code is designed to run on a Raspberry Pi with specific hardware components and serves as a door sensor system. The code is written in Python and utilizes various libraries, including `time`, `board`, `adafruit_mlx90393`, `math`, `collections`, `RPi.GPIO`, and `threading`.

The code monitors a door using a combination of a magnetic field sensor (MLX90393), a break beam sensor, and two relays to control a motor and a direction relay. The magnetic field sensor measures the magnetic field strength and direction, while the break beam sensor detects the interruption of a light beam caused by the door opening or closing. The motor and direction relay are used to control the door motor and direction.

## Hardware Requirements
- Raspberry Pi
- MLX90393 magnetic field sensor
- Break beam sensor
- Relay for motor control
- Relay for direction control

## Prerequisites
- Raspberry Pi OS installed on the Raspberry Pi
- Python 3.x installed on the Raspberry Pi
- Required libraries installed: `time`, `board`, `adafruit_mlx90393`, `math`, `collections`, `RPi.GPIO`, and `threading`

## How to Use
1. Connect the hardware components (MLX90393, break beam sensor, relays) to the Raspberry Pi according to the pin assignments in the code.
2. Install the required libraries if they are not already installed on your Raspberry Pi.
3. Run the code on your Raspberry Pi using Python.
4. The code will monitor the door status and log the magnetic field strength, direction, and door events to a log file (`door.log`) in the specified directory (`/home/pi/door.log`).
5. You can customize the parameters in the code, such as the pin assignments, logging intervals, and sensor settings, to suit your specific requirements.

**Note**: This code is intended as a reference and may require modification to work with your specific hardware setup. Please refer to the documentation of the hardware components and libraries for detailed usage instructions.



