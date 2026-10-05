# Door Sensor System

This Python code is designed to run on a Raspberry Pi with specific hardware components and serves as a door sensor system. The code is written in Python and utilizes various libraries, including `time`, `board`, `adafruit_mlx90393`, `math`, `collections`, `RPi.GPIO`, and `threading`.

The code monitors a door using a combination of a magnetic field sensor (MLX90393), a break beam sensor, and two relays to control a motor and a direction relay. The magnetic field sensor measures the magnetic field strength and direction changes (caused by a pet wearing a magnet), while the break beam sensor detects the interruption of a light beam caused by a pet. The motor and direction relay are used to control the door motor and direction.

## Web controls

After starting `python3 door.py`, open `http://<pi-ip-address>:8080/` on your
local network. The page shows the door state and Open/Close buttons. No extra
packages are required; deploy `door_web.py` alongside `door.py`.

Open uses the normal opening cycle and automatic closing timer. Close bypasses
the minimum-open timer and activity checks. Commands received during movement
or settling are ignored. The page returns after the movement finishes; use
Refresh status to update it. Automatic magnet control remains active, so Close
does not keep the door locked shut.

There is no login: anyone who can reach port 8080 can operate the door. Keep it
on a trusted local network. If the port is occupied, the controller logs the
error and continues without web controls. No web-server tests are included.

## Tests

From this directory, run:

```bash
python3 -m unittest discover -s . -p 'test_*.py' -v
```

The tests require NumPy (`python3 -m pip install numpy`) and use Python's
standard-library `unittest`. The module imports normally without opening log
files, registering signal handlers, or loading Raspberry Pi libraries. Tests
pass fake GPIO, magnetic readings, relays, a clock, and a process-exit function
into constructors. No background threads are started and no motor is operated.
Tests cover import isolation, opening/closing sequences, relay polarity, the
90-second minimum open time, the 500-second activity-blocked exit timeout,
recent activity, calibration, and magnetic detection thresholds.

These regression tests describe current behavior, including exiting after the
maximum timeout. They do not verify physical obstruction protection, motor
interference, or reliability on real hardware. The existing tuple-identity
comparison in `log_interval` can emit a SyntaxWarning when importing the script.

`main()` configures logging, selects BCM GPIO numbering, connects the real
hardware, and starts the controller. Running `python3 door.py` on the Pi uses
this setup. For other callers, `Relay` and `Beam` require `gpio=`, `Sensor`
requires a `read_magnetic=` callable returning `(x, y, z)`, and `Door` requires
`sensor=`, `motor=`, and `direction=`. `Door` wires the sensor callback itself;
its optional `clock=` and `exit_process=` arguments default to production time
and process-exit functions.

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
