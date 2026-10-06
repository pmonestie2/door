# Door Sensor System

This Python code is designed to run on a Raspberry Pi with specific hardware components and serves as a door sensor system. The code is written in Python and utilizes various libraries, including `time`, `board`, `adafruit_mlx90393`, `math`, `collections`, `RPi.GPIO`, and `threading`.

The code monitors a door using a combination of a magnetic field sensor (MLX90393), a break beam sensor, and two relays to control a motor and a direction relay. The magnetic field sensor measures the magnetic field strength and direction changes (caused by a pet wearing a magnet), while the break beam sensor detects the interruption of a light beam caused by a pet. The motor and direction relay are used to control the door motor and direction.

## Web controls

After starting `python3 door.py`, open `http://<pi-ip-address>:8080/` on your
local network. The page shows the door state and Open/Close buttons. No extra
packages are required; deploy `door_web.py` alongside `door.py`.

Open and Close hold the selected state, overriding automatic timers and sensor
triggers, until the next website command, schedule boundary, or process restart.
With scheduling disabled, the hold lasts until another website command or restart.
Close still requires a clear camera check. Commands during movement are retained
and applied after movement finishes. The page displays the current hold.

There is no login: anyone who can reach port 8080 can operate the door. Keep it
on a trusted local network. If the port is occupied, the controller logs the
error and continues without web controls. No web-server tests are included.

The same commands are available as JSON REST endpoints: `POST /open` and
`POST /close`. Send `Content-Type: application/json` and an empty JSON object:

```bash
curl -H 'Content-Type: application/json' -d '{}' http://<pi-ip-address>:8080/close
curl -H 'Content-Type: application/json' -d '{}' http://<pi-ip-address>:8080/open
```

Responses contain `open`, `busy`, and `keep_closed` booleans. Requests normally
return after the movement finishes. When busy, Close records the override for
the controller to apply after the current movement. `open` reflects the
controller's commanded state, not position feedback. GET requests never move
the door. JSON commands need no form token; the API has the same trusted-network
access as the website.

## Android camera

The native [Android app](android/README.md) supports Android 5.1 and newer,
including older Nexus phones. It has the same Push/Pull modes and crop controls
as the iPhone app and uses the same Pi endpoints. It runs capture in a foreground service so the screen can sleep, and captures
full-resolution photos without a shutter sound on supported phones. Push intervals
are selectable from 1 to 60 seconds. Stop in the app or notification ends the run. No Google account, photo-library
access, or rooting is required. Images are retained only on the Pi.

## iPhone camera

The native [iPhone app](ios/README.md) has a Push/Pull toggle: Push uploads a cropped
photo every ten seconds; Pull waits for a Pi command and uploads a fresh photo.
Both use `POST /camera`. In Pull mode, use **Take photo** on the website or
`POST /camera/capture` with JSON `{}`. Check the returned request ID at
`GET /camera/request?id=...`; each request times out after thirty seconds. Open `ios/CatDoorCamera/CatDoorCamera.xcodeproj`
in Xcode to install it on an iPhone running iOS 15 or newer. No jailbreak is needed.
Deploy `door_camera.py` with the updated web module and restart the controller.
The website's **Latest photo** link displays the latest upload; capture/receipt
timestamps are available at `GET /camera/latest`. After each pulled photo is saved, storage keeps only the latest five photos
and their JSON metadata under `camera_images/`. Push uploads accumulate until the next pull. See the app README for setup, retention, and installation.

## Photo check before closing

Every normal closing cycle from an open state, including website Close, first
requests a fresh phone photo through the existing Pull protocol. Keep the phone
app started in **Pull** mode. Deploy `pull_photo.py`, `door_camera.py`, and
`door_inference.py` alongside the controller and web module. Opening never requests
a photo or runs inference. On every process start (including reboot), the controller
lowers the door for the full motor cycle without requesting a photo or running
inference. This establishes the closed position before starting web controls and
sensor/control threads. Subsequent closing cycles require the camera check.

`door_inference.py` reads the saved image matching that capture request and passes
its bytes to `run_inference(image)`. The trained classifier in `fat_cat_model/`
is loaded once and reused, with the threshold stored in `model.pt`. A background
thread warms it at startup with a synthetic image while the door homes. The warm-up
result is discarded; it requests no phone photo and does not affect movement.
An early real inference request waits on the same model lock. Readiness or failure
is logged. Only an
`unobstructed` result permits closing; obstructions and errors keep the door open.

The website's **Run inference** button requests a fresh photo and displays the
model result without moving the door or changing its override. The same action is
available as `POST /camera/inference` with JSON `{}`; it returns `clear_to_close`
and `mock: false`, or an error if capture or inference fails. Website inference works with the door open or closed; requests
are rejected only while it is moving or settling.

A missing phone, timed-out or busy capture request, unreadable photo, or inference
error leaves the door open. Checks can wait approximately 35 seconds for a photo;
failed or blocked checks retry on a later eligible control-loop iteration, with
at least five seconds after each check. No cached photo is used.

After capture and inference, automatic closing rechecks sensor activity and the
schedule. Website Close still overrides sensors and the schedule, but requires
the photo/inference check. Website Open received during capture cancels that close.

## Activity analytics

Open **Activity graph** on the controls page (`/analytics`). Reports are computed
on demand by `door_analytics.py`, with a simple graph, total transits, average per
calendar day, and busiest buckets. Choose daily buckets or hour-of-day totals
across a six-calendar-month window. The initial view is July–December 2023;
choose Live events and an end date to inspect new activity.

A magnet-triggered opening counts once as an inferred `TRANSIT`. While already
open (including scheduled opening), a new magnet burst also counts once after
at least eight seconds without detection. Motor movement and settling are ignored,
and closing never adds a transit. A single sensor cannot establish direction or
confirm that the cat completed a passage. Fast return trips can merge into one event.
`OPEN` and `CLOSE` are completed motor commands, not position feedback.

Events persist in `door_events.sqlite3` alongside the script. Deploy
`door_analytics.py` with the controller and web module, and keep the database
writable. Logging errors are reported without stopping door control.

The separate **one-time** importer converts 2023 completed magnet openings to
ordinary transit events. It ignores raw magnetic readings and beam openings,
and preserves motor open/close records. Run from this directory:

```bash
python3 import_2023.py ../door.log.1 ../door.log.2
```

Imports are repeatable without duplicates; archive and live events are separate.
Copy the populated database to the Pi while its controller is stopped if importing
elsewhere. No import runs at controller startup or when viewing reports.
Old logs may have gaps: zero means no recorded events, not verified inactivity.
Historical transit times use the logged opening completion; old records retain
their local wall times. No website tests are added.

## Daily keep-open range

Edit `KEEP_OPEN_START` and `KEEP_OPEN_END` near the top of `door.py`, then restart
the script. The current settings are `"07:20"` and `"18:00"`, in the Raspberry Pi's local
time, using 24-hour `HH:MM` format. The page displays the configured range.

During this daily range the controller opens the door without needing a magnet
and keeps it open. Website Close takes priority
until the next schedule boundary. At the start boundary, any manual Close override
is cleared and the door opens. At 18:00, the override is cleared and
normal behavior resumes with a fresh 90-second minimum-open timer and the usual
activity checks. Schedule changes are checked approximately every 1.5 seconds
when the controller is idle.

An end time equal to or earlier than the start disables this feature (for example,
`"00:00"` to `"00:00"`). Such ranges do not wrap overnight. Startup requests a photo
and runs the closing check before attempting the closing cycle to establish the
door position. A movement already underway finishes before a scheduled opening.

`run_control_loop()` calls `update_door_state()` and waits 1.5 seconds between
updates. Schedule transitions and recent sensor activity have separate helpers;
the sensor's own thread remains responsible for magnetic readings.

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
90-second minimum open time, continued opening while sensor activity persists,
recent activity, calibration, and magnetic detection thresholds.

These regression tests describe current behavior, including exiting after the
maximum timeout. They do not verify physical obstruction protection, motor
interference, or reliability on real hardware. The existing tuple-identity
comparison in `log_interval` can emit a SyntaxWarning when importing the script.

`main()` configures logging, selects BCM GPIO numbering, connects the real
hardware, and starts the controller. Running `python3 door.py` on the Pi uses
this setup. For other callers, `Relay` and `Beam` require `gpio=`, `Sensor`
requires a `read_magnetic=` callable returning `(x, y, z)`, and `Door` requires
`sensor=`, `motor=`, and `direction=`. `Door` wires the sensor’s `on_magnet_detected` handler itself;
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

Each real inference logs its predicted label, obstruction score, and decision threshold
to both `/home/pi/door.log` and `/home/pi/door-script.log`, including website checks. Synthetic startup warm-up results are discarded.
