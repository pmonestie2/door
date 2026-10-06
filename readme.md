# Cat door

A Raspberry Pi controller for a motorized cat door, with a small website, a phone
camera, obstruction detection, and passage analytics.
That cat has a magnet attached to his colar. A very cheap magnet detector is used to sense cat presence:
The reason is that magnet detectors are much more sensitive then RFID where as you would need a powerful antena.
This cat door is meant to prevent racoons which can "destroy/enter" a regular catdoor (even RFID): This one is a vertical door with an actuator.
The Pi controls all the logic, the mortor and the magnet sensor.
The phone supplies photos so that there is no blockage to close the door: you don't want to close while the cat is in the tunnel.
A phone is used instead of a camera because this is what I had!!!

![Cat door setup](docs/cat-door.jpg)




Training ran separately on a Mac - A simple binary classifier.

The current installation is a **Raspberry Pi 3 running 64-bit Raspberry Pi OS**,
with an old Nexus 5X camera. An iPhone app is also included.

In normal operation the pi will boot, start door.py and listen for magenet even - To be noted that the magnet baselines itself: this is important as
if the magnet is moved and such it would change calibration data. so routinely the app self calibrate (when there is no cat).

## Everyday commands

Open the controls at **[http://picat.local:8080](http://picat.local:8080)**.
On the phone, use `http://picat:8080`, `http://picat.local:8080`, or the Pi's LAN IP,
whichever resolves on that device.

On the Pi:

```sh
# Restart the controller and website
sudo systemctl restart catdoor

# Check the service
systemctl status catdoor

# Follow controller events and inference scores
tail -f ~/door.log

# Follow everything, including HTTP requests and Python errors
tail -f ~/door-script.log
```

**Restarting lowers the door to establish its closed position.** That startup
cycle does not request a photo or run an obstruction check. The model warms up
in a background thread while the door lowers; web controls start after lowering
and settling finish. Manual website holds are cleared on restart.

## How the door behaves

| Situation | Behavior |
|---|---|
| Magnet detected while closed | Opens; website Close does not disable sensor opening. |
| Normal automatic closing | Waits the minimum-open time, checks recent activity, then requests a fresh photo. |
| Model says unobstructed | Closing can proceed after rechecking activity and schedule changes. |
| Obstruction, missing phone, or inference error | Stays open and retries later. |
| Website **Open** | Opens and holds open; no inference is needed. |
| Website **Close** | Clears the open hold and requests a camera-checked close, then resumes automatic control. |
| Enabled daily schedule | Opens for the configured interval. A website hold takes priority until the next schedule boundary. |

The website Open hold lasts until Close, the next schedule boundary,
or a controller restart. With scheduling disabled, it lasts until another
website command or restart. Commands received during movement are retained and
applied after the current movement finishes. Website Open during a pending camera
check cancels that close.

There is **no maximum-open timeout**. A blocked closing check does not force the
door closed or restart the controller. Unexpected control-loop errors still exit
the process, and systemd restarts it.

The displayed door state is the controller's commanded state, not a measurement
from a position sensor. Startup lowering establishes the reference position.

## Configuration

Edit the constants near the top of [`door.py`](door.py), then restart the service.
Python does not reload edits automatically.

| Setting | Current value | Meaning |
|---|---|---|
| `KEEP_OPEN_START` | `"21:20"` | Start of the daily keep-open interval. |
| `KEEP_OPEN_END` | `"18:00"` | End of the daily keep-open interval. |
| `OPEN_MIN_TIME` | `20` | Seconds after opening and settling before automatic closing is eligible. |
| `CLOSE_CHECK_INTERVAL` | `10` | Minimum seconds after a closing check finishes before requesting another photo. |


To keep the door open from 08:00 to 18:00, set those two times explicitly. Times
use the Pi's local timezone and 24-hour `HH:MM` format. An end equal to or earlier
than the start disables the schedule; intervals do not wrap overnight.

At a schedule boundary, either website hold expires. The end boundary also
starts a fresh minimum-open timer. The control loop runs every 1.5 seconds, so
the actual interval between photos includes loop timing, camera capture/upload,
and inference in addition to `CLOSE_CHECK_INTERVAL`.

Motor timings are currently 13 seconds opening plus 3 seconds settling, and
16 seconds lowering plus 2 seconds settling. These are configured in the movement
methods and depend on this particular mechanism.

## Phone camera

Install either the [Android app](android/README.md) or the
[iPhone app](ios/README.md), enter the Pi server address, frame/crop the doorway,
select **Pull**, and tap **Start**.

- **Pull:** the phone waits for a Pi request, takes a fresh photo, and uploads it to the pi (there is a websocket kept oopen)
  This is the mode required for automatic obstruction checks.
- **Push:** the phone takes photos periodically, useful for gathering training
  images. Android offers intervals from 1 to 60 seconds; iPhone uses 10 seconds.

The Android app runs capture in a foreground service, so the screen can sleep.
At night, from 18:00 through 07:59 in phone local time, it switches on the camera
light, allows **1.5 seconds** for exposure to settle, and keeps the light on until
the JPEG arrives. It switches off before processing/upload and stays off between
captures. Stopping the service or a capture timeout releases the camera and light.
The iPhone app needs to remain active and keeps its screen awake.

The apps do not save pictures to Photos or Google Photos. Uploads are stored on
the Pi in `camera_images/`. **Each successful Pull upload prunes storage to the
latest five photos and their metadata.** Push uploads accumulate until the next
Pull cleanup. Copy useful training examples to the Mac before they are removed.

On the website:

- **Take photo** requests a new capture.
- **Latest photo** displays the most recent upload.
- **Run inference** requests a new photo and shows the classification without
  moving the door. It works with the door raised or lowered, but not during
  movement or settling. It does not add an analytics event.

App installation, builds, permissions, and crop controls are documented in the
[Android](android/README.md) and [iPhone](ios/README.md) directories.

## Obstruction model

[`door_inference.py`](door_inference.py) requests a photo and checks the exact
uploaded filename associated with that request. It never substitutes an older
cached photo. [`fat_cat_model/inference.py`](fat_cat_model/inference.py) loads the
committed PyTorch checkpoint once and reuses it. Startup warm-up uses a synthetic
image whose result is discarded; an early real check waits for the model lock.

The model produces an **obstruction score** between 0 and 1. Scores at or above
the threshold stored in the checkpoint mean obstructed. The current threshold
is approximately **0.817265**. These scores are uncalibrated: 0.9 does not mean a
measured 90% probability of obstruction.

Each real prediction, including website checks, logs its label, score, and
threshold to both `~/door.log` and `~/door-script.log`:

```text
inference: result=unobstructed score=0.582405 threshold=0.817265
```

The current model uses RGB images letterboxed to 384×384, a frozen
MobileNetV3-Small backbone, 2×2 average pooling, and a trained linear head.
It correctly classifies the 53 images used for model selection. That is tuning
performance, not an independent test; one darkened obstruction was missed in
additional brightness/contrast/JPEG checks. A single camera does not reliably
measure height above the floor. Details are in
[`fat_cat_model/model-report.json`](fat_cat_model/model-report.json).

To classify a saved photo on the Pi:

```sh
~/door-venv/bin/python ~/door/fat_cat_model/inference.py /path/to/photo.jpg
```

This standalone command imports PyTorch and loads the model every time. A measured
Pi 3 run took about 18 seconds to load, followed by roughly 0.5 seconds per warmed
prediction, including image loading and preprocessing. The running service reuses
the loaded model. Phone capture and upload add separate latency.

## Training on the Mac

Training code is in [`training/`](training/README.md). Photos remain outside the
repository under `~/Documents/code/CATDOOR_TRAINING/`:

```text
samples/
  obstructed/
  unobstructed/
validation/
  obstructed/
  unobstructed/
```

Run the current model search from the Mac:

```sh
cd ~/Documents/PI3/door/training
~/Documents/code/CATDOOR_TRAINING/.venv/bin/python improve_margin.py
```

Each run writes a new timestamped directory under `CATDOOR_TRAINING/training-runs/`.
It does not replace the deployed weights. The [training README](training/README.md)
explains selection, robustness checks, dependencies, and path overrides.
The deployed `fat_cat_model/model.pt` and its report are committed; photo datasets,
virtual environments, APKs, and generated training runs are not.

## Activity analytics

The website's **Activity graph** computes reports on demand from
`door_events.sqlite3`. Choose **Live events** for current activity or **2023 logs**
for the archive, an end date, and daily or hour-of-day buckets. Reports cover six
calendar months and show total inferred transits, daily average, and busiest buckets.
The default archive view is July–December 2023.

A magnet-triggered opening counts as one inferred `TRANSIT`. While already open,
including during scheduled or manual opening, a new magnet burst also counts
once after eight seconds of quiet. Movement and settling are excluded. Closing
never adds a transit. Direction is unknown, and fast return trips can merge.
`OPEN` and `CLOSE` record completed motor commands, including startup lowering.
Camera requests, inference scores, and model warm-up do not count as activity.

The previously imported 2023 archive remains in the database, separate from live
events. The one-time import script has been removed. Empty buckets mean no
recorded events, not proof that the cat was inactive.

## Pi setup and service

The current installation uses `/home/pi/door` for this repository and
`/home/pi/door-venv` for its Python environment. Hardware:

| Component | Connection / role |
|---|---|
| MLX90393 | I²C magnetic field sensor. |
| Motor relay | BCM GPIO 27, inverted polarity. |
| Direction relay | BCM GPIO 17. |
| Optional break-beam sensor | BCM GPIO 22; disabled in normal startup. |
| Phone camera | Wi-Fi HTTP uploads; no wired camera connection. |

Enable I²C on the Pi and give the service user access to the GPIO/I²C devices.
The restored installation uses 64-bit Raspberry Pi OS and Python 3.13. Its runtime
dependencies are NumPy, RPi.GPIO, Adafruit Blinka, the Adafruit MLX90393 library,
PyTorch, torchvision, and Pillow. Standard-library modules need no installation.
Use the existing environment for routine operation; a fresh environment can be
prepared with:

```sh
python3 -m venv ~/door-venv
~/door-venv/bin/python -m pip install --upgrade pip
~/door-venv/bin/python -m pip install numpy RPi.GPIO adafruit-blinka adafruit-circuitpython-mlx90393
~/door-venv/bin/python -m pip install -r ~/door/fat_cat_model/requirements.txt \
  --extra-index-url https://download.pytorch.org/whl/cpu
```

Install the included systemd unit after placing the repository and environment
at those paths:

```sh
sudo cp ~/door/catdoor.service /etc/systemd/system/catdoor.service
sudo systemctl daemon-reload
sudo systemctl enable --now catdoor
```

The unit starts the controller at boot and restarts it after an exit. It already
runs the website; no separate web process is needed. Stop the service before
launching `door.py` manually, so two processes do not control the same hardware.
The website/API have no login and are intended for the local network.

## HTTP API

JSON commands use `Content-Type: application/json` and `{}` as the request body.
For example:

```sh
curl -H 'Content-Type: application/json' -d '{}' http://picat.local:8080/open
curl -H 'Content-Type: application/json' -d '{}' http://picat.local:8080/close
curl -H 'Content-Type: application/json' -d '{}' http://picat.local:8080/camera/inference
```

| Method | Path | Purpose |
|---|---|---|
| `GET` | `/` | Door state and controls. |
| `POST` | `/open`, `/close` | Hold open or request closing; return `open`, `busy`, `keep_open`, `close_pending`, and `keep_closed` (always false). |
| `POST` | `/camera/capture` | Queue a fresh phone capture; return its request ID. |
| `GET` | `/camera/request?id=...` | Check capture status and matching image metadata. |
| `GET` | `/camera/command` | Phone long-poll for a capture command. |
| `POST` | `/camera` | Phone JPEG upload, with capture timestamp and optional request ID headers. |
| `GET` | `/camera/latest`, `/camera/latest.jpg` | Latest metadata or JPEG. |
| `POST` | `/camera/inference` | Fresh-photo check; return `clear_to_close` and `mock: false`. |
| `GET` | `/analytics` | Six-month activity report. |

Only one capture can be pending at a time. Camera uploads are limited to 2 MiB.
GET requests do not move the door. A successful Close request can remain pending
while leaving the door open if the camera check blocks closure; inspect the state
and logs rather than treating HTTP success as confirmation of physical position.

## Troubleshooting

| Symptom | Check |
|---|---|
| Inference takes about 30 seconds, then fails | The Pi is likely waiting for a photo. Check phone power, Wi-Fi, address, Pull mode, and Start. |
| CLI inference starts slowly | Each invocation reloads PyTorch; the service keeps it loaded. |
| Door stays open | Check the website hold, schedule, sensor activity, and inference result in `door.log`. |
| Light stays off during the day | Lighting is scheduled by phone local time; night is 18:00–08:00. |
| Website is briefly unavailable after restart | Startup lowering and settling finish before web controls start. |
| A useful photo disappeared | Pull retention keeps five photos; copy training examples promptly. |

## Code map and offline checks

| File / directory | Responsibility |
|---|---|
| `door.py` | Hardware, movement, schedule, manual holds, control loop. |
| `door_web.py` | Standard-library HTTP server and HTML controls. |
| `door_camera.py`, `pull_photo.py` | Upload storage, capture commands, and request matching. |
| `door_inference.py`, `fat_cat_model/` | Cached classifier, warm-up, and closing checks. |
| `door_analytics.py` | Live events and reports, including the existing archive. |
| `android/`, `ios/` | Native phone camera apps. |
| `training/` | Mac training and model evaluation. |
| `catdoor.service` | Boot/start/restart configuration. |

The controller tests use fake hardware and clocks; they do not operate the door:

```sh
cd ~/door
~/door-venv/bin/python -m unittest discover -s . -p 'test_*.py' -v
```

The model tests are separate in `training/test_classifier.py`. Tests do not replace
physical checks of camera exposure, motor behavior, or obstruction classification.

Website Close never holds the door closed: magnet opening resumes after closing.
An active keep-open schedule can also reopen it on the next control-loop iteration.
