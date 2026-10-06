import time
import math
from collections import deque
from datetime import date, timedelta
import threading
import sys
import numpy as np
import signal
import cProfile
import itertools
import os
from functools import wraps
from door_inference import check_before_close

MAGNET_DETECT_LG_TYPE = 'magnet_detect'

MAGNET_LG_TYPE = 'magnet'

LOG_FILE_PATH = "/home/pi/door.log"
# Daily keep-open range in the Pi's local time (24-hour HH:MM).
# An end time <= the start time disables scheduled opening.
KEEP_OPEN_START = "07:20"
KEEP_OPEN_END = "18:00"
log_file = None

def configure_logging(path=LOG_FILE_PATH):
    """Set up file logging and rotation only when the application starts."""
    global log_file, LOG_FILE_PATH
    LOG_FILE_PATH = path
    log_file = open(path, "a")
    signal.signal(signal.SIGHUP, handle_sighup)

def handle_sighup(signum, frame):
    """Reopen the log file after rotation."""
    global log_file
    log_file.close()
    log_file = open(LOG_FILE_PATH, "a")
    log("rotated file")

def log(msg, to_file=True):
    """Print a timestamped message and optionally write it to the configured log."""
    _msg = "[%s] %s"%(time.ctime(), msg)
    if to_file and log_file is not None:
        log_file.write(_msg + "\n")
        log_file.flush()
    print(_msg, flush=True)


type_to_logtime = {}
def silence(type):
    """Mark a message type as silenced."""
    #silence by setting -1
    type_to_logtime[type] = (-1,-1)
def unsilence(type):
    """Restore logging for a message type."""
    type_to_logtime.pop(type, None)
def log_interval(type, msg, interval=10.0, count = 1, to_file=False):
    """Rate-limit messages separately for each type, allowing count per interval."""
    if type not in type_to_logtime:
        type_to_logtime[type] = (0, 0)
    if type_to_logtime[type] is (-1, -1):
        #silenced
        return
    (last_log_time, cnt) = type_to_logtime[type]
    current_time = time.time()
    is_time_ellapsed = current_time - last_log_time > interval
    is_count_under = cnt<count
    if is_time_ellapsed or is_count_under:
        log("[%s] %s"%(type, msg), to_file=to_file)
        if is_time_ellapsed:
            new_count=1
            new_time=time.time()
        else:
            new_count = cnt +1
            new_time = last_log_time
        type_to_logtime[type] = (new_time, new_count)

class Relay:
    def __init__(self, pin, inverted=False, *, gpio):
        self.gpio = gpio
        self.pin = pin
        self.inverted = inverted
        self.gpio.setup(pin, self.gpio.OUT)
        self.trig(False)
    def trig(self, on=True):
        """Set the relay state using its configured polarity."""
        self.gpio.output(self.pin, not on if not self.inverted else on)
    def test(self):
        """Toggle the relay five times with two-second pauses."""
        for i in range(5):
            self.trig(i % 2 == 0)
            time.sleep(2)

class Beam:
    def __init__(self, pin, break_beam_callback, *, gpio):
        self.gpio = gpio
        self.pin = pin
        self.break_beam_callback = break_beam_callback
        self.gpio.setup(pin, self.gpio.IN, pull_up_down=self.gpio.PUD_UP)
        self.gpio.add_event_detect(pin, self.gpio.BOTH, callback=self.internal_break_beam_callback)
        self.broken_time=0
    def internal_break_beam_callback(self, channel=None):
        """Refresh beam state and notify the controller when the beam is blocked."""
        if self.gpio.input(self.pin):
            self.broken=False
        else:
            self.broken=True
            self.broken_time=time.time()
            self.break_beam_callback()
    def is_broken(self):
        """Refresh beam state and invoke its callback if blocked; returns no value."""
        return self.internal_break_beam_callback()

class SensorData:
    def __init__(self, x, y ,z):
        self.x = x
        self.y = y
        self.z = z
        self.time = time.time()
        self.pc_norm = None
        self.angle_change = None

def create_magnetic_reader():
    """Connect and initialize the Pi's magnetic sensor.

    Returns:
        Callable: A reader returning an (x, y, z) tuple of magnetic field readings.
    """
    import board
    import adafruit_mlx90393

    i2c = board.I2C()
    device = adafruit_mlx90393.MLX90393(i2c, gain=adafruit_mlx90393.GAIN_2X)
    device.display_status()

    def read():
        """Returns:
            tuple: Current (x, y, z) magnetic field readings.
        """
        values = device.magnetic
        if device.last_status > adafruit_mlx90393.STATUS_OK:
            device.display_status()
        return values

    return read


class Sensor:
    def __init__(self, on_magnet_detected, val_lookback=10, *, read_magnetic):
        self.lookback_size = val_lookback
        self.read_magnetic = read_magnetic
        self.value_lookback = deque(maxlen=val_lookback)
        self.avg = None
        self.avg_norm = None
        self.magnet = False
        self.magnet_time = 0
        self.on_magnet_detected = on_magnet_detected
        self.thread = threading.Thread(target=self.run_thread)
        self.calibration_time = 0

    def compute_avg(self, max_std):
        """Returns:
            tuple or None: Mean (x, y, z) readings when the lookback is full and
            every axis has standard deviation <= max_std; otherwise None.
        """
        if len(self.value_lookback)<self.lookback_size:
            return

        x_values = [data.x for data in self.value_lookback]
        y_values = [data.y for data in self.value_lookback]
        z_values = [data.z for data in self.value_lookback]
        if np.std(x_values) <= max_std and np.std(y_values) <= max_std and np.std(z_values) <= max_std:
            avg_x = np.mean(x_values)
            avg_y = np.mean(y_values)
            avg_z = np.mean(z_values)
            return avg_x, avg_y, avg_z
        else:
            return None
    def force_calibration(self):
        """Allow recalibration on the next stable window of sensor readings."""
        #calibration is only done every N seconds. this force recalibration
        self.calibration_time = 0

    def read_sensor(self):
        """Append one magnetic field reading to the bounded lookback."""
        MX, MY, MZ = self.read_magnetic()
        #value read are put in the lookback, to be used also for calibration
        self.value_lookback.append(SensorData(MX, MY, MZ))


    def magnet_detect_lookback(self, arr):
        """Returns:
            bool: Whether at least two supplied readings exceed a detection threshold.
        """
        cnt = 0
        for data in arr:
            if self.magnet_detected(data):
                cnt = cnt+1
                if cnt > 1:
                    return True
        return False

    def magnet_detected(self,data:SensorData, verbose=False):
        """Returns:
            bool: Whether magnitude changes by over 5% or direction by over 9 degrees.
        """
        (m_change, d_change) = self.compute_change(data)
        if abs(m_change) > 5 or abs(d_change) > 5:
            log_interval(MAGNET_DETECT_LG_TYPE, "magnet detected: m_change=%s, d_change=%s"%(str(m_change), str(d_change)),
                         interval=5, count=1, to_file=True)
            return True
        return False

    def compute_change(self, sensorData:SensorData):
        """Returns:
            tuple[float, float]: Absolute magnitude change as a percentage of the
            baseline, and direction change as a percentage of 180 degrees.
        """
        if sensorData.pc_norm != None:
            return sensorData.pc_norm, sensorData.angle_change
        #if True:
        #    return (0.0,0.0)
        v1=self.avg
        norm1 = self.avg_norm

        v2 = np.array([sensorData.x, sensorData.y, sensorData.z])
        norm2 = np.linalg.norm(v2)

        pc_norm = abs(100 * (norm2 - norm1) / norm1)
        csm = np.dot(v1, v2) / (norm1 * norm2)
        angle = np.arccos(csm)
        angle_change = (angle / np.pi) * 100
        sensorData.pc_norm = pc_norm
        sensorData.angle_change = angle_change
        return pc_norm,angle_change

    def run_thread(self):
        """Continuously sample, calibrate, and notify on detected magnetic changes."""
        for i in range(0, self.lookback_size):
            self.read_sensor()
        while True:
            try:
                self.read_sensor()
                if time.time() - self.calibration_time > 30:
                    #recalibrate
                    avg = self.compute_avg(.9)
                    if avg is not None:
                        (x,y,z) = avg
                        self.avg = np.array([x,y,z])
                        self.avg_norm = np.linalg.norm(self.avg)
                        self.calibration_time = time.time()
                        for s in self.value_lookback:
                            s.pc_norm = None
                            s.angle_change = None
                        log_interval("reset magnet", "x=%s, y=%s, z=%s"%(avg), interval = 60)
                if self.avg is not None:
                    #slice the lookback from the last 4 and detect at least 2 changes
                    if self.magnet_detect_lookback(list(itertools.islice(self.value_lookback, len(self.value_lookback)-4, None))):
                        self.magnet=True
                        self.magnet_time = time.time()
                        # Notify the controller that a magnet was detected
                        self.on_magnet_detected()
                    else:
                        self.magnet=False
                time.sleep(.08)
            except Exception as e:
                log("exiting as exception" + str(e))
                os._exit(1)


class FakeBeam:
    def is_broken(self):
        """Returns:
            bool: Always False because beam detection is disabled.
        """
        return False
    def __init__(self):
        self.broken = False
        self.broken_time = 0

def serialize_movement(method):
    """Returns:
        Callable: A wrapped method that ignores commands while another movement
        holds the lock, instead of queuing a later reversal.
    """
    @wraps(method)
    def run(self, *args, **kwargs):
        if not self.movement_lock.acquire(blocking=False):
            return
        try:
            return method(self, *args, **kwargs)
        finally:
            self.movement_lock.release()
    return run


class Door:
    def __init__(self, *, sensor, motor, direction, beam=None,
                 clock=time, exit_process=os._exit,
                 keep_open_start="00:00", keep_open_end="00:00", analytics=None,
                 close_check=check_before_close):
        """Configure hardware, timing, and the fresh-photo check used before closing."""
        self.clock = clock
        self.close_check = close_check
        self.next_close_check = 0
        self.open_request_version = 0
        self.analytics = analytics
        self.exit_process = exit_process
        self.keep_open_start = self.parse_schedule_time(keep_open_start)
        self.keep_open_end = self.parse_schedule_time(keep_open_end)
        self.schedule_event = self.latest_schedule_event()
        self.website_closed = False
        self.lock=False
        self.movement_lock = threading.Lock()
        self.open_min_time = 90
        self.open_max_time = 500
        self.cool_down_time = 5
        self.open_time = self.clock.time()
        self.closed_time = self.clock.time()
        self.open = None
        self.sensor = sensor
        self.sensor.on_magnet_detected = self.open_door_from_magnet
        self.thread = threading.Thread(target=self.run_control_loop)
        self.motor = motor
        self.direction = direction
        self.beam = FakeBeam() if beam is None else beam

    @staticmethod
    def parse_schedule_time(value):
        """Returns:
            int: Minutes since midnight parsed from a 24-hour HH:MM string.

        Raises:
            ValueError: If the string is not a valid HH:MM time.
        """
        if len(value) != 5 or value[2] != ":" or not (value[:2] + value[3:]).isdigit():
            raise ValueError("Schedule times must use 24-hour HH:MM format")
        hour, minute = map(int, value.split(":"))
        if not (0 <= hour < 24 and 0 <= minute < 60):
            raise ValueError("Schedule times must be between 00:00 and 23:59")
        return hour * 60 + minute

    def is_scheduled_open(self):
        """Returns:
            bool: Whether local time is in the enabled daily keep-open range,
            including the start and excluding the end.
        """
        event = self.latest_schedule_event()
        return event is not None and event[1] == self.keep_open_start

    def schedule_description(self):
        """Returns:
            str: The daily HH:MM range, or "Disabled" for a nonpositive interval.
        """
        if self.keep_open_end <= self.keep_open_start:
            return "Disabled"
        start_hour, start_minute = divmod(self.keep_open_start, 60)
        end_hour, end_minute = divmod(self.keep_open_end, 60)
        return f"{start_hour:02d}:{start_minute:02d}–{end_hour:02d}:{end_minute:02d}"

    def latest_schedule_event(self):
        """Returns:
            tuple[date, int] or None: Date and minute of the most recent schedule
            boundary, or None when scheduling is disabled.
        """
        if self.keep_open_end <= self.keep_open_start:
            return None
        now = self.clock.localtime(self.clock.time())
        today = date(now.tm_year, now.tm_mon, now.tm_mday)
        minute = now.tm_hour * 60 + now.tm_min
        if minute >= self.keep_open_end:
            return today, self.keep_open_end
        if minute >= self.keep_open_start:
            return today, self.keep_open_start
        return today - timedelta(days=1), self.keep_open_end

    def update_schedule_state(self):
        """Expire manual Close at a schedule boundary and reset timers at its end.

        Returns:
            bool: Whether the current schedule period calls for keeping the door open.
        """
        event = self.latest_schedule_event()
        if event != self.schedule_event:
            self.schedule_event = event
            self.website_closed = False
            if event is not None and event[1] == self.keep_open_end:
                self.open_time = self.clock.time()
        return event is not None and event[1] == self.keep_open_start

    def close_door(self, force=False, source="automatic"):
        """Request closing; website Close overrides sensors until the next schedule event."""
        if source == "website":
            self.update_schedule_state()
            self.website_closed = True
        self._close_door(force, source)

    @serialize_movement
    def _close_door(self, force=False, source="automatic"):
        """Run a closing cycle, honoring website override or forced startup homing."""
        if not self.open or self.lock:
            return
        if not force and not self.website_closed and self.is_scheduled_open():
            return
        if self.clock.time() < self.next_close_check:
            return
        open_request_version = self.open_request_version
        open_time = self.open_time
        try:
            log("requesting fresh phone photo and running inference before closing")
            allowed = self.close_check()
        except Exception as error:
            log("door remains open: photo/inference check failed: %s" % error)
            allowed = False
        self.next_close_check = self.clock.time() + 5
        if allowed is not True:
            log("door remains open: photo/inference check did not allow closing")
            return
        # The capture can take thirty seconds; reconsider events during that wait.
        scheduled_open = self.update_schedule_state()
        if open_request_version != self.open_request_version:
            return
        if not force and not self.website_closed:
            if scheduled_open or self.has_recent_activity():
                return
            if self.open_time != open_time and self.clock.time() - self.open_time <= self.open_min_time:
                return
        log("door closing")
        silence(MAGNET_LG_TYPE)
        silence(MAGNET_DETECT_LG_TYPE)

        self.lock=True
        self.open = False
        self.direction.trig(False)
        self.motor.trig(True)
        self.clock.sleep(16)
        self.motor.trig(False)
        self.direction.trig(True)
        self.closed_time = self.clock.time()
        unsilence(MAGNET_LG_TYPE)
        unsilence(MAGNET_DETECT_LG_TYPE)
        #allow sensor to reset
        self.sensor.force_calibration()
        self.clock.sleep(2)
        self.record_movement("CLOSE", source)
        self.lock = False
        log("door closed")

    def open_door_from_beam(self):
        """Request opening in response to a blocked beam."""
        log_interval("beam", "beam detected", interval=5, count=3, to_file=True)
        self.open_door(source="beam")

    def open_door_from_magnet(self):
        """Record magnet visits while open, then request opening if needed."""
        if self.analytics is not None:
            try:
                self.analytics.magnet(self.clock.time(),
                                      blocked=self.lock or not self.open or self.website_closed)
            except Exception as error:
                log("analytics unavailable: %s" % error)
        log_interval(MAGNET_LG_TYPE, "magnet detected", interval=5, count = 2, to_file=True)
        self.open_door(source="magnet")


    def open_door(self, time_to_open=13, source="magnet"):
        """Request opening; website requests clear the keep-closed override."""
        if source == "website":
            self.open_request_version += 1
            self.update_schedule_state()
            self.website_closed = False
        self._open_door(time_to_open, source)

    @serialize_movement
    def _open_door(self, time_to_open=13, source="magnet"):
        """Run the opening cycle, then start the normal minimum-open timer."""
        if self.website_closed or self.open or self.lock:
            return
        log("door opening source =%s" %source)
        silence(MAGNET_LG_TYPE)
        silence(MAGNET_DETECT_LG_TYPE)

        self.lock = True
        self.open = True
        self.direction.trig(True)
        self.motor.trig(True)
        self.clock.sleep(time_to_open)
        self.motor.trig(False)
        log("door opened source=%s" %source)
        unsilence(MAGNET_LG_TYPE)
        unsilence(MAGNET_DETECT_LG_TYPE)
        self.sensor.force_calibration()
        self.clock.sleep(3)
        self.open_time = self.clock.time()
        self.record_movement("OPEN", source)
        self.lock = False

    def record_movement(self, kind, source):
        """Record completed movement without letting a logging failure stop control."""
        if self.analytics is not None:
            try:
                self.analytics.movement_finished(self.clock.time(), kind, source)
            except Exception as error:
                log("analytics unavailable: %s" % error)

    def has_recent_activity(self):
        """Returns:
            bool: Whether a sensor is active or was active within the last eight seconds.
        """
        self.beam.is_broken()
        now = self.clock.time()
        magnet_active = self.sensor.magnet or now - self.sensor.magnet_time < 8
        beam_active = self.beam.broken or now - self.beam.broken_time < 8
        return magnet_active or beam_active

    def update_door_state(self):
        """Apply schedule changes, then manual Close, scheduled opening, or normal closing."""
        scheduled_open = self.update_schedule_state()
        if self.lock:
            return
        if self.website_closed:
            self.close_door()
            return
        if scheduled_open:
            self.open_door(source="schedule")
            return
        if not self.open:
            return

        elapsed = self.clock.time() - self.open_time
        if elapsed <= self.open_min_time:
            return
        if not self.has_recent_activity():
            self.close_door()
        elif elapsed > self.open_max_time:
            log("exiting, open time too long")
            self.exit_process(1)
        else:
            log_interval("door_close", "door cannot close: recent sensor activity",
                         interval=5, to_file=True)

    def run_control_loop(self):
        """Update the door state, then wait 1.5 seconds before checking again."""
        while True:
            try:
                self.update_door_state()
                self.clock.sleep(1.5)
            except Exception as error:
                log("exiting as exception: %s" % error)
                self.exit_process(1)


def beam_log():
    """Ignore beam events in the standalone beam-check mode."""
    #print("change")
    pass

def main():
    """Initialize hardware and logging, then start the selected operating mode."""
    import RPi.GPIO as GPIO

    configure_logging()
    GPIO.setmode(GPIO.BCM)
    if len(sys.argv)>1 and sys.argv[1] == "beam":
        #cProfile.run('door.sensor.run_thread()', sort='cumtime')
        k = Beam(22, beam_log, gpio=GPIO)
        while True:
            time.sleep(1)
    else:
        from door_analytics import Analytics
        analytics = None
        try:
            analytics = Analytics()
        except Exception as error:
            log("analytics unavailable: %s" % error)
        sensor = Sensor(on_magnet_detected=None, read_magnetic=create_magnetic_reader())
        door = Door(sensor=sensor, motor=Relay(27, inverted=True, gpio=GPIO),
                    direction=Relay(17, gpio=GPIO),
                    keep_open_start=KEEP_OPEN_START, keep_open_end=KEEP_OPEN_END,
                    analytics=analytics)
        from door_web import start_server
        try:
            start_server(door)
            log("web controls listening on port 8080")
        except OSError as error:
            log("web controls unavailable: %s" % error)
        log("checking phone before startup closing and starting door thread")
        door.open = True
        door.close_door(force=True)
        door.thread.start()
        door.sensor.thread.start()


if __name__ == '__main__':
    main()
