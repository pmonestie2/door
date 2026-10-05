import time
import math
from collections import deque
import threading
import sys
import numpy as np
import signal
import cProfile
import itertools
import os
from functools import wraps

MAGNET_DETECT_LG_TYPE = 'magnet_detect'

MAGNET_LG_TYPE = 'magnet'

LOG_FILE_PATH = "/home/pi/door.log"
log_file = None

def configure_logging(path=LOG_FILE_PATH):
    """Set up file logging and rotation only when the application starts."""
    global log_file, LOG_FILE_PATH
    LOG_FILE_PATH = path
    log_file = open(path, "a")
    signal.signal(signal.SIGHUP, handle_sighup)

def handle_sighup(signum, frame):
    global log_file
    log_file.close()
    log_file = open(LOG_FILE_PATH, "a")
    log("rotated file")

def log(msg, to_file=True):
    _msg = "[%s] %s"%(time.ctime(), msg)
    if to_file and log_file is not None:
        log_file.write(_msg + "\n")
        log_file.flush()
    print(_msg, flush=True)


type_to_logtime = {}
def silence(type):
    #silence by setting -1
    type_to_logtime[type] = (-1,-1)
def unsilence(type):
    type_to_logtime.pop(type, None)
def log_interval(type, msg, interval=10.0, count = 1, to_file=False):
    """
    log at an interval - interval is different for each type
    """
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
        self.gpio.output(self.pin, not on if not self.inverted else on)
    def test(self):
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
        if self.gpio.input(self.pin):
            self.broken=False
        else:
            self.broken=True
            self.broken_time=time.time()
            self.break_beam_callback()
    def is_broken(self):
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
    """Connect the Pi hardware and return a callable yielding (x, y, z)."""
    import board
    import adafruit_mlx90393

    i2c = board.I2C()
    device = adafruit_mlx90393.MLX90393(i2c, gain=adafruit_mlx90393.GAIN_2X)
    device.display_status()

    def read():
        values = device.magnetic
        if device.last_status > adafruit_mlx90393.STATUS_OK:
            device.display_status()
        return values

    return read


class Sensor:
    def __init__(self, callback, val_lookback=10, *, read_magnetic):
        self.lookback_size = val_lookback
        self.read_magnetic = read_magnetic
        self.value_lookback = deque(maxlen=val_lookback)
        self.avg = None
        self.avg_norm = None
        self.magnet = False
        self.magnet_time = 0
        self.callback = callback
        self.thread = threading.Thread(target=self.run_thread)
        self.calibration_time = 0

    def compute_avg(self, max_std):
        """
        Given the values in a bounded list, value_lookback compute an average for each axis
        std_def of values have to deviate no more than max_std
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
        #calibration is only done every N seconds. this force recalibration
        self.calibration_time = 0

    def read_sensor(self):
        MX, MY, MZ = self.read_magnetic()
        #value read are put in the lookback, to be used also for calibration
        self.value_lookback.append(SensorData(MX, MY, MZ))


    def magnet_detect_lookback(self, arr):
        """
        detect magnet in range - at least 2 values have to be above threshold
        """
        cnt = 0
        for data in arr:
            if self.magnet_detected(data):
                cnt = cnt+1
                if cnt > 1:
                    return True
        return False

    def magnet_detected(self,data:SensorData, verbose=False):
        (m_change, d_change) = self.compute_change(data)
        if abs(m_change) > 5 or abs(d_change) > 5:
            log_interval(MAGNET_DETECT_LG_TYPE, "magnet detected: m_change=%s, d_change=%s"%(str(m_change), str(d_change)),
                         interval=5, count=1, to_file=True)
            return True
        return False

    def compute_change(self, sensorData:SensorData):
        """
        rate of change for direction and magnitude between the avg data stored and the current data in sensorData
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
                        #magnet change detected - call the callback
                        self.callback()
                    else:
                        self.magnet=False
                time.sleep(.08)
            except Exception as e:
                log("exiting as exception" + str(e))
                os._exit(1)


class FakeBeam:
    def is_broken(self):
        return False
    def __init__(self):
        self.broken = False
        self.broken_time = 0

def serialize_movement(method):
    """Ignore commands during a movement instead of queuing a later reversal."""
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
                 clock=time, exit_process=os._exit):
        self.clock = clock
        self.exit_process = exit_process
        self.lock=False
        self.movement_lock = threading.Lock()
        self.open_min_time = 90
        self.open_max_time = 500
        self.cool_down_time = 5
        self.open_time = self.clock.time()
        self.closed_time = self.clock.time()
        self.open = None
        self.sensor = sensor
        self.sensor.callback = self.open_door_from_magnet
        self.thread = threading.Thread(target=self.read_sensor_thread)
        self.motor = motor
        self.direction = direction
        self.beam = FakeBeam() if beam is None else beam


    @serialize_movement
    def close_door(self):
        if not self.open or self.lock:
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
        self.lock = False
        log("door closed")

    def open_door_from_beam(self):
        log_interval("beam", "beam detected", interval=5, count=3, to_file=True)
        self.open_door(source="beam")

    def open_door_from_magnet(self):
        log_interval(MAGNET_LG_TYPE, "magnet detected", interval=5, count = 2, to_file=True)
        self.open_door(source="magnet")


    @serialize_movement
    def open_door(self, time_to_open=13, source="magnet"):
        if self.open or self.lock:
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
        self.lock = False

    def read_sensor_thread(self):
        while True:
            try:
                if self.open and self.clock.time() - self.open_time > self.open_min_time and not self.lock:
                    #figures out if both magnet and door are not active - if so close the door
                    magnet_on = True if self.sensor.magnet or self.clock.time() - self.sensor.magnet_time < 8 else False
                    self.beam.is_broken()
                    beam_on = True if self.beam.broken or self.clock.time() - self.beam.broken_time < 8 else False
                    if not beam_on and not magnet_on:
                        #note that if door is already closed this will do nothing.
                        self.close_door()
                    else:
                        #was open too long - exiting
                        if self.clock.time() - self.open_time > self.open_max_time:
                            log("exiting, open time too long")
                            self.exit_process(1)
                        log_interval("door_close","door can't close - beam/magnet detected %s %s"%(beam_on, magnet_on), interval=5, to_file=True)
                self.clock.sleep(1.5)
            except Exception as e:
                log("exiting as exception" + str(e))
                self.exit_process(1)


def beam_log():
    #print("change")
    pass

def main():
    import RPi.GPIO as GPIO

    configure_logging()
    GPIO.setmode(GPIO.BCM)
    if len(sys.argv)>1 and sys.argv[1] == "beam":
        #cProfile.run('door.sensor.run_thread()', sort='cumtime')
        k = Beam(22, beam_log, gpio=GPIO)
        while True:
            time.sleep(1)
    else:
        sensor = Sensor(callback=None, read_magnetic=create_magnetic_reader())
        door = Door(sensor=sensor, motor=Relay(27, inverted=True, gpio=GPIO),
                    direction=Relay(17, gpio=GPIO))
        log("closing door and starting door thread")
        door.open = True
        door.close_door()
        door.thread.start()
        door.sensor.thread.start()
        from door_web import start_server
        try:
            start_server(door)
            log("web controls listening on port 8080")
        except OSError as error:
            log("web controls unavailable: %s" % error)


if __name__ == '__main__':
    main()
