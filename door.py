import time
import board
import adafruit_mlx90393
import math
from collections import deque
import RPi.GPIO as GPIO
import threading
import sys
import numpy as np
import signal
import cProfile
import itertools

MAGNET_DETECT_LG_TYPE = 'magnet_detect'

MAGNET_LG_TYPE = 'magnet'

LOG_FILE_PATH = "/home/pi/door.log"
log_file  = open(LOG_FILE_PATH, "a")

def handle_sighup(signum, frame):
    global log_file
    log_file.close()
    log_file = open(LOG_FILE_PATH, "a")
    log("rotated file")
signal.signal(signal.SIGHUP, handle_sighup)

def log(msg, to_file=True):
    _msg = "[%s] %s"%(time.ctime(), msg)
    if to_file:
        log_file.write(_msg + "\n")
        log_file.flush()
    print(_msg)


type_to_logtime = {}
def silence(type):
    type_to_logtime[type] = (-1,-1)
def unsilence(type):
    type_to_logtime.pop(type, None)
def log_interval(type, msg, interval=10.0, count = 1, to_file=False):
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
    def __init__(self, pin, inverted=False):
        self.pin = pin
        self.inverted = inverted
        GPIO.setup(pin, GPIO.OUT)
        self.trig(False)
    def trig(self, on=True):
        GPIO.output(self.pin, not on if not self.inverted else on)
    def test(self):
        for i in range(5):
            self.trig(i % 2 == 0)
            time.sleep(2)

class Beam:
    def __init__(self, pin, break_beam_callback):
        self.pin = pin
        self.break_beam_callback = break_beam_callback
        GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.add_event_detect(pin, GPIO.BOTH, callback=self.internal_break_beam_callback)
        self.broken_time=0
    def internal_break_beam_callback(self, channel=None):
        if GPIO.input(self.pin):
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

class Sensor:
    def __init__(self, callback, val_lookback=10):
        self.lookback_size = val_lookback
        self.i2c = board.I2C()  # uses board.SCL and board.SDA
        self.SENSOR = adafruit_mlx90393.MLX90393(self.i2c, gain=adafruit_mlx90393.GAIN_2X)
        self.SENSOR.display_status()
        self.value_lookback = deque(maxlen=val_lookback)
        self.avg = None
        self.avg_norm = None
        self.magnet = False
        self.magnet_time = 0
        self.callback = callback
        self.thread = threading.Thread(target=self.run_thread)
        self.calibration_time = 0

    def compute_avg(self, max_std):
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
        self.calibration_time = 0

    def read_sensor(self):
        MX, MY, MZ = self.SENSOR.magnetic
        if self.SENSOR.last_status > adafruit_mlx90393.STATUS_OK:
            self.SENSOR.display_status()
        self.value_lookback.append(SensorData(MX, MY, MZ))


    def magnet_detect_lookback(self, arr):
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
            self.read_sensor()

            if time.time() - self.calibration_time > 30:
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
                if self.magnet_detect_lookback(list(itertools.islice(self.value_lookback, len(self.value_lookback)-4, None))):
                    self.magnet=True
                    self.magnet_time = time.time()
                    self.callback()
                else:
                    self.magnet=False
            time.sleep(.08)


class Door:
    def __init__(self):
        self.lock=False
        self.open_min_time = 10
        self.cool_down_time = 5
        self.open_time = time.time()
        self.closed_time = time.time()
        self.open = None
        self.sensor = Sensor(val_lookback = 10, callback=self.open_door_from_magnet)
        self.thread = threading.Thread(target=self.read_sensor_thread)
        self.motor = Relay(27, inverted=True)
        self.direction = Relay(17)
        self.beam = Beam(22, self.open_door_from_beam)


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
        time.sleep(16)
        self.motor.trig(False)
        self.direction.trig(True)
        self.closed_time = time.time()
        unsilence(MAGNET_LG_TYPE)
        unsilence(MAGNET_DETECT_LG_TYPE)
        #allow sensor to reset
        self.sensor.force_calibration()
        time.sleep(2)
        self.lock = False
        log("door closed")

    def open_door_from_beam(self):
        log_interval("beam", "beam detected", interval=5, count=3, to_file=True)
        self.open_door(source="beam")

    def open_door_from_magnet(self):
        log_interval(MAGNET_LG_TYPE, "magnet detected", interval=5, count = 2, to_file=True)
        self.open_door(source="magnet")


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
        time.sleep(time_to_open)
        self.motor.trig(False)
        log("door opened source=%s" %source)
        unsilence(MAGNET_LG_TYPE)
        unsilence(MAGNET_DETECT_LG_TYPE)
        self.sensor.force_calibration()
        time.sleep(3)
        self.open_time = time.time()
        self.lock = False

    def read_sensor_thread(self):
        while True:
            if self.open and time.time() - self.open_time > self.open_min_time and not self.lock:
                magnet_on = True if self.sensor.magnet or time.time() - self.sensor.magnet_time < 8 else False
                self.beam.is_broken()
                beam_on = True if self.beam.broken or time.time() - self.beam.broken_time < 8 else False
                if not beam_on and not magnet_on:
                    self.close_door()
                else:
                    log_interval("door_close","door can't close - beam/magnet detected %s %s"%(beam_on, magnet_on), interval=5, to_file=True)
            time.sleep(1)

if __name__ == '__main__':
    door = Door()
    if len(sys.argv)>1 and sys.argv[1] == "inter":
        #cProfile.run('door.sensor.run_thread()', sort='cumtime')
        door.sensor.run_thread()
    else:
        log("closing door and starting door thread")
        door.open = True
        door.close_door()
        door.thread.start()
        door.sensor.thread.start()

