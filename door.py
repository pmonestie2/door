import time
import board
import adafruit_mlx90393
import math
from collections import deque
import RPi.GPIO as GPIO
import threading
import sys

log_file  = open("/home/pi/door.log", "a")

def log(msg, to_file=True):
    _msg = "[%s] %s"%(time.ctime(), msg)
    if to_file:
        log_file.write(_msg + "\n")
        log_file.flush()
    print(_msg)
type_to_logtime = {}
def log_interval(type, msg, interval=10.0, to_file=False):
    if type not in type_to_logtime:
        type_to_logtime[type] = 0
    if time.time() - type_to_logtime[type] > interval:
        log("[%s] %s"%(type, msg), to_file=to_file)
        type_to_logtime[type] = time.time()
class Relay:
    def __init__(self, pin, inverted=False):
        self.pin = pin
        self.inverted = inverted
        GPIO.setup(pin, GPIO.OUT)
        self.trig(False)
    def trig(self, on=True):
        GPIO.output(self.pin, not on if not self.inverted else on)
    def test(self):
        self.trig(False)
        time.sleep(2)
        self.trig(True)
        time.sleep(2)
        self.trig(False)
        time.sleep(2)
        self.trig(True)
        time.sleep(2)
        self.trig(False)

class Beam:
    def __init__(self, pin, break_beam_callback):
        self.pin = pin
        self.break_beam_callback = break_beam_callback
        GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.add_event_detect(pin, GPIO.BOTH, callback=self.internal_break_beam_callback)
        self.time=0
    def internal_break_beam_callback(self, channel=None):
        if GPIO.input(self.pin):
            self.broken=False
            self.time = 0
        else:
            self.broken=True
            self.time=time.time()
            self.break_beam_callback()
    def is_broken(self):
        return not GPIO.input(self.pin)

class Sensor:
    def __init__(self, val_lookback=15):
        self.val_lookback = val_lookback
        self.i2c = board.I2C()  # uses board.SCL and board.SDA
        self.SENSOR = adafruit_mlx90393.MLX90393(self.i2c, gain=adafruit_mlx90393.GAIN_2X)
        self.SENSOR.display_status()
        self.mf_array = deque(maxlen=val_lookback)
        self.df_array = deque(maxlen=val_lookback)

    def read_sensor(self):
        MX, MY, MZ = self.SENSOR.magnetic
        mfield = math.sqrt(MX**2 + MY**2 + MZ**2)
        dfield = [MX/mfield, MY/mfield, MZ/mfield]
        if self.SENSOR.last_status > adafruit_mlx90393.STATUS_OK:
            self.SENSOR.display_status()
        self.mf_array.append(mfield)
        self.df_array.append(dfield)
        log_interval("field", "magnetic field: %s, drirection field %s"%(str(mfield),str(dfield)), interval=20)
        return mfield,dfield
    def check_div(self, min_length=None):
        lookup_size = min_length if min_length is not None else self.val_lookback
        if len(self.mf_array) < lookup_size:
            return False
        for i in range(len(self.mf_array) - lookup_size, lookup_size):
            for j in range(i + 1, len(self.mf_array)):
                if abs(self.mf_array[i] - self.mf_array[j]) > 2.0:
                    return False
        for i in range(len(self.df_array) - 1):
            for j in range(i + 1, len(self.df_array)):
                for k in range(3):
                    if abs(self.df_array[i][k] - self.df_array[j][k]) > 2.0:
                        return False
        return True
    def clear(self):
        self.mf_array.clear()
        self.df_array.clear()



class Door:
    def __init__(self):
        self.lock=False
        self.open_min_time = 5
        self.cool_down_time = 5
        self.open_time = time.time()
        self.closed_time = time.time()
        self.open = None
        self.sensor = Sensor(val_lookback = 15)
        (self.m1, self.d1) = self.sensor.read_sensor()
        self.thread = threading.Thread(target=self.read_sensor_thread)
        self.motor = Relay(27, inverted=True)
        self.direction = Relay(17)
        self.beam = Beam(22, self.open_door_from_beam)



    def compute_change(self, m2,d2):
        pcm = 100 * (m2 - self.m1) / self.m1
        dot_product = sum([self.d1[i] * d2[i] for i in range(3)])
        pcd = 100 * (1 - dot_product)
        return pcm,pcd


    def reset_sensor_if_stable(self, m2, d2, min_legth = None, force_log=False):
        if self.sensor.check_div(min_length = min_legth):
            self.m1 = m2
            self.d1 = d2
            sz_lookback  = len(self.sensor.mf_array)
            self.sensor.clear()
            msg = "reset sensor with mf = %s df = %s - size lookback= %s"%(str(m2),str(d2),sz_lookback)
            if force_log:
                log(msg)
            else:
                log_interval("reset_sensor", msg, interval=20)

    def magnet_detected(self, verbose=False):
        (m2, d2) = self.sensor.read_sensor()
        self.reset_sensor_if_stable(m2, d2)
        (m_change, d_change) = self.compute_change(m2, d2)
        if verbose:
            log_interval("magnet_change", "magnet changes detected: magnitude= %s , dir=%s"%(m_change, d_change), interval=1.5, to_file=False)
        if abs(m_change) > 8 or abs(d_change) > 8:
            return True
        return False

    def magnet_wait_detect(self, min_check=3, interval=1.0, verbose=False):
        v=[]
        for i in range(min_check):
            v.append(self.magnet_detected(verbose=verbose))
            time.sleep(interval)
        if all(v):
            return 1
        if not any(v):
            return 0
        return 2

    def close_door(self):
        if not self.open or self.lock:
            return
        log("door closing")
        self.lock=True
        self.open = False
        self.direction.trig(False)
        self.motor.trig(True)
        time.sleep(16)
        self.motor.trig(False)
        self.direction.trig(True)
        self.closed_time = time.time()
        (m2, d2) = self.sensor.read_sensor()
        ##reset sensor base this way, by reading for a bit - This is b/c door moves disturb sensor
        for i in range(10):
            self.sensor.read_sensor()
            time.sleep(.05)
        self.reset_sensor_if_stable(m2, d2, 8, force_log=True)
        log("door closed")
        self.lock = False

    def open_door_from_beam(self):
        self.open_door(source="beam")

    def open_door(self, time_to_open=12, source="magnet"):
        if self.open or self.lock:
            return
        log("door opening source =%s" %source)
        self.lock = True
        self.open = True
        self.direction.trig(True)
        self.motor.trig(True)
        time.sleep(time_to_open)
        self.motor.trig(False)
        log("door opened source=%s" %source)
        self.open_time = time.time()
        self.lock = False


    def read_sensor_thread(self):
        while True:
            if self.open and time.time() - self.open_time > self.open_min_time:
                if self.magnet_wait_detect(3, .2, verbose=True) == 0 and not self.beam.is_broken():
                    self.close_door()
                else:
                    log_interval("magnet_detect", "magnet still detected", interval=1.5, to_file=False)
            else:
                is_magnet = self.magnet_wait_detect(3, .1) == 1
                if is_magnet and not self.open:
                    if self.cool_down_time < time.time() - self.closed_time:
                        self.open_door()
                    else:
                        log("detected magnet but hit cooldown time", to_file=False)
            time.sleep(.08)

if __name__ == '__main__':
    door = Door()
    if len(sys.argv)>1 and sys.argv[1] == "inter":
        pass
    else:
        log("closing door and starting door thread")
        door.open = True
        door.close_door()
        door.thread.start()
    #while True:
    #    print(sensor.magnet_detected(verbose=True))
    #    time.sleep(.2)
    ##sensor.thread.start()
