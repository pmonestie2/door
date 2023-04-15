import time
import board
import adafruit_mlx90393
import math
from collections import deque
import RPi.GPIO as GPIO
import threading
import sys

log_file  = open("/home/pi/door.log", "a")

def log(msg):
    log_file.write(msg + "\n")
    log_file.flush()
    print(msg)

class relay:
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

class beam:
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
    def get(self):
        #self.break_beam_callback(None)
        log("beam broken %s"% self.broken)

class Sensor:
    def __init__(self):
        self.lock=False
        self.open_min_time = 5
        self.cool_down_time = 5
        self.open_time = time.time()
        self.closed_time = time.time()
        self.open = None
        self.i2c = board.I2C()  # uses board.SCL and board.SDA
        self.SENSOR = adafruit_mlx90393.MLX90393(self.i2c, gain=adafruit_mlx90393.GAIN_2X)
        self.SENSOR.display_status()
        self.val_lookback = 15
        self.mf_array = deque(maxlen=self.val_lookback)
        self.df_array = deque(maxlen=self.val_lookback)
        print(self.SENSOR)
        (self.m1, self.d1) = self.read_sensor()
        self.thread = threading.Thread(target=self.read_sensor_thread)
        self.motor = relay(27, inverted=True)
        self.direction = relay(17)
        self.beam = beam(22, self.open_door_from_beam)


    def read_sensor(self):
        MX, MY, MZ = self.SENSOR.magnetic
        mfield = math.sqrt(MX**2 + MY**2 + MZ**2)
        dfield = [MX/mfield, MY/mfield, MZ/mfield]
        if self.SENSOR.last_status > adafruit_mlx90393.STATUS_OK:
            self.SENSOR.display_status()
        self.mf_array.append(mfield)
        self.df_array.append(dfield)
        #print every 10 seconds - make sure time is an integer
        if int(time.time()) % 10 == 0:
            print("magnetic field: ", mfield)
            print("direction field: ", dfield)

        return mfield,dfield

    def compute_change(self, m2,d2):
        pcm = 100 * (m2 - self.m1) / self.m1
        dot_product = sum([self.d1[i] * d2[i] for i in range(3)])
        pcd = 100 * (1 - dot_product)
        return pcm,pcd

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

    def reset_sensor_if_stable(self, m2, d2, min_legth = None):
        if self.check_div(min_length = min_legth):
            self.m1 = m2
            self.d1 = d2
            sz = len(self.mf_array)
            self.mf_array.clear()
            self.df_array.clear()
            print("reset sensor with mf = %s df = %s - size lookback= %s"%(str(m2),str(d2),str(sz)))

    def magnet_detected(self, verbose=False):
        (m2, d2) = self.read_sensor()
        self.reset_sensor_if_stable(m2, d2)
        (m_change, d_change) = self.compute_change(m2, d2)
        if verbose:
            print("magnet changes detected, mag, dir=", m_change, d_change)
        if abs(m_change) > 10 or abs(d_change) > 10:
            return True
        return False

    def magnet_wait_detect(self, min_check=3, interval=1, verbose=False):
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
        log("door closing at %s" % time.ctime())
        self.lock=True
        self.open = False
        self.direction.trig(False)
        self.motor.trig(True)
        time.sleep(16)
        self.motor.trig(False)
        self.closed_time = time.time()
        (m2, d2) = self.read_sensor()
        ##reset sensor base this way, by reading for a bit - This is b/c door moves disturb sensor
        for i in range(10):
            self.read_sensor()
            time.sleep(.05)
        self.reset_sensor_if_stable(m2, d2, 8)
        log("door closed at %s" % time.ctime())
        self.lock = False

    def open_door_from_beam(self):
        log("open door from beam: %s" % time.ctime())
        print(self)
        self.open_door()

    def open_door(self, time_to_open=12):
        if self.open or self.lock:
            return
        log("door opening at %s" % time.ctime())
        self.lock = True
        self.open = True
        self.direction.trig(True)
        self.motor.trig(True)
        time.sleep(time_to_open)
        self.motor.trig(False)
        log("door opened at %s" % time.ctime())
        self.open_time = time.time()
        self.lock = False


    def read_sensor_thread(self):
        print(self)
        while True:
            if self.open and time.time() - self.open_time > self.open_min_time:
                if self.magnet_wait_detect(3, .2, verbose=True) == 0:
                    self.close_door()
                else:
                    print("magnet still detected")
            else:
                is_magnet = self.magnet_wait_detect(4, .1) == 1
                if is_magnet and not self.open:
                    if self.cool_down_time < time.time() - self.closed_time:
                        self.open_door()
                    else:
                        print("detectd magnet but hit cooldown time")
            if self.open:
                log("door open at %s" % time.ctime())
            time.sleep(.4)



if __name__ == '__main__':
    sensor = Sensor()
    if len(sys.argv)>1 and sys.argv[1] == "inter":
        pass
    else:
        print("closing door and starting thread")
        sensor.open = True
        sensor.close_door()
        sensor.thread.start()
    #while True:
    #    print(sensor.magnet_detected(verbose=True))
    #    time.sleep(.2)
    ##sensor.thread.start()
