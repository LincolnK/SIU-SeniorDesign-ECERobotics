import RPi.GPIO as GPIO

class Motor(object):
    """Class for managing pwm/direction motor control on raspberry pi"""
    def __init__(self, pwm, direction, frequency=500):
        self._pwm_line = pwm
        self._frequency = frequency
        self._direction_line = direction
        self._dc = 0
        self._speed=0
        self._dir = True
        self._pwm = None
    def start(self):
        if not GPIO.getmode():#should maybe just error instead if not set up?
            GPIO.setmode(GPIO.BCM)
        GPIO.setup([self._pwm_line, self._direction_line], GPIO.OUT)
        self._pwm = GPIO.PWM(self._pwm_line, self._frequency)
        self._pwm.start(self._dc)
        GPIO.output(self._direction_line, True)
    def __enter__(self):
        self.start()
        return self
    def __exit__(self, e_type, value, traceback):
        self._pwm.stop()
        GPIO.cleanup([self._pwm_line, self._direction_line])
    def _set_direction(self, d):
        if d != self._dir:
            self._dir = d
            GPIO.output(self._direction_line, d)
    @property
    def forward(self):
        return self._dir
    @forward.setter
    def forward(self, value):
        self._set_direction(bool(value))
    @property
    def backward(self):
        return not self._dir
    @backward.setter
    def backward(self, value):
        self._set_direction(not bool(value))
    def reverse(self):
        self._set_direction(not self._dir)
    def drive(self, speed, forward=True, duration=None):
        self.speed = speed
        self.forward = forward
    @property
    def speed(self):
        return self._dc
    @speed.setter
    def speed(self, value):
        value = float(value)
        direction = (value>=0)
        if (value != self._speed) or (direction != self._dir):
            self._speed=value
            self.forward = direction
            self._dc = abs(value)
            print self._dc
            self._pwm.ChangeDutyCycle(self._dc)
            
