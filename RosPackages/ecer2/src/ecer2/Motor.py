"""
Provides object to send inputs to motor controller
"""
import RPi.GPIO as GPIO

class Motor(object):
    """
        Control class for a motor output on raspberry pi

        Args:
            pwm (int): IO line for pwm output
            direction (int): IO line for direction output
            frequency (int): frequency in hertz used by pwm
    """
    def __init__(self, pwm, direction, frequency=500):
        self._pwm_line = pwm
        self._frequency = frequency
        self._direction_line = direction
        self._dc = 0
        self._speed=0
        self._dir = True
        self._pwm = None
    def start(self):
        """
            Initializes IO
        """
        if not GPIO.getmode():#should maybe just error instead if not set up?
            GPIO.setmode(GPIO.BCM)
        GPIO.setup([self._pwm_line, self._direction_line], GPIO.OUT)
        self._pwm = GPIO.PWM(self._pwm_line, self._frequency)
        self._pwm.start(self._dc)
        GPIO.output(self._direction_line, True)
    def __enter__(self):
        """
            Contextmanager enter
        """
        self.start()
        return self
    def __exit__(self, e_type, value, traceback):
        """
            Contextmanager exit
        """
        self._pwm.stop()
        GPIO.cleanup([self._pwm_line, self._direction_line])
    def _set_direction(self, d):
        if d != self._dir:
            self._dir = d
            GPIO.output(self._direction_line, d)
    @property
    def forward(self):
        """
            Get or set whether motor is moving forward

            Returns:
                bool: True if moving forward
        """
        return self._dir
    @forward.setter
    def forward(self, value):
        self._set_direction(bool(value))
    @property
    def backward(self):
        """
            Get or set whether motor is moving backwards

            Returns:
                bool: True if moving backward
        """
        return not self._dir
    @backward.setter
    def backward(self, value):
        self._set_direction(not bool(value))
    def reverse(self):
        """
            reverse the current direction without changing speed
        """
        self._set_direction(not self._dir)
    def drive(self, speed=0, forward=True, duration=None):
        """
            drive the motor at a set speed

            Args:
                speed (int,optional): Desired speed. Defaults to 0
                forward (int,optional): Desired direction. Defaults to forward (True)
                duration (float,optional): How long to move. Defaults to indefinite(None)
        """
        self.speed = speed
        self.forward = forward
    @property
    def speed(self):
        """
            Gets or sets current speed
            Returns:
                float:Current speed, represented as duty cycle out of 100, where negative numbers are reverse
        """
        return (self._dc if self._dir else -1 * self._dc)
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
