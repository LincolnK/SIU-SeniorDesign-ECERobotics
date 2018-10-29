"""
    Provides a helper class to read and listen for changes to IR sensor on Rapspberry PI GPIO
"""
import RPi.GPIO as GPIO


class Sensor(object):
    """
        Watches a single sensor and makes its value available

        Args:
            line (int): GPIO pin used by sensor

        Attributes:
            value (boolean)a: Current value of the sensor
    """
    def __init__(self,line):
        self._line = line
        self._current=False
    def start(self):
        """
            initialize IO
        """
        if not GPIO.getmode():
            GPIO.setmode(GPIO.BCM)
            self._current = GPIO.input(self._line)
        GPIO.setup(line,GPIO.IN)
        GPIO.add_event_detect(self._line,GPIO.BOTH)
        GPIO.add_event_callback(self._line,self._cb)
    @property
    def value(self):
        return self._current
    def _cb(self,channel):
        self._current = GPIO.input(self._line)
    def __enter__(self):
        """ContextManager enter"""
        self.start()
        return self
    def __exit__(self,e_type,value,traceback):
        """ContextManager exit"""
        GPIO.cleanup(self._line)
    def wait(self,edge=GPIO.BOTH):
        """
            Thread waits until the sensor value changes

            Args:
                edge (int): Which edge to wait for. By default triggers on GPIO.BOTH
        """
        GPIO.wait_for_edge(self._line,edge)
    def listen(self,cb):
        """
            Add a new listener that will be called when the sensor changes

            Args:
                cb (Callable[[string]]): Callback that will be called whenever the sensor changes
        """
        GPIO.add_event_callback(self._line,cb)



