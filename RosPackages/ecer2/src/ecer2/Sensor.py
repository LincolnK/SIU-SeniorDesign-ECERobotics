import RPi.GPIO as GPIO


class Sensor(object):
    def __init__(self,line):
        self._line = line
        self._current=None
    def start(self):
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
        self.start()
        return self
    def __exit__(self,e_type,value,traceback):
        GPIO.cleanup(self._line)
    def wait(self,edge=GPIO.BOTH):
        GPIO.wait_for_edge(self._line,edge)
    def listen(self,cb):
        GPIO.add_event_callback(self._line,cb)



