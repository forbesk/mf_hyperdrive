from pca9685_driver import Device
import Jetson.GPIO as GPIO


I2C_BUS = 0
I2C_ADDR = 0x40
EN_PIN = 12
ENABLED = GPIO.LOW


class PCA9685:

    def __init__(self):
        """
        Initialize PCA9685 on the Hyperdrive board. 

        Outputs are disabled on startup.
        """
        self._enabled = False
        self.outputs = [0] * 16

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(EN_PIN, GPIO.OUT)
        self.enabled = False  # Disable on startup

        self.device = Device(I2C_ADDR, bus_number=I2C_BUS)

    @property
    def enabled(self):
        return self._enabled

    @enabled.setter
    def enabled(self, value):
        self._enabled = value
        GPIO.output(EN_PIN, ENABLED if value else not ENABLED)

    @property
    def frequency(self):
        return self.get_pwm_frequency

    @frequency.setter
    def frequency(self, value):
        self.device.set_pwm_frequency(value)
        
    def set_pwm(self, channel, value):
        self.device.set_pwm(channel, value)

