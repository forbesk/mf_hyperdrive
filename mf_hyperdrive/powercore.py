import traceback
import serial
from threading import Event, Thread
from crc import Configuration, Calculator
import rclpy
from rclpy import logging
import time


def to_int(ba: bytearray) -> int:
    """Converts a little-endian bytearray to an integer."""
    res = 0
    for i in range(len(ba)):
        res += ba[i] << (8 * i)
    return res

class PowerCore:

    PACKET_LEN = 32
    PERIOD = 0.010
    SOP = 0x55

    def __init__(self, port: str, baud: int = 57600):
        self.mission : bool = False
        self.currents : List[float] = [0.0]*12
        self.voltage : float = 0.0
        self.port = port
        self.baud = baud
        self.last_packet = 0
        self.__rxbuf : bytes = b''
        self.logger = logging.get_logger(__name__)
        self.thread : Thread = Thread(target=self.__run)
        self.__stop : Event = Event()

        self.__crc_config = Configuration(
                width=16,
                polynomial=0x1021,
                init_value=0xffff,
                final_xor_value=0x0000,
                reverse_input=False,
                reverse_output=False,
        )
        self.__crc_calc = Calculator(self.__crc_config)


    def connect(self):
        self.logger.info("Connecting to power core..") 

        if self.thread.is_alive():
            self.logger.warn("Cannot connect to power core: already connected")
        try:
            self.serial = serial.Serial(self.port, self.baud, timeout=PERIOD)
            self.serial.open()
            if self.__stop.set():
                self.thread.join()
                self.__stop.clear()
                self.thread = Thread(target=self.__run)
            self.thread.start()
        except SerialException as e:
            self.logger.error(f"Failed to connect to power core: {traceback.format_exc()}")

    def __run(self):
        """Read serial port data, verify packet, and update local information."""
        while(not self.__stop.is_set()):
            self.__rxbuf += self.serial.read(size=PACKET_LEN)

            if len(self.__rxbuf) >= PACKET_LEN and self.__rxbuf[0] == SOP: 
                # A possibly valid packet has been found
                if self.__crc_calc.verify(bytearray(self.__rxbuf[:PACKET_LEN - 2]), bytearray(self.__rxbuf[-2:])):
                    self.last_packet = int(self.__rxbuf[1])
                    self.mission = bool(self.__rxbuf[2])
                    self.voltage = float(to_int(self.__rxbuf[3:5]))/1000.0
                    for i in range(0, 2*12, 2):
                        self.currents[i] = float(to_int(self.__rxbuf[i+5: i+7]))/1000.0
                    self.__rxbuf = self.__rxbuf[-PACKET_LEN:] if len(self.__rxbuf) > PACKET_LEN else b''

                    self.logger.info(f"Packet: {self.last_packet}\tVoltage: {self.voltage:.02f}")
                else:
                    # CRC failure
                    self.serial.flushInput()
                    self.__rxbuf = b''
            elif len(self.__rxbuf) < PACKET_LEN and self.__rxbuf[0] == SOP:
                # Correct start, but not enough data
                continue
            else:
                # Too many bytes or an invalid start of packet
                self.serial.flushInput()
                self.__rxbuf = b''

        self.serial.close()

    def close(self):
        """Close the serial port."""
        self.__stop.set() 
        self.thread.join(timeout=0.1)


if __name__=='__main__':
    pc = PowerCore(port='/dev/ttyACM0')
    pc.connect()

    while(True):
        try:
            time.sleep(1)
        except KeyboardInterrupt as e:
            pc.close()
            exit()
     
