import traceback
import serial
import time
from dataclasses import dataclass
from threading import Event, Thread
from crc import Configuration, Calculator
from typing import Callable, Optional 

import rclpy
from rclpy import logging


def to_int(ba: bytearray) -> int:
    """Converts a little-endian bytearray to an integer."""
    res = 0
    for i in range(len(ba)):
        res += ba[i] << (8 * i)
    return res

PACKET_LEN = 32
PERIOD = 0.010
SOP = 0x55


@dataclass
class PowerCoreMsg:
    packet_id: int = 0
    mission: bool = False
    voltage: float = 0.0
    currents = [0.0]*12


class PowerCore:

    def __init__(self, port: str = '/dev/ttyTHS1', baud: int = 57600, callback: Optional[Callable] = None):
        self.callback : Optional[Callback] = callback
        self.port = port
        self.baud = baud
        self.last_packet : Optional[PowerCoreMsg] = None 
        self.__rxbuf : bytes = b''
        self.logger = rclpy.logging.get_logger('PowerCore')
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
            self.serial = serial.Serial(self.port, self.baud)
            if self.serial.isOpen():
                self.serial.close()
            self.serial.open()
            if self.__stop.is_set():
                self.thread.join()
                self.__stop.clear()
                self.thread = Thread(target=self.__run)
            self.thread.start()
            self.logger.info("Connected")
        except serial.SerialException as e:
            self.logger.error(f"Failed to connect to power core: {traceback.format_exc()}")

    def __run(self):
        """Read serial port data, verify packet, and update local information."""
        while(not self.__stop.is_set()):
            time.sleep(PERIOD)
            self.__rxbuf += self.serial.read(size=PACKET_LEN)

            if len(self.__rxbuf) >= PACKET_LEN and self.__rxbuf[0] == SOP: 
                # A possibly valid packet has been found
                msg = bytearray(self.__rxbuf[:PACKET_LEN - 2])
                crc_exp = to_int(bytearray(self.__rxbuf[-2:]))
                crc_act = self.__crc_calc.checksum(msg)

                if crc_exp == crc_act:
                    self.last_packet = PowerCoreMsg()
                    self.last_packet.packet_id = int(self.__rxbuf[2])
                    self.last_packet.mission = self.__rxbuf[3] > 0
                    self.last_packet.voltage = float(to_int(self.__rxbuf[4:6]))/1000.0

                    for i in range(0, 12):
                        self.last_packet.currents[i] = float(to_int(self.__rxbuf[2*i+6: 2*i+8]))/1000.0
                    self.__rxbuf = self.__rxbuf[-PACKET_LEN:] if len(self.__rxbuf) > PACKET_LEN else b''
                    if self.callback is not None:
                        self.callback(self.last_packet)
                    # print(f"Voltage: {self.voltage:.02f}\tCurrents: {currents}", end="\r")
                else:
                    # CRC failure
                    self.logger.info("CRC failure")
                    self.serial.flushInput()
                    self.__rxbuf = b''
            elif 0 < len(self.__rxbuf) < PACKET_LEN and self.__rxbuf[0] == SOP:
                # Correct start, but not enough data
                continue
            else:
                # Too many bytes or an invalid start of packet
                self.logger.debug("Invalid SOP, flushing input")
                self.serial.flushInput()
                self.__rxbuf = b''

        self.serial.close()

    def close(self):
        """Close the serial port."""
        self.__stop.set() 
        self.thread.join(timeout=0.1)


def rx_callback(msg: PowerCoreMsg):
    currents = ", ".join([f"{i:0.02f}" for i in msg.currents])
    print(f"PID:{msg.packet_id}\tVoltage: {msg.voltage:0.02f}\tMission: {msg.mission}\tCurrents: {currents}")


if __name__=='__main__':
    pc = PowerCore(port='/dev/ttyTHS1', callback=rx_callback)
    pc.connect()

    while(True):
        try:
            time.sleep(1)
        except KeyboardInterrupt as e:
            pc.close()
            exit()
     
