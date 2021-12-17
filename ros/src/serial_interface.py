"""
This module contains a component that communicates with
the particular arduino board and sends the message framework via
serial port with specified baudrate, timeout and board pid.
"""
import serial
import os
from time import sleep, time
import json
from std_msgs import msg

class SerialInterface:
    """
    Initialize class with baudrate, timeout, and pid of arduino board
    which is fixed in our case pid = 239A.
    """
    def __init__(self, baud, timeout, pid):
        self.baud = baud
        self.timeout = timeout
        self.pid = pid
    """
    Detect the arduino board and initialize the serial module from pyserial

    """
    def open_port(self):
        port = '/dev/ttyACM0'
        command=os.popen("ls -l " + port[:-1] +"*").read()
        print(command)
        #port="/dev/"+command[command.find('>')+2:].replace("\n","")

        #Opening of the serial port
        try:
            self.arduino = serial.Serial(port, self.baud, timeout=self.timeout)
        except Exception as a:
            print(a)
            print('Please check the port {}'.format(port))

    """
    Send the string message framework to arduino

    """
    def send(self, message):
        self.arduino.flushInput()
        message_str = json.dumps(message)
        message_str = message_str
        self.arduino.write(bytes(message_str.encode()))

    def receive(self):
        """
        The format of the received string
        b'[
            {
                "id":0,
                "current":0,
                "torque":true,
                "velocity":0,
                "position":0,
                "pwm":0,
                "baudrate":57600
            },
            {
                "id":1,
                "current":0,
                "torque":true,
                "velocity":0,
                "position":0,
                "pwm":0,
                "baudrate":57600
            }
        ]\n'
        """
        msg_str = self.arduino.readline()

        msg_str = msg_str.decode().strip()
        
        if len(msg_str) < 5 or not '{' in msg_str:
            return None

        msg_json = json.loads(msg_str)
        return msg_json
