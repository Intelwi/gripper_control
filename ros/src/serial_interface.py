"""
This module contains a component that communicates with
the particular Teensy board and sends the message via
serial port with specified baudrate, timeout and board pid.
"""
import serial
import json

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
        msg_str = self.arduino.readline()
        
        msg_str = msg_str.decode().strip()
        

        if len(msg_str) < 5 or not '{' in msg_str:
            return None

        if '}{' not in msg_str:
            return [json.loads(msg_str)]

        msgs = msg_str.split("}{")
        
        for i in range(len(msgs)):
            if '{' not in msgs[i]:
                msgs[i] = '{' + msgs[i]
            if '}' not in msgs[i]:
                msgs[i] = msgs[i] + '}'
            
        msgs_json = []

        for msg in msgs:
            msgs_json.append(json.loads(msg))

        return msgs_json
