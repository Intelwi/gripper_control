#!/usr/bin/env python

## Simple ropod_talker that listens to std_msgs/Strings published
## to the 'circuit_playground' topic

import rospy
from std_msgs.msg import String
#from mcr_manipulation_msgs.msg import GripperCommand
from std_msgs.msg import String
import serial_interface

def callback(data):
    command = int(data.data)
    json_command = {
            "command": 0,
    }

    if command == 0:
        print('Closing the gripper.')
        json_command['command'] = 1

    else:
        print('Opening the gripper.')
        json_command['command'] = 0

    serial_msg.send(json_command)

def robot_listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('/arm_1/gripper_command', String, callback)

if __name__ == '__main__':

    serial_msg = serial_interface.SerialInterface(9600, 1, "239A")
    serial_msg.open_port()
    robot_listener()

    while True:
        msg = serial_msg.receive()
        print(msg)
        rospy.sleep(0.1)
