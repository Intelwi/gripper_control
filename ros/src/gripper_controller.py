#!/usr/bin/env python

import rospy
from std_msgs.msg import String
#from mcr_manipulation_msgs.msg import GripperCommand
from std_msgs.msg import String
import serial_interface

"""
This module contains a component that receives commands for the gripper and forwards them to the Teensy board.
"""

class GripperController:
    def __init__(self):
        rospy.init_node('gripper_controller', anonymous=True)
        self.serial_msg = serial_interface.SerialInterface(9600, 1, "239A")
        self.serial_msg.open_port()

        self.cmd_listener = rospy.Subscriber('/arm_1/gripper_command', String, self.callback)
        self.feedback_publisher = rospy.Publisher('/arm_1/gripper_feedback', String)

    def callback(self, data):
        command = int(data.data)
        json_command = {
                "command": 0,
        }

        if command == 0:
            print('Closing the gripper.')
            json_command['command'] = 1

        else:
            print('Opening the gripper.')

        self.serial_msg.send(json_command)
    
    def handle_msg(self):
        msg = self.serial_msg.receive()
        self.feedback_publisher.pub(msg)

if __name__ == '__main__':

    try:
        gripper_controller = GripperController()
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            gripper_controller.handle_msg()
            rate.sleep()
    
    except rospy.ROSInterruptException as ex:
        rospy.logerr(ex)
