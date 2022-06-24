#!/usr/bin/env python

import rospy
from std_msgs.msg import String
#from mcr_manipulation_msgs.msg import GripperCommand
from std_msgs.msg import String
import serial_interface


class GripperController:
    """ROS wrapper for receiving commands for the gripper and forwarding them to the Teensy board.
    """

    component_name = 'gripper_controller'

    def __init__(self):
        rospy.init_node(component_name, anonymous=True)
        self.serial_msg = serial_interface.SerialInterface(9600, 1, "239A")
        self.serial_msg.open_port()

        self.gripper_command_topic = rospy.get_param('~gripper_command_topic', '/arm_1/gripper_command')
        self.cmd_listener = rospy.Subscriber(self.gripper_command_topic, String, self.callback)

        self.gripper_feedback_topic = rospy.get_param('~gripper_feedback_topic', '/arm_1/gripper_feedback')
        self.feedback_publisher = rospy.Publisher(self.gripper_feedback_topic, String)

    def callback(self, data):
        """Callback for receiving gripper command

            Keyword arguments:
            @param data -- command from the gripper
        """
        command = int(data.data)
        json_command = {
                "command": 0,
        }

        if command == 0:
            rospy.logdebug('Closing the gripper.')
            json_command['command'] = 1

        else:
            rospy.logdebug('Opening the gripper.')

        self.serial_msg.send(json_command)
    
    def handle_msg(self):
        """Function for receiving feedback from serial and publishing to ros
        """
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
