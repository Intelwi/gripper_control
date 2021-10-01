#!/usr/bin/env python

## Simple ropod_talker that listens to std_msgs/Strings published
## to the 'circuit_playground' topic

import rospy
from std_msgs.msg import String
from mcr_manipulation_msgs.msg import GripperCommand
import playground_interface
command_to_sent = 0

command_map = {0: '2', 1: '7'}

def callback(data,cmdtosent):
    #if cmdtosent!=data.command:
    command_to_sent=command_map[data.command]
    #rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.command)
    serial_msg.send_command(command_to_sent)
    # https://answers.ros.org/question/231492/passing-arguments-to-callback-in-python/
    #else:
    #    ("The message was already sent")

def robot_listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber('/arm_1/gripper_command', GripperCommand, callback,(command_to_sent))

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':

    serial_msg = playground_interface.SerialInterface(9600,1,"239A")
    serial_msg.open_port()
    robot_listener()
