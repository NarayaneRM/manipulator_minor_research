#!/usr/bin/env python3

import rospy
import os
from std_msgs.msg import String
import random

# KUKA API for ROS
version = 'V15032017'

# Utility functions for colored output
def cl_black(msge): return '\033[30m'+msge+'\033[0m'
def cl_red(msge): return '\033[31m'+msge+'\033[0m'
def cl_green(msge): return '\033[32m'+msge+'\033[0m'
def cl_orange(msge): return '\033[33m'+msge+'\033[0m'
def cl_blue(msge): return '\033[34m'+msge+'\033[0m'
def cl_purple(msge): return '\033[35m'+msge+'\033[0m'
def cl_cyan(msge): return '\033[36m'+msge+'\033[0m'
def cl_lightgrey(msge): return '\033[37m'+msge+'\033[0m'
def cl_darkgrey(msge): return '\033[90m'+msge+'\033[0m'
def cl_lightred(msge): return '\033[91m'+msge+'\033[0m'
def cl_lightgreen(msge): return '\033[92m'+msge+'\033[0m'
def cl_yellow(msge): return '\033[93m'+msge+'\033[0m'
def cl_lightblue(msge): return '\033[94m'+msge+'\033[0m'
def cl_pink(msge): return '\033[95m'+msge+'\033[0m'
def cl_lightcyan(msge): return '\033[96m'+msge+'\033[0m'

class kuka_iiwa_ros_node:
    def __init__(self):
        self.JointPosition = ([None, None, None, None, None, None, None], None)
        self.ToolPosition = ([None, None, None, None, None, None], None)
        self.ToolForce = ([None, None, None], None)
        self.ToolTorque = ([None, None, None], None)
        self.isReady = True #Corrigir!
        self.isFinished = False
        self.isCompliance = (False, None)
        self.isReadyToMove = (False, None)
        self.isCollision = (False, None)
        self.isMastered = (False, None)
        self.OperationMode = (None, None)
        self.OperatorAck = False

        os.system('clear')
        print(cl_pink('\n=========================================='))
        print(cl_pink('<   <  < << SHEFFIELD ROBOTICS >> >  >   >'))
        print(cl_pink('=========================================='))
        print(cl_pink(' KUKA API for ROS'))
        print(cl_pink(' Client Version: ' + version))
        print(cl_pink('==========================================\n'))

        rospy.Subscriber("JointPosition", String, self.JointPosition_callback)
        rospy.Subscriber("ToolPosition", String, self.ToolPosition_callback)
        rospy.Subscriber("ToolForce", String, self.ToolForce_callback)
        rospy.Subscriber("ToolTorque", String, self.ToolTorque_callback)
        rospy.Subscriber("isCompliance", String, self.isCompliance_callback)
        rospy.Subscriber("isCollision", String, self.isCollision_callback)
        rospy.Subscriber("isMastered", String, self.isMastered_callback)
        rospy.Subscriber("OperationMode", String, self.OperationMode_callback)
        rospy.Subscriber("isReadyToMove", String, self.isReadyToMove_callback)
        rospy.Subscriber("OperatorAck", String, self.OperatorAck_callback)
        rospy.Subscriber("isFinished", String, self.isFinished_callback)

        self.pub_kuka_command = rospy.Publisher('kuka_command', String, queue_size=10)

        rospy.init_node('kuka_iiwa_client_' + str(random.randrange(0, 100)), anonymous=False)
        self.rate = rospy.Rate(100)

    def send_command(self, command_str):
        self.pub_kuka_command.publish(command_str)
        self.rate.sleep()

    def JointPosition_callback(self, data):
        try:
            self.JointPosition = (
                [float(x) for x in data.data.split(']')[0][1:].split(', ')],
                float(data.data.split(']')[1])
            )
        except Exception as e:
            rospy.logerr(f"Error parsing JointPosition: {e}")

    def isCompliance_callback(self, data):
        try:
            d = str(data.data).split()
            self.isCompliance = (d[0] == 'True', float(d[1]))
        except Exception as e:
            rospy.logerr(f"Error parsing isCompliance: {e}")

    def OperatorAck_callback(self, data):
        try:
            d = str(data.data).split()
            self.OperatorAck = (d[0] == 'True', float(d[1]))
        except Exception as e:
            rospy.logerr(f"Error parsing OperatorAck: {e}")

    def isReadyToMove_callback(self, data):
        try:
            d = str(data.data).split()
            self.isReadyToMove = (d[0] == 'True', float(d[1]))
        except Exception as e:
            rospy.logerr(f"Error parsing isReadyToMove: {e}")

    def isFinished_callback(self, data):
        try:
            d = str(data.data).split()
            self.isFinished = (d[0] == 'True', float(d[1]))
        except Exception as e:
            rospy.logerr(f"Error parsing isFinished: {e}")

    def isCollision_callback(self, data):
        try:
            d = str(data.data).split()
            self.isCollision = (d[0] == 'True', float(d[1]))
        except Exception as e:
            rospy.logerr(f"Error parsing isCollision: {e}")

    def isMastered_callback(self, data):
        try:
            d = str(data.data).split()
            self.isMastered = (d[0] == 'True', float(d[1]))
        except Exception as e:
            rospy.logerr(f"Error parsing isMastered: {e}")

    def OperationMode_callback(self, data):
        try:
            d = str(data.data).split()
            self.OperationMode = (d[0], float(d[1]))
        except Exception as e:
            rospy.logerr(f"Error parsing OperationMode: {e}")

    def ToolPosition_callback(self, data):
        try:
            self.ToolPosition = (
                [float(x) for x in data.data.split(']')[0][1:].split(', ')],
                float(data.data.split(']')[1])
            )
        except Exception as e:
            rospy.logerr(f"Error parsing ToolPosition: {e}")

    def ToolForce_callback(self, data):
        try:
            self.ToolForce = (
                [float(x) for x in data.data.split(']')[0][1:].split(', ')],
                float(data.data.split(']')[1])
            )
        except Exception as e:
            rospy.logerr(f"Error parsing ToolForce: {e}")

    def ToolTorque_callback(self, data):
        try:
            self.ToolTorque = (
                [float(x) for x in data.data.split(']')[0][1:].split(', ')],
                float(data.data.split(']')[1])
            )
        except Exception as e:
            rospy.logerr(f"Error parsing ToolTorque: {e}")

if __name__ == "__main__":
    try:
        node = kuka_iiwa_ros_node()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
