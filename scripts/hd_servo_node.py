#! /usr/bin/python2

__author__ = 'lethic'

import rospy
import serial
from hd_servo.msg import *

#####################
#radians:
#degrees: -82           0             82
#pulse:   448          1415          2382
# 11.8 pulse = 1 degree
#####################

class hdServoNode:
    def __init__(self):
        try:
            self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout = 1)
        except:
            print "Cannot connect to the robot"

        self.pub = rospy.Publisher("/hd_servo/motor_states", MotorStateList, queue_size = 4)
        self.sub = rospy.Subscriber("/hd_servo/motor_sequence_point", MotorSequencePoint, self.motorSequencePointCallback)

        #publish motor states at 100hz
        rospy.Timer(rospy.Duration(0.01), self.statesPub)

    def statesPub(self, event):
        self.ser.write('QP\r')
        results = self.ser.readline()
        states_msg = MotorStateList()
        if(len(results) == 13):
            for i in range(12):
                motor_state = MotorState()
                motor_state.joint_id = i
                motor_state.pulse = ord(results[i]) * 10
                motor_state.degrees = (motor_state.pulse - 448) / 11.8 - 82
                motor_state.radians = motor_state.degrees / 180.0 * 3.14
                states_msg.motor_states.append(motor_state)
            self.pub.publish(states_msg)

        try:
            self.ser.inWaiting()
        except:
            print "Lost connection!"

    def motorSequencePointCallback(self, msg):
        commands = ''
        for command in msg.motor_commands:
            m_id = command.joint_id
            m_pos = (command.position / 3.14 * 180.0 + 82) * 11.8 + 448 #radians to pulse
            if command.invert:
                m_pos = 1415 - (m_pos - 1415)
            m_time = command.time
            commands += '#' + str(m_id) + ' P' + str(int(m_pos)) + ' T' + str(int(m_time)) + ' '
        commands += '\r'
        print commands
        self.ser.write(commands)

if __name__ == '__main__':
    rospy.init_node('hd_servo_node')
    hd_servo_node = hdServoNode()
    rospy.spin()
