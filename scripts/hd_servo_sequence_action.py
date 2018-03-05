#! /usr/bin/python2

__author__ = 'lethic'

import rospy
import actionlib
from hd_servo.msg import *

#####################
#radians:
#degrees: -82           0             82
#pulse:   448          1415          2382
# 11.8 pulse = 1 degree
#####################

class hdServoSeqServer:
    def __init__(self):
        self.TimeOut = 120.0
        self.pub = rospy.Publisher("/hd_servo/motor_sequence_point", MotorSequencePoint, queue_size=4)
        self.sub = rospy.Subscriber("/hd_servo/motor_states", MotorStateList, self.motorStateListCallback)
        self.motor_positions = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        self.MOTOR_ERROR = 0.05
        self.server = actionlib.SimpleActionServer('hd_servo_sequence_action_server', hd_sequenceAction, self.execute, False)
        self.server.start()

    def motorStateListCallback(self, msg):
        for i, motor_state in enumerate(msg.motor_states):
                self.motor_positions[i] = motor_state.radians

    def pointProcessed(self, pointID):
        if pointID < 0:
            return True
        if not self.jntSeq.points[pointId].block:
            return True
        for i, cmd in self.jntSeq.points[frameId].motor_commands:
            if cmd.position - self.motor_positions[mmap[cmd.joint_name]] > self.MOTOR_ERROR:
                return False
        return True

    def execute(self, goal):
        print 'start execute'
        self.jntSeq=goal.motor_sequence
        frameCount = 0
        numPoints = len(self.jntSeq.points)
        r = rospy.Rate(50)
        processed=False
        tme = rospy.Time.now()
        pointCount = 0
        while not processed:
            if pointCount >= numPoints:
                processed = True
            elif (rospy.Time.now() - tme > rospy.Duration().from_sec(self.TimeOut)):
                processed = True
                print 'time out'
            else:
                if pointCount < numPoints and self.pointProcessed(pointCount-1):
                    print 'start move motors'
                    self.pub.publish(self.jntSeq.points[pointCount])
                    pointCount = pointCount + 1
            r.sleep()
        self.server.set_succeeded()

if __name__ == '__main__':
    rospy.init_node('hd_servo_seqence_action_node')
    server = hdServoSeqServer()
    rospy.spin()
