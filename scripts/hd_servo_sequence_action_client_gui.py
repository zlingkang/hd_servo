#!/usr/bin/python2

import rospy
import actionlib

from std_msgs.msg import UInt8
from hd_servo.msg import *

import Tkinter as tk
from PIL import ImageTk, Image

class actionGUI:
    def __init__(self, master):
        self.master = master
        self.servo_names = ['hip_roll_l', 'hip_yaw_l', 'thigh_l', 'knee_l', 'ankle_pitch_l', 'ankle_roll_l',
                'hip_roll_r', 'hip_yaw_r', 'thigh_r', 'knee_r', 'ankle_pitch_r', 'ankle_roll_r']
        self.servo_pos = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

        self.servo_scales = []
        for r, servo_name in enumerate(self.servo_names):
            tk.Label(text = servo_name, width = 10).grid(row = r%6, column = 0 if r<6 else 2)
            servo_scale = tk.Scale(self.servo_pos[r], orient = tk.HORIZONTAL, width = 10, to = 180)
            servo_scale.set(90)
            servo_scale.grid(row = r%6, column = 1 if r<6 else 3)
            self.servo_scales.append(servo_scale)

        r = r%6 + 1
        self.go_button = tk.Button(master, text = "GO", command = self.go_callback)
        self.go_button.grid(row = r, column = 0)

        #self.img = ImageTk.PhotoImage(Image.open("joints_config.png"))
        #self.imglabel = tk.Label(self.master, image=self.img).grid(row = 0, column = 4, rowspan = 20, columnspan = 20, sticky = tk.W+tk.E+tk.N+tk.S, padx = 5, pady = 5);

        self.client = actionlib.SimpleActionClient('hd_servo_sequence_action_server', hd_sequenceAction)
        self.client.wait_for_server()

    def go_callback(self):
        #send the action goal
        goal = hd_sequenceGoal()
        seq = goal.motor_sequence
        seq.header.stamp = rospy.Time.now()

        pts = MotorSequencePoint()
        pts.block = False
        for i, servo_name in enumerate(self.servo_names):
            command = MotorCommand()
            command.joint_id = i
            command.position = (self.servo_scales[i].get()-90.0)/180.0*3.14
            command.time = 10
            command.invert = False
            pts.motor_commands.append(command)
        seq.points.append(pts)
        self.client.send_goal(goal)
        print 'sent!'
        self.client.wait_for_result(rospy.Duration.from_sec(5.0))
        if self.client.get_result():
            print 'done!'


def main():
    root = tk.Tk()
    rospy.init_node('hd_servo_gui')
    app = actionGUI(root)
    root.mainloop()

if __name__ == "__main__":
    main()
