##Introduction  
ROS package for Arduino servo sequence control.

##Background
When we want to control multiple-joint robots, we always need speed control, position feedback, and group servo control. High-end servos such as Dynamixel has such functionalities and it comes with ROS package. However, for simple hobby servos driven by PWM, it is hard to do that. I found a firmware (http://www.lamja.com/?p=504) which allows the functionalities mentioned above by using only Arduino and PWM servos. This repository is a ROS action node wrapper for this firmware. And it
enables high-level sequence control for servos. 

##Usage
* Install the firmware file to your Arduino. And connect servos to your Arduino, connect your Arduino to the PC or Raspberry pi which will run the ROS node.
* Run the scripts in the script folder:
```
./hd_servo_node.py
./hd_servo_sequence_action.py
./hd_servo_sequence_action_client_gui.py
```

##Credits
The Arduino firmware comes from: http://www.lamja.com/?p=504

##License
You are free to use it for any purpose at your own risk.
