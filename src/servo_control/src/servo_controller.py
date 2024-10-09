#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
import serial

class ServoController:
    def __init__(self):
        self.servo1_position = 0
        self.servo2_position = 0

        self.ser = serial.Serial('/dev/ttyACM0', 9600)

        rospy.Subscriber('/servo1_position', Float64, self.servo1_callback)
        rospy.Subscriber('/servo2_position', Float64, self.servo2_callback)

    def servo1_callback(self, msg):
        self.servo1_position = msg.data
        self.send_position_to_arduino(1, self.servo1_position)

    def servo2_callback(self, msg):
        self.servo2_position = msg.data
        self.send_position_to_arduino(2, self.servo2_position)

    def send_position_to_arduino(self, servo_number, position):
        command = f"{servo_number},{position}\n"
        self.ser.write(command.encode())

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('servo_controller')
    controller = ServoController()
    controller.run()
