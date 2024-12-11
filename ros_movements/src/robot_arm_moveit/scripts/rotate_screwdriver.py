#! /usr/bin/python3

import rospy
from std_msgs.msg import String

def send_rotate_screwdriver_command():
    rospy.init_node('rotate_screwdriver_sender', anonymous=True)
    pub = rospy.Publisher('command_topic', String, queue_size=10)
    rospy.sleep(1)  # Wait for connection

    command_msg = String()
    command_msg.data = "rotate_screwdriver"

    rospy.loginfo("Sending command: rotate_screwdriver")
    pub.publish(command_msg)
    rospy.sleep(1)  # Allow time for processing

if __name__ == "__main__":
    try:
        send_rotate_screwdriver_command()
    except rospy.ROSInterruptException:
        pass
