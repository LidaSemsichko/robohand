#! /usr/bin/python3

import rospy
from std_msgs.msg import String

def send_push_spring_command():
    rospy.init_node('push_spring_sender', anonymous=True)
    pub = rospy.Publisher('command_topic', String, queue_size=10)
    rospy.sleep(1)

    command_msg = String()
    command_msg.data = "push_spring"

    rospy.loginfo("Sending command: push_spring")
    pub.publish(command_msg)
    rospy.sleep(1)

if __name__ == "__main__":
    try:
        send_push_spring_command()
    except rospy.ROSInterruptException:
        pass
