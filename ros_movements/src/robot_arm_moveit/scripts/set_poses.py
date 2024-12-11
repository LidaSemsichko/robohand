#! /usr/bin/python3

from __future__ import print_function
from six.moves import input

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib

try:
    from math import pi, tau, dist, fabs, cos
except:
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi
    
    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))


from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

class MyRobot:

    def __init__(self, Group_Name):

        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('node_set_predefined_pose', anonymous=True)
        
        self._robot = moveit_commander.RobotCommander()
        self._scene = moveit_commander.PlanningSceneInterface()
        
        self._planning_group = Group_Name
        self._group = moveit_commander.MoveGroupCommander(self._planning_group)
        
        self._display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)

        self._exectute_trajectory_client = actionlib.SimpleActionClient('execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self._exectute_trajectory_client.wait_for_server()

        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()

        rospy.loginfo('\033[95m' + "Planning Group: {}".format(self._planning_frame) + '\033[0m')
        rospy.loginfo('\033[95m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
        rospy.loginfo('\033[95m' + "Group Names: {}".format(self._group_names) + '\033[0m')
        rospy.loginfo('\033[95m' + " >>> MyRobot initialization is done." + '\033[0m')

    def set_pose(self, arg_pose_name):
        rospy.loginfo('\033[32m' + "Going to Pose: {}".format(arg_pose_name) + '\033[0m')
        self._group.set_named_target(arg_pose_name)
        
        plan_success, plan, planning_time, error_code = self._group.plan()
        
        goal = moveit_msgs.msg.ExecuteTrajectoryGoal()
        goal.trajectory = plan
        self._exectute_trajectory_client.send_goal(goal)
        self._exectute_trajectory_client.wait_for_result()
        rospy.loginfo('\033[32m' + "Now at Pose: {}".format(arg_pose_name) + '\033[0m')
    
    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[95m' + "Object of class MyRobot Deleted." + '\033[0m')


def send_push_spring_command():
    pub = rospy.Publisher('command_topic', String, queue_size=10)
    rospy.sleep(1)

    command_msg = String()
    command_msg.data = "push_spring"

    rospy.loginfo("Sending command: push_spring")
    pub.publish(command_msg)
    rospy.sleep(1)

def send_rotate_screwdriver_command():
    pub = rospy.Publisher('command_topic', String, queue_size=10)
    rospy.sleep(1)

    command_msg = String()
    command_msg.data = "rotate_screwdriver"

    rospy.loginfo("Sending command: rotate_screwdriver")
    pub.publish(command_msg)
    rospy.sleep(1)

def take_first_screw(arm, instrument_changer):
    arm.set_pose("go_up1")
    rospy.sleep(1)
    arm.set_pose("pick_screw1")
    rospy.sleep(1)
    arm.set_pose("go_up1")
    rospy.sleep(1)
    arm.set_pose("preparing_to_put1")
    rospy.sleep(1)
    arm.set_pose("put_screw1")
    rospy.sleep(1)
    send_rotate_screwdriver_command()
    rospy.sleep(3)
    arm.set_pose("preparing_to_put1")

def take_second_screw(arm, instrument_changer):
    arm.set_pose("go_up2")
    rospy.sleep(1)
    arm.set_pose("pick_screw2")
    rospy.sleep(1)
    arm.set_pose("go_up2")
    rospy.sleep(1)
    arm.set_pose("preparing_to_put2")
    rospy.sleep(1)
    arm.set_pose("put_screw2")
    rospy.sleep(1)
    send_rotate_screwdriver_command()
    rospy.sleep(3)
    arm.set_pose("preparing_to_put2")

def take_third_screw(arm, instrument_changer):
    arm.set_pose("go_up3")
    rospy.sleep(1)
    arm.set_pose("pick_screw3")
    rospy.sleep(1)
    arm.set_pose("go_up3")
    rospy.sleep(1)
    arm.set_pose("preparing_to_put3")
    rospy.sleep(1)
    arm.set_pose("put_screw3")
    rospy.sleep(1)
    send_rotate_screwdriver_command()
    rospy.sleep(3)
    arm.set_pose("preparing_to_put3")

def take_fourth_screw(arm, instrument_changer):
    arm.set_pose("go_up4")
    rospy.sleep(1)
    arm.set_pose("pick_screw4")
    rospy.sleep(1)
    arm.set_pose("go_up4")
    rospy.sleep(1)
    arm.set_pose("preparing_to_put4")
    rospy.sleep(1)
    arm.set_pose("put_screw4")
    rospy.sleep(1)
    send_rotate_screwdriver_command()
    rospy.sleep(3)
    arm.set_pose("preparing_to_put4")

def put_paste(arm, instrument_changer):
    arm.set_pose("preparing_for_paste")
    rospy.sleep(1)
    arm.set_pose("put_paste")
    rospy.sleep(1)
    send_push_spring_command()
    rospy.sleep(3)
    arm.set_pose("preparing_for_paste")
    rospy.sleep(3)
    arm.set_pose("default_pose")

def solder(arm, instrument_changer):
    arm.set_pose("prepare_for_solder")
    rospy.sleep(1)
    arm.set_pose("put_solder")
    rospy.sleep(1)
    arm.set_pose("prepare_for_solder")
    rospy.sleep(1)
    arm.set_pose("default_pose")

def main():
    arm = MyRobot("main_arm")
    instrument_changer = MyRobot("instrument_changer")
    
    arm.set_pose("default_pose")
    rospy.sleep(1)
    
    instrument_changer.set_pose("default_instrument")
    rospy.sleep(1)

    while not rospy.is_shutdown():
        take_first_screw(arm, instrument_changer)
        rospy.sleep(1)
        take_second_screw(arm, instrument_changer)
        rospy.sleep(1)
        take_third_screw(arm, instrument_changer)
        rospy.sleep(1)
        take_fourth_screw(arm, instrument_changer)
        rospy.sleep(1)
        arm.set_pose("default_pose")
        rospy.sleep(1)
        instrument_changer.set_pose("spring_tool")
        rospy.sleep(1)
        put_paste(arm, instrument_changer)
        rospy.sleep(1)
        arm.set_pose("default_pose")
        rospy.sleep(1)
        instrument_changer.set_pose("soldering_tool")
        rospy.sleep(1)
        solder(arm, instrument_changer)
        # send_push_spring_command()

        break

    del arm
    del instrument_changer

if __name__ == '__main__':
    main()


