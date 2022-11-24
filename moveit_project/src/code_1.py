#!/usr/bin/env python3

from __future__ import print_function
#from six.moves import input

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi, tau, dist, fabs, cos
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list


class Mazinger():
    def __init__(self):
        # AA
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('move_group_python_interface_tutorial', anonymous=True)
        
        # AA
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()

        # AA 
        self.group_name = "end_effector"
        self.move_group = moveit_commander.MoveGroupCommander(self.group_name)

        # AA
        self.display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )

        # Configuration and initial states
        self.rate = rospy.Rate(10)  # 10hz
        self.ctrl_c = False

        rospy.on_shutdown(self.shutdownhook)

    def move_robot(self):

        while not self.ctrl_c:
            # We can get the name of the reference frame for this robot:
            planning_frame = self.move_group.get_planning_frame()
            print("============ Planning frame: %s" % planning_frame)

            # We can also print the name of the end-effector link for this group:
            eef_link = self.move_group.get_end_effector_link()
            print("============ End effector link: %s" % eef_link)

            # We can get a list of all the groups in the robot:
            group_names = self.robot.get_group_names()
            print("============ Available Planning Groups:", self.robot.get_group_names())

            # Sometimes for debugging it is useful to print the entire state of the
            # robot:
            print("============ Printing robot state")
            print(self.robot.get_current_state())
            print("")
            print('**********************************************************************')

    def shutdownhook(self):
        self.ctrl_c = True  


if __name__ == '__main__':
    #rospy.init_node('code1_kuka', anonymous=True)
    rosbot_object = Mazinger()
    try:
        input("=> Press 'Enter' to print robot Information ...")
        rosbot_object.move_robot()
    except rospy.ROSInterruptException:
        pass