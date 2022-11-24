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

def all_close(goal, actual, tolerance):
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
        x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
        # Euclidean distance
        d = dist((x1, y1, z1), (x0, y0, z0))
        # phi = angle between orientations
        cos_phi_half = fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
        return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)

    return True

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
    
    def plan_display(self):
        # Waypoints
        waypoints = []
        scale = 1

        # Getting Initial Pose
        wpose = self.move_group.get_current_pose().pose

        # First Movement
        wpose.position.z += scale * 0.1  # First move up (z)
        wpose.position.y += scale * 0.1  # and sideways (y)
        wpose.position.x -= scale * 0.05
        waypoints.append(copy.deepcopy(wpose))

        # Second Movement
        wpose.position.z += scale * 0.1  # First move up (z)
        wpose.position.y -= scale * 0.1  # and sideways (y)
        waypoints.append(copy.deepcopy(wpose))

        # Third Movement
        wpose.position.z -= scale * 0.1  # First move up (z)
        wpose.position.y -= scale * 0.1  # and sideways (y)
        waypoints.append(copy.deepcopy(wpose))

        # Fourth Movement
        wpose.position.z -= scale * 0.1  # First move up (z)
        wpose.position.y += scale * 0.1  # and sideways (y)
        wpose.position.x += scale * 0.05
        waypoints.append(copy.deepcopy(wpose))

        # AAA
        (plan, fraction) = self.move_group.compute_cartesian_path(
            waypoints, 0.01, 0.0  # waypoints to follow  # eef_step
        )  # jump_threshold

        # Note: We are just planning, not asking move_group to actually move the robot yet:
        return plan, fraction

    def execute_plan(self, plan):
        self.move_group.execute(plan, wait=True)

    def shutdownhook(self):
        self.ctrl_c = True  


if __name__ == '__main__':
    kuka_object = Mazinger()
    try:
        input("=> Press 'Enter' to Plan and Display a Cartesian Path ...")
        cartesian_plan, fraction = kuka_object.plan_display()
        input("\n=> Press 'Enter' to Execute a Saved Path ...")
        kuka_object.execute_plan(cartesian_plan)


    except rospy.ROSInterruptException:
        pass