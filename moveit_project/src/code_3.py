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
    
    def move_robot(self):
        pose_goal = geometry_msgs.msg.Pose()
        print('Init Pose')
        print(pose_goal)
        print('---------------------------')

        pose_goal.orientation.x = 0
        pose_goal.orientation.y = 0.70712
        pose_goal.orientation.z = 0
        pose_goal.orientation.w = 0.70712
        
        pose_goal.position.x = 0.1546
        pose_goal.position.y = -0.1965
        pose_goal.position.z = 0.9636

        print('Final Pose')
        print(pose_goal)
        print('---------------------------')

        self.move_group.set_pose_target(pose_goal)
        success = self.move_group.go(wait=True)

        # Calling `stop()` ensures that there is no residual movement
        self.move_group.stop()
        # It is always good to clear your targets after planning with poses.
        self.move_group.clear_pose_targets()

        # AAAAA
        current_pose = self.move_group.get_current_pose().pose
        return all_close(pose_goal, current_pose, 0.01)

    def shutdownhook(self):
        self.ctrl_c = True  


if __name__ == '__main__':
    rosbot_object = Mazinger()
    try:
        input("=> Press 'Enter' to execute a movement using a Pose Goal ...")
        rosbot_object.move_robot()
    except rospy.ROSInterruptException:
        pass