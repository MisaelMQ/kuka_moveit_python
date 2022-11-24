#!/usr/bin/env python3

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import math

class Mazinger():
    def __init__(self):
        # Initializing Namespace for Moving Robot with MoveIt
        moveit_commander.roscpp_initialize(sys.argv) 
        
        # To initialize the Robot Commander
        self.robot = moveit_commander.RobotCommander()

        # To configure Move Commander with the group Chosen  
        self.group_name = "end_effector"
        self.move_group = moveit_commander.MoveGroupCommander(self.group_name)

        # To configure the Publisher to display Movement in RVIZ
        self.display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )

        # Configuration 
        self.rate = rospy.Rate(10)  # 10hz
        self.ctrl_c = False

        rospy.on_shutdown(self.shutdownhook)

    def complete_movement(self):
        # Variable to store the number of iteration
        aux_var = 1

        # Condition to start Movement
        input("=> Press 'Enter' to Start ...")
        print('----------------------------------------------------')

        # Loop to execute if Ctrl+C is not pressed
        while not self.ctrl_c:
            # Function to Reset Pose with Joint Goal
            self.reset_pose()
            
            # Loop to verify that initial pose has been reached
            flag = True
            while flag:
                # Reading current state of the robot
                aux_pose = self.move_group.get_current_pose().pose
                
                # Checking if goal pose has been reached
                if(0.524 < aux_pose.position.x < 0.526) and (-0.001 < aux_pose.position.y < 0.001) and (0.889 < aux_pose.position.z < 0.891) and (-0.001 < aux_pose.orientation.x < 0.001) and (0.706 < aux_pose.orientation.y < 0.708) and (-0.001 < aux_pose.orientation.z < 0.001) and (0.706 < aux_pose.orientation.w < 0.708):
                    flag = False
            
            # Calculating Cartesian Path for Plan Chosen
            print('Initial Pose Reloaded ... ')
            cartesian_plan, fraction = self.plan_display()

            # Executing Cartesian Plan Calculated
            print('Cartesian Plan #{} Loaded ...'.format(str(aux_var)))
            self.execute_plan(cartesian_plan)

            # Printing if Plan has been finished
            print('Plan #{} Executed ...'.format(str(aux_var)))

            # Loop to verify that initial pose has been reached
            flag = True
            while flag:
                # Reading current state of the robot
                aux_pose = self.move_group.get_current_pose().pose
                
                # Checking if goal pose has been reached
                if(0.524 < aux_pose.position.x < 0.526) and (-0.001 < aux_pose.position.y < 0.001) and (0.889 < aux_pose.position.z < 0.891) and (-0.001 < aux_pose.orientation.x < 0.001) and (0.706 < aux_pose.orientation.y < 0.708) and (-0.001 < aux_pose.orientation.z < 0.001) and (0.706 < aux_pose.orientation.w < 0.708):
                    flag = False
            
            # Increasing number of iteration
            print('----------------------------------------------------')
            aux_var += 1
            
    
    def plan_display(self):
        # Variable to store Waypoints
        waypoints = []
        # Variable to increase or decrease movement
        scale = 1

        # Getting Initial Pose
        wpose = self.move_group.get_current_pose().pose

        # Zero Movement
        wpose.position.x -= scale * 0.1
        waypoints.append(copy.deepcopy(wpose))

        # First Movement
        wpose.position.z += scale * 0.1  
        wpose.position.y += scale * 0.1  
        waypoints.append(copy.deepcopy(wpose))

        # Second Movement
        wpose.position.z += scale * 0.1  
        wpose.position.y -= scale * 0.1 
        waypoints.append(copy.deepcopy(wpose))

        # Third Movement
        wpose.position.z -= scale * 0.1  
        wpose.position.y -= scale * 0.1 
        waypoints.append(copy.deepcopy(wpose))

        # Fourth Movement
        wpose.position.z -= scale * 0.1 
        wpose.position.y += scale * 0.1 
        waypoints.append(copy.deepcopy(wpose))

        # Final Movement
        wpose.position.x += scale * 0.1
        waypoints.append(copy.deepcopy(wpose))


        # Computing Cartesian Path with Waypoints
        (plan, fraction) = self.move_group.compute_cartesian_path(
            waypoints, 0.01, 0     # waypoints to follow  # eef_step
        )  # jump_threshold

        # Returning Plan Generated
        return plan, fraction
    
    def reset_pose(self):
        # Getting Current Joint Poses
        joint_goal = self.move_group.get_current_joint_values()
        
        # Updating Joint Poses
        joint_goal[0] = 0.0
        joint_goal[1] = -1.5708
        joint_goal[2] = 1.5708
        joint_goal[3] = 0.0
        joint_goal[4] = 0.0
        joint_goal[5] = 0.0

        # Executing Movement for Joint Goal
        self.move_group.go(joint_goal, wait=True)

        # Ensuring no Residual Movement
        self.move_group.stop()

        # For checking if goal position has been reached
        current_joints = self.move_group.get_current_joint_values()
        return self.all_close(joint_goal, current_joints, 0.01)

    def execute_plan(self, plan):
        # Executing Plan previously Generated
        self.move_group.execute(plan, wait=True)

    def all_close(self, goal, actual, tolerance):
        # Checking if goal is a list
        if type(goal) is list:
            for index in range(len(goal)):
                if abs(actual[index] - goal[index]) > tolerance:
                    return False

        # Checking if type of variable is correct
        elif type(goal) is geometry_msgs.msg.PoseStamped:
            return self.all_close(goal.pose, actual.pose, tolerance)

        # Checking tolerance between both poses
        elif type(goal) is geometry_msgs.msg.Pose:
            # Extracting information for current and goal pose 
            x0, y0, z0, qx0, qy0, qz0, qw0 = moveit_commander.pose_to_list(actual)
            x1, y1, z1, qx1, qy1, qz1, qw1 = moveit_commander.pose_to_list(goal)
            
            # Obtaining Euclidean Distances
            d = math.dist((x1, y1, z1), (x0, y0, z0))

            # Obtaining Angle between Poses
            cos_phi_half = math.fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
            
            return d <= tolerance and cos_phi_half >= math.cos(tolerance / 2.0)

        return True

    def shutdownhook(self):
        # Going to Start Values if Ctrl+C is pressed
        self.ctrl_c = True
        self.reset_pose() 

# Main Function
if __name__ == '__main__':
    # Initializing Node for moving Robot
    rospy.init_node('move_kuka_python', anonymous=True)
    # Creating an Instance of the Class Created
    kuka_object = Mazinger()
    try:
        # Executing Complete Movement of the Robot
        kuka_object.complete_movement()

    except rospy.ROSInterruptException:
        pass