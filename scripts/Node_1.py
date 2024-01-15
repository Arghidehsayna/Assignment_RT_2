#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Odometry
import actionlib
from assignment_2_2023.msg import Vel, PlanningAction, PlanningGoal
from actionlib_msgs.msg import GoalStatus

# Initialize the publisher to publish position and velocity
kinematics_pub = rospy.Publisher("/kinematics", Vel, queue_size=1)

def publish_kinematics_callback(msg):
# Extract position and velocity information from Odometry message
    linear_vel_now = msg.twist.twist.linear
    angular_vel_now = msg.twist.twist.angular

# Create a custom message containing the robot's position and velocity
    kinematics_msg = Vel()
    kinematics_msg.x = msg.pose.pose.position.x
    kinematics_msg.y = msg.pose.pose.position.y
    kinematics_msg.vel_x = linear_vel_now.x
    kinematics_msg.vel_z = angular_vel_now.z
    
# Publish the custom message
    kinematics_pub.publish(kinematics_msg)

def initialize_action_client():
# Initialize the action client for reaching the goal
    planning_client = actionlib.SimpleActionClient('/reaching_goal', PlanningAction)
    planning_client.wait_for_server()
    rospy.loginfo("Action client successfully initialized.")
    return planning_client

def main():
    rospy.init_node('node_a_client')
    rospy.loginfo("Node A started successfully.")

    action_client = initialize_action_client()
    have_goal = False
    goal_new = PlanningGoal()  # Initialize goal outside the loop

    while not rospy.is_shutdown():
        rospy.Subscriber("/odom", Odometry, publish_kinematics_callback)

        target_x = rospy.get_param('/des_pos_x')
        target_y = rospy.get_param('/des_pos_y')
        rospy.loginfo("Current goal is: target_x = %f, target_y = %f", target_x, target_y)

        print("Robot Control Instructions\n")
        print("  Type 'change': Set Target Position\n")
        print("  Type 'cancel': Cancel Target\n")
        print("  Type 'exit': Exit\n")
        user_choice = input("What's your choice now:\n").lower()
        
        if user_choice == 'change':
            if have_goal:
                rospy.logwarn("You should first stop the current target")
            else:
                new_target_x = float(input('Enter a value for the new x_coordinate:'))
                new_target_y = float(input('Enter a value for the new y_coordinate:'))

                rospy.set_param('/des_pos_x', new_target_x)
                rospy.set_param('/des_pos_y', new_target_y)

                goal_new.target_pose.pose.position.x = new_target_x
                goal_new.target_pose.pose.position.y = new_target_y

                action_client.send_goal(goal_new)
                have_goal = True
                
        elif user_choice == 'cancel':
             if have_goal:
                have_goal = False
                action_client.cancel_goal()
                rospy.loginfo("The goal is cancelled successfully")
            else:
                rospy.logwarn("There are no goals to be cancelled. First determine a goal!!!")
                
        elif user_choice == 'exit':
            break

        else:
            rospy.logwarn("Warning!!! The command should either be 'change' or 'cancel'.")

        rospy.loginfo("The chosen goal is: target_x = %f, target_y = %f",
                      goal_new.target_pose.pose.position.x, goal_new.target_pose.pose.position.y)

if __name__ == '__main__':
    main()       




