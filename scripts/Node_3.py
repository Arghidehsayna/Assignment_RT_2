#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Point, Pose, Twist
from nav_msgs.msg import Odometry
from assignment_2_2023.msg import Vel
from assignment_2_2023.srv import LastTarget, LastTargetRequest, Average, AverageResponse
import actionlib
import actionlib.msg
import math

class AverageCalculator:
     def __init__(self):
        # Initialize variables to store state
        self.velocity_list = []             # List to store velocity values
        self.distance = 0                   # Distance between current position and target
        self.average_velocity_x = 0         # Average velocity along the x-axis

        # Wait for the LastTarget service to become available
        rospy.wait_for_service("last_target")
        self.last_target_client = rospy.ServiceProxy('last_target', LastTarget)
        
        # Initialize the ROS node
        rospy.init_node("node_c_service")
        rospy.loginfo("Node started and ready to calculate the average")

        # Setup the 'average' service
        rospy.Service('average', Average, self.calculate_average)

        # Subscribe to the 'kinematics' topic to receive velocity updates
        rospy.Subscriber("/kinematics", Vel, self.update_average)

     def calculate_average(self, req):
        # Callback function for the 'average' service
        res = AverageResponse()
        res.dist = self.distance
        res.velocity_mean = self.average_velocity_x
        return res
        
     def update_average(self, kinematics_msg):
        # Callback function for 'kinematics' topic updates
        # Get the latest target coordinates from the LastTarget service
        last_target_response = self.last_target_client(LastTargetRequest())

        # Extract coordinates and velocity values
        target_x, target_y = last_target_response.target_x, last_target_response.target_y
        current_x, current_y = kinematics_msg.x, kinematics_msg.y
        current_velocity_x = kinematics_msg.vel_x

        # Calculate the distance between current position and target
        self.distance = math.sqrt((target_x - current_x)**2 + (target_y - current_y)**2)

        # Update the velocity list with the latest velocity value
        self.velocity_list.append(current_velocity_x)

        # Get the window size for calculating the average
        window_size = rospy.get_param('average_window')
        
        
        
        # Calculate the average velocity along the x-axis
        if len(self.velocity_list) < window_size:
            self.average_velocity_x = sum(self.velocity_list) / len(self.velocity_list)
        else:
            self.average_velocity_x = sum(self.velocity_list[-window_size:]) / window_size
            
            
            
def main():
    # Create an instance of the AverageCalculator class
    average_calculator = AverageCalculator()

    # Keep the program running
    rospy.spin()

if __name__ == "__main__":
    main()
    

