#!/usr/bin/env python3

# Import necessary libraries
import rospy
from your_package_name.srv import LastTarget, LastTargetResponse  

# Callback function to handle last_target service requests
def handle_last_target(request):
    # Create a response object
    response = LastTargetResponse()
    
    
    # Get the last target coordinates from ROS parameters
    response.target_x = rospy.get_param('/des_pos_x', default=0.0)
    response.target_y = rospy.get_param('/des_pos_y', default=0.0)
    
    return response
    
# Main function to initialize the ROS node and set up the service
def main():
    # Initialize the ROS node
    rospy.init_node("node_b_service")
    
    # Log information about the node's status
    rospy.loginfo("Target node ready to provide the last user-entered target")
    
    # Set up the ROS service to handle last_target requests
    rospy.Service('last_target', LastTarget, handle_last_target)
    
    # Keep the node running
    rospy.spin()

# Entry point of the script
if __name__ == "__main__":
    # Run the main function
    main()
    
