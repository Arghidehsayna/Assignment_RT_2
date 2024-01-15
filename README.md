**Assignment_RT_2**

The second task in the Research track entails creating a ROS package for a simulated robot in Gazebo. This package consists of three nodes:

 Node A :
Purpose:
This script controls the robot's movement by allowing users to set and cancel goals. It initializes an action client to interact with a goal-reaching action server and continuously publishes the robot's kinematics (position and velocity) to a specified topic.

Dependencies:
ROS
Required ROS message types: geometry_msgs, nav_msgs, actionlib, and custom messages from the assignment_2_2023 package.
Usage:
Ensure ROS is installed.
Execute the script (node_a_client.py).
Follow on-screen instructions to set or cancel goals.
Enter 'exit' to terminate the program.

 Node B :
Purpose:
This script provides a ROS service (last_target) to retrieve the last user-entered target coordinates. It responds to service requests by fetching the target coordinates from ROS parameters.

Dependencies:
ROS
Replace your_package_name with the actual package name.
Custom service type (LastTarget) from your_package_name.srv package.
Usage:
Ensure ROS is installed.
Replace your_package_name with the actual package name.
Execute the script (node_b_service.py).
The service will be available at /last_target.

 Node C :
Purpose:
This script calculates the average velocity along the x-axis and the distance to the target. It subscribes to the kinematics topic for velocity updates and uses the last_target service to obtain the target coordinates.

Dependencies:
ROS
Required ROS message types: geometry_msgs, nav_msgs, messages, and services from the assignment_2_2023 package.
Usage:
Ensure ROS is installed.
Execute the script (node_c_service.py).
The service will be available at /average.
Set the average_window parameter in the ROS parameter server.
Customize the scripts based on your specific robot and environment configurations. If you encounter issues, refer to the ROS documentation or contact the developer for assistance.






