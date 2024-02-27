import rospy
from gazebo_msgs.srv import GetModelProperties, GetLinkProperties
from geometry_msgs.msg import Pose

rospy.init_node('com_extractor')

# Wait for the required service to become available
rospy.wait_for_service('/gazebo/get_model_properties')

# Create service proxy
get_model_properties = rospy.ServiceProxy('/gazebo/get_model_properties', GetModelProperties)
get_link_properties = rospy.ServiceProxy('/gazebo/get_link_properties', GetLinkProperties)

# Specify the name of your robot model
robot_name = 'bike'

# Get the properties of your robot model
model_properties = get_model_properties(robot_name)

# Retrieve the link names of the robot model
link_names = model_properties.body_names

# Calculate the mass of the whole body
total_mass = 0.0

# Calculate the mass and COM of each link
for link_name in link_names:
    link_properties = get_link_properties(link_name)
    link_mass = link_properties.mass
    link_com = link_properties.com

    total_mass += link_mass

    print("Link:", link_name)
    print("Mass:", link_mass)
    print("Center of Mass (COM):")
    print("  x:", link_com.position.x)
    print("  y:", link_com.position.y)
    print("  z:", link_com.position.z)
    print()

print("Total Mass:", total_mass)

