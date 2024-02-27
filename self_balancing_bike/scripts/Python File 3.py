import rospy
from gazebo_msgs.srv import GetModelProperties, GetModelState
from geometry_msgs.msg import Pose

rospy.init_node('com_extractor')

# Wait for the required services to become available
rospy.wait_for_service('/gazebo/get_model_properties')
rospy.wait_for_service('/gazebo/get_model_state')

# Create service proxies
get_model_properties = rospy.ServiceProxy('/gazebo/get_model_properties', GetModelProperties)
get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

# Specify the name of your bike model
bike_name = 'bike'

# Get the properties of your bike model
model_properties = get_model_properties(bike_name)

# Retrieve the link names of the bike model
link_names = model_properties.body_names

# Calculate the center of mass (COM) of the entire bike model with respect to the Gazebo world coordinates
total_mass = 0.0
com_x = 0.0
com_y = 0.0
com_z = 0.0

for link_name in link_names:
    link_properties = get_link_properties(link_name, 'world')
    link_mass = link_properties.mass
	link_com = link_properties.com
    
    total_mass += link_mass
    com_x += link_mass * link_com.x
    com_y += link_mass * link_com.y
    com_z += link_mass * link_com.z

com_x /= total_mass
com_y /= total_mass
com_z /= total_mass

print("Bike Model:", bike_name)
print("Center of Mass (COM) with respect to World:")
print("  x:", com_x)
print("  y:", com_y)
print("  z:", com_z)

