import rospy
from gazebo_msgs.srv import GetModelProperties, GetLinkState
from geometry_msgs.msg import Pose

rospy.init_node('com_extractor')

# Wait for the required services to become available
rospy.wait_for_service('/gazebo/get_model_properties')
rospy.wait_for_service('/gazebo/get_link_state')

# Create service proxies
get_model_properties = rospy.ServiceProxy('/gazebo/get_model_properties', GetModelProperties)
get_link_state = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)

# Specify the name of your robot model
robot_name = 'bike'

# Get the properties of your robot model
model_properties = get_model_properties(robot_name)

# Retrieve the link names of the robot model
link_names = model_properties.body_names

# Calculate the COMs of all links with respect to the Gazebo world coordinates
for link_name in link_names:
    link_state = get_link_state(link_name, 'world')

    link_com = link_state.link_state.pose.position

    print("Link:", link_name)
    print("Center of Mass (COM) with respect to World:")
    print("  x:", link_com.x)
    print("  y:", link_com.y)
    print("  z:", link_com.z)
    print()

