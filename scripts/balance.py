#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
from tf.transformations import euler_from_quaternion

class BikeControllerNode:
    def __init__(self):
        rospy.init_node('bike_controller_node', anonymous=True)
        rospy.Subscriber('/imu', Imu, self.imu_callback)
        self.rw_pub = rospy.Publisher('/bike/rw_jVel_controller/command', Float64, queue_size=10)
        self.target_roll = 0.0  # Target pitch angle (modify as per your requirement)
        self.kp = 0.5  # Proportional gain (modify as per your requirement)

    def imu_callback(self, msg):
        # Extract the pitch angle from the IMU quaternion data
        quaternion = (
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        )
        roll, _, _ = euler_from_quaternion(quaternion)

        # Compute the control effort based on the error between the current pitch and target pitch
        error = self.target_roll - roll
        control_effort = self.kp * error

        # Publish the control effort to the reaction wheel joint
        self.publish_effort(control_effort)

    def publish_effort(self, effort):
        command_msg = Float64()
        command_msg.data = effort
        self.rw_pub.publish(command_msg)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        bike_controller_node = BikeControllerNode()
        bike_controller_node.run()
    except rospy.ROSInterruptException:
        pass
