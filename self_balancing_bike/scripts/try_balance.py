#!/usr/bin/env python

from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan, Imu
from geometry_msgs.msg import Quaternion,Twist
import rospy
import tf

class RWbike():
    def __init__(self):

        rospy.init_node('balancing')

        self.orient_quat = [0.0, 0.0, 0.0, 0.0]
        self.orient_euler = [0.0, 0.0, 0.0]
        self.curr_angle = 0.0
        self.prev_angle = 0.0
        self.w = 0.0
        self.i = 0.0
        
        self.data_cmd = Float64()
        self.data_cmd.data = 0.0
        self.data_pub = rospy.Publisher('/bike/rw_jVel_controller/command', Float64, queue_size=10)

        rospy.Subscriber('/imu', Imu, self.imu_callback)

    def imu_callback(self, msg):

        self.orient_quat[0] = msg.orientation.x
        self.orient_quat[1] = msg.orientation.y
        self.orient_quat[2] = msg.orientation.z
        self.orient_quat[3] = msg.orientation.w
        self.orient_euler = tf.transformations.euler_from_quaternion(self.orient_quat)


    def pid(self):
            
        self.curr_angle = self.orient_euler[0]
        print(self.curr_angle,"\n")

        kp=-30
        kd=0.155
        dt=0.001
        ki=0

        self.i = self.i + self.curr_angle*dt

        self.w = self.w - (kp*self.curr_angle)-(kd*(self.curr_angle-self.prev_angle)/dt)-(ki*self.i)     

        self.data_cmd.data= self.w
        rospy.loginfo("Reaction wheel w: "+str(self.w))
        self.data_pub.publish(self.data_cmd.data)

        self.prev_angle=self.curr_angle


if __name__ == '__main__':

    bike = RWbike()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():

        try:
            bike.pid()
            rate.sleep()

        except rospy.ROSInterruptException:
            pass

