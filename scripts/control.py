#!/usr/bin/env python3

from std_msgs.msg import Float64
from sensor_msgs.msg import Imu, JointState
from geometry_msgs.msg import Quaternion,Twist
import rospy
import tf
import numpy as np
import math



class Bike():  # Self Balancing Bike
    def __init__(self):

        rospy.init_node('bike_balance_node')

#Initialisations:
        

        #Frequency of operation.
        self.sample_rate = 100.0 #100 Hz

        #Pose from IMU
        self.bike_orientation_quaternion = [0.0, 0.0, 0.0, 0.0]
        self.bike_orientation_euler = [0.0, 0.0, 0.0] #roll,pitch,yaw values
        self.bike_angular_velocity=[0.0,0.0,0.0]


    #Geometric,Inertial properties of the bike,flywheel (for balancing)
        
        self.tilt_angle_bike = 0.0 #Roll angle of the bike, which needs to be driven to 0 for balancing
        self.moi_flywheel = 0.0075 #flywheel MOI
        self.k_bike = 10.6232          # (1.084*9.8)
        self.moi_bike = 1062   #Bike MOI about the horizontal axis (Contributions of all parts of the bike accounted for)
 
        self.kp = -7
        self.w_flywheel = 0 #Flywheel angular velocity
        self.kd = -0.0001 #Damping constant for torque control
        
        self.data_cmd = Float64()
        self.data_cmd.data = 0.0
        

    #Publishers
        self.data_flywheel = rospy.Publisher('/bike/rw_jVel_controller/command', Float64, queue_size=1)


    #Subscribers
        rospy.Subscriber('/imu', Imu, self.imu_callback)


#Callbacks

    def imu_callback(self, msg):
        self.bike_orientation_quaternion[0] = msg.orientation.x
        self.bike_orientation_quaternion[1] = msg.orientation.y
        self.bike_orientation_quaternion[2] = msg.orientation.z
        self.bike_orientation_quaternion[3] = msg.orientation.w

        #conversion of orientation quarternion to euler angles
        (self.bike_orientation_euler[1], self.bike_orientation_euler[0], self.bike_orientation_euler[2]) = tf.transformations.euler_from_quaternion([self.bike_orientation_quaternion[0], self.bike_orientation_quaternion[1], self.bike_orientation_quaternion[2], self.bike_orientation_quaternion[3]])
        self.tilt_angle_bike = self.bike_orientation_euler[1] #Bike roll angle (Balancing)
        # self.bike_angle = self.bike_orientation_euler[2] #Bike yaw angle (Navigation)
        self.bike_angular_velocity[0] = msg.angular_velocity.x #bike angular velocity (Balancing)



#Balancing
        
    def balancing(self):
        
        dt = 1.0/self.sample_rate #time interval
        w_bike=self.bike_angular_velocity[0] #Angular velocity of the bike about the x axis
        
        if abs(self.tilt_angle_bike) > 0.001: #Roll angle treated as the "error" term for control

             #PD control for torque
             #Damping constant selected for critical damping of the bike
             #vertical position achieved without oscillating
            torque  = -(self.kp*self.tilt_angle_bike + self.kd*w_bike) 
            print(torque)

            #Incrementing flywheel angular velocity and publishing it
            
            alpha = torque/self.moi_flywheel
            self.w_flywheel += alpha*dt
            
            self.data_cmd.data = self.w_flywheel 
            self.data_flywheel.publish(self.data_cmd.data)



if __name__ == '__main__':

    bike = Bike() #Creating bike object
    r = rospy.Rate(bike.sample_rate) #100Hz frequency
    while not rospy.is_shutdown():

        try:
            bike.balancing() #Running the balancing algorithm
            r.sleep()
        except rospy.exceptions.ROSTimeMovedBackwardsException:
            pass
