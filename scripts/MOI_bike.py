#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu
import csv
import os
import tf

class IMUDataLogger:
    def __init__(self):
        rospy.init_node('imu_data_logger_node', anonymous=True)
        self.imu_subscriber = rospy.Subscriber('/imu', Imu, self.imu_callback)
        self.output_file = "/home/mohit/workspace/catkin_ws/src/self_balancing_bike/scripts/imu_data.csv"
        rospy.loginfo("Output file path: {}".format(self.output_file))

        self.initialize_csv_file()
        self.stop_recording = False
        self.last_recorded_time = rospy.Time.now()

        
    def initialize_csv_file(self):
        with open(self.output_file, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['Timestamp', 'Tilt_angle'])

    def imu_callback(self, msg):
        
        if not self.stop_recording:

            current_time = rospy.Time.now()
            time_diff = current_time - self.last_recorded_time
            
            if time_diff.to_sec() >= 0.01:
        
                timestamp = msg.header.stamp
                orientation_x = msg.orientation.x
                orientation_y = msg.orientation.y
                orientation_z = msg.orientation.z
                orientation_w = msg.orientation.w
                
                (roll, pitch, yaw) = tf.transformations.euler_from_quaternion([orientation_x, orientation_y, orientation_z, orientation_w])


                rospy.loginfo("Timestamp: {}, tilt_angle(theta): {}".format(timestamp, roll))
                
                with open(self.output_file, mode='a', newline='') as file:
                    writer = csv.writer(file)
                    writer.writerow([timestamp, roll])

                # Check if tilt angle exceeds 0.20 radians
                if abs(roll) > 0.20:
                    rospy.loginfo("Tilt angle exceeds 0.20 radians. Stopping recording.")
                    self.stop_recording = True

                self.last_recorded_time = current_time


def main():
    try:
        imu_logger = IMUDataLogger()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
