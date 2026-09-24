#!/usr/bin/env python3

import rospy
import actionlib
from for_torque.msg import FallCounterAction, FallCounterFeedback, FallCounterResult
import time
import calibration
import numpy as np
from sensor_utils import read_initial_sensor_offset, read_sensor_data
from std_msgs.msg import Float64, Float32MultiArray
from control_system.msg import CubeMarsEncoder
from calibration import ft_data
from rotation import ft_rotate
from sigmoid_calc import sigmoid
from wti_calculator import wti_calc
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_inverse, quaternion_multiply
import csv
import os
from datetime import datetime

class FallCounter():
     # create messages that are used to publish feedback/result
     _feedback = FallCounterFeedback()
     _result = FallCounterResult()

     def encoder_update(self,msg: CubeMarsEncoder):
     # update motor velocity in rad/s
        data = msg.data
        self.left_vel = data[1]
        self.right_vel = data[4]
        self.actual_twist = self.wheel_diameter*(self.right_vel - self.left_vel)/(2.0*self.wheelbase)
        self.actual_linear = self.wheel_diameter*(self.right_vel + self.left_vel)/ 4.0

     def __init__(self):

         self._as = actionlib.SimpleActionServer('fall_counter_server', FallCounterAction, execute_cb=self.execute_cb, auto_start = False)
         self.pub_linear_velocity = rospy.Publisher('linear_velocity', Float64, queue_size=1)
         self.pub_angular_velocity = rospy.Publisher('angular_velocity', Float64, queue_size=1)
         self.encoder_sub = rospy.Subscriber('/encoder_data', CubeMarsEncoder,self.encoder_update)
         self.desired_rate = 25
         self.wheelbase = 0.6858
         self.wheel_diameter = 0.236
         self.default_s = 0.65/5

         self._as.start()


     def execute_cb(self, goal):
         # helper variables
         r = rospy.Rate(self.desired_rate)
         self.initial_time = time.time()
         self.linear_command = goal.linear_mag*self.default_s
         if self.linear_command >= 10:
             self.linear_command = 10
         elif self.linear_command <= -10:
             self.linear_command = -10
         self.theta_dir = goal.theta_traj
         if 1.57<= self.theta_dir < 2.36:
            self.theta_dir = -(self.theta_dir-1.57)
            self.linear_command = -self.linear_command
         elif -2.36 < self.theta_dir <= -1.57:
            self.theta_dir = -(self.theta_dir+1.57)
            self.linear_command = -self.linear_command
         elif self.theta_dir >= 2.36 or self.theta_dir <= -2.36:
            self.theta_dir = 0
            self.linear_command = -self.linear_command
         timestamp = datetime.now().strftime("%Y_%m_%d_%H_%M")
         base_dir = os.path.dirname(os.path.abspath(__file__))
         input_dir = os.path.join(base_dir, 'csv', 'input')
         input_file = f'{timestamp}_input.csv'
         self.inputfilepath = os.path.join(input_dir, input_file)
         fall_dir = os.path.join(base_dir, 'csv', 'fall')
         fall_file = f'{timestamp}_fall_data.csv'
         self.fallfilepath = os.path.join(fall_dir, fall_file)
         for directory in [input_dir, fall_dir]:
            if not os.path.exists(directory):
                os.makedirs(directory)
         self.odom_initial = rospy.wait_for_message('/odom', Odometry, timeout=None)
         self.initial_orientation_quat = np.array([
            self.odom_initial.pose.pose.orientation.x,
            self.odom_initial.pose.pose.orientation.y,
            self.odom_initial.pose.pose.orientation.z,
            self.odom_initial.pose.pose.orientation.w])

         self.time_array, self.sigmoid_array, self.sig_d = sigmoid(self.theta_dir,self.desired_rate)
         input_header = ["Time", "Sigmoid", "Delta_Sig"]
         with open(self.inputfilepath, 'w', newline='') as csvfile:
         # Create a csv.writer object
            csv_writer = csv.writer(csvfile)
         # Write the header row
            csv_writer.writerow(input_header)

         # Write the data rows
            for times,theta,omega in zip(self.time_array, self.sigmoid_array, self.sig_d):
                csv_writer.writerow([times,theta,omega])

         data_header = ['Time', 'WTI_a', 'WTI_ml', 'WTI_mr','WTI_p','Fx','Fy','Fz','Tx','Ty','Tz','Vl','Vr','Yaw',
                        'dPsi','ds']
         with open(self.fallfilepath, 'w', newline='') as datafile:
            # Create a csv.writer object
                csv_writer = csv.writer(datafile)
                csv_writer.writerow(data_header)

         # start executing the action
         for command in self.sig_d:
            self.current_time = time.time()
            self.time_passed = self.current_time - self.initial_time
            if self.time_passed > 3.0:
                self.linear_command = 0
            self.odom_actual = rospy.wait_for_message('/odom', Odometry, timeout=None)
            self.current_orientation_quat = np.array([
                self.odom_actual.pose.pose.orientation.x,
                self.odom_actual.pose.pose.orientation.y,
                self.odom_actual.pose.pose.orientation.z,
                self.odom_actual.pose.pose.orientation.w])
            self.pub_linear_velocity.publish(self.linear_command)
            #self.pub_linear_velocity.publish(0)
            #self.pub_angular_velocity.publish(0)
            self.pub_angular_velocity.publish(command)
            raw_ft = read_initial_sensor_offset()
            ft = ft_data(raw_ft)
            ft_rotated = ft_rotate(ft)
            wti_data = wti_calc(ft_rotated)
            self._feedback.wti = wti_data
            self.odom_change = quaternion_multiply(self.current_orientation_quat,
                              quaternion_inverse(self.initial_orientation_quat))
            self.roll, self.pitch, self.yaw = euler_from_quaternion (self.odom_change)
            self._feedback.odom = self.yaw
            self._feedback.input_twist = command
            self._feedback.actual_twist = self.odom_actual.twist.twist.angular.z
            # publish the feedback
            self._as.publish_feedback(self._feedback)
            ft_feedback = np.hstack(ft_rotated)
            combined_data = [self.time_passed, *wti_data, *ft_feedback, self.left_vel,
                            self.right_vel, self.yaw, self.odom_actual.twist.twist.angular.z,
                            self.odom_actual.twist.twist.linear.x]
            with open(self.fallfilepath, 'a', newline='') as datafile:
            # Create a csv.writer object
                csv_writer = csv.writer(datafile)
                csv_writer.writerow(combined_data)
            r.sleep()

         
         self._result.yaw_change = self.yaw
         rospy.loginfo('%s: Succeeded' % self._as)
         self._as.set_succeeded(self._result)
         #os.system("rosnode kill velocity_publisher")

if __name__ == '__main__':
     rospy.init_node('fall_counter_server')
     server = FallCounter()
     rospy.spin()