#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64, Float64MultiArray
from sensor_utils import read_initial_sensor_offset, read_sensor_data
from calibration import ft_data
from velocity_calculator import VelocityCalculator
from fall_checker import ft_deviation_check, ft_rate_check
import time
import os
from plotter import plot_data  # Importing the plotter module
import numpy as np
from rospy.numpy_msg import numpy_msg
from rotation import ft_rotate
from for_torque.msg import ft_list, FallCounterAction, FallCounterGoal
import actionlib
import math
from datetime import datetime
import csv

class VelocityPublisher:
    
    def __init__(self):
        rospy.init_node('velocity_publisher', anonymous=True)
        self.pub_linear_velocity = rospy.Publisher('linear_velocity', Float64, queue_size=2)
        self.pub_angular_velocity = rospy.Publisher('angular_velocity', Float64, queue_size=2)
        self.pub_force_torque = rospy.Publisher('force_torque',numpy_msg(ft_list),queue_size=2)
        self.pub_time = rospy.Publisher('ft_time', Float64, queue_size=2)
        self.rate = rospy.Rate(100)#100
        self.first_rate_check = True
        self.first_dev_check = True
        self.start_time = time.time()

        self.velocity_calculator = VelocityCalculator(max_linear_velocity=10, max_angular_velocity=10)
        self.start_checks()
        # Data storage for plotting
        self.times, self.forces, self.torques, self.linear_velocities, self.angular_velocities = [], [], [], [], []
        
        base_dir = os.path.dirname(os.path.abspath(__file__))
        ft_dir = os.path.join(base_dir, 'csv', 'ft')
        timestamp = datetime.now().strftime("%Y_%m_%d_%H_%M")
        ft_file = f'{timestamp}_ft_data.csv'
        field_names = ['Time', 'Fx', 'Fy','Fz','Tx','Ty','Tz']
        self.ftfilepath = os.path.join(ft_dir, ft_file)
        velocity_dir = os.path.join(base_dir, 'plots', 'velocity')
        ft_velocity_dir = os.path.join(base_dir,"plots","ft_vel")
        for directory in [ft_dir, velocity_dir]:
            if not os.path.exists(directory):
                os.makedirs(directory)
        with open(self.ftfilepath, 'w', newline='') as datafile:
                csv_writer = csv.writer(datafile)
                csv_writer.writerow(field_names)
        self.velocity_publisher()
        
    def start_checks(self):
        self.initial_sensor_offset = read_initial_sensor_offset()  # Read once
        self.initial_readings = self.collect_initial_readings(self.initial_sensor_offset)
        self.velocity_calculator.calculate_initial_threshold(self.initial_readings)
        #rospy.loginfo(f"Initial threshold: {velocity_calculator.dynamic_threshold}")
    
    def dev_check(self, event):
        if self.first_dev_check == True:
            self.last_dev = self.force_torque_rotated
            self.first_dev_check = False

        else:
            self.current_dev = self.force_torque_rotated
            self.ft_dev = np.subtract(self.current_dev, self.last_dev)
            self.fall_state = ft_deviation_check(self.ft_dev)

    def rate_check(self, event):
        if self.first_rate_check == True:
            self.last_rate = self.force_torque_rotated
            self.first_rate_check = False
        else:
            self.current_rate = self.force_torque_rotated
            self.ft_change = np.subtract(self.current_rate, self.last_rate)
            self.ft_rate = np.divide(self.ft_change, 0.05)
            self.fall_state = ft_rate_check(self.ft_rate)

    def collect_initial_readings(self, initial_sensor_offset, duration=5):
        initial_readings = []
        starting_time = time.time()
        rospy.loginfo('Hold onto handle in operating position')
        while time.time() - starting_time < duration:
            sensor_data = read_sensor_data(initial_sensor_offset)
            initial_readings.append(sensor_data)
            time.sleep(0.1)
        return initial_readings
            
    def velocity_publisher(self):
        self.fall_state = "No fall"
        while not rospy.is_shutdown():
            self.current_time = time.time()
            self.sensor_data = read_sensor_data(self.initial_sensor_offset)
            sensor_ft = read_initial_sensor_offset()
            raw_ft = ft_data(sensor_ft)
            rotated_raw_ft = ft_rotate(raw_ft)
            time_passed = self.current_time-self.start_time
            combined_data = [time_passed,*rotated_raw_ft]
            with open(self.ftfilepath, 'a', newline='') as datafile:
            # Create a csv.writer object
                csv_writer = csv.writer(datafile,delimiter=',')
                csv_writer.writerow(combined_data)
            self.force_torque = ft_data(self.sensor_data)    
            self.force_torque_rotated = np.array(ft_rotate(self.force_torque),dtype=np.float64)
            rospy.Timer(rospy.Duration(1.0/10.0), self.dev_check)
            rospy.Timer(rospy.Duration(1.0/20.0), self.rate_check)
            #rospy.loginfo(f"Sensor data: {self.sensor_data}")
            #rospy.loginfo(f"Force/Torque data: {self.force_torque_rotated}")
            #rospy.loginfo_throttle(1,f"Sensor data: {self.sensor_data}")
            rospy.loginfo_throttle(1,f"Force/Torque data: {self.force_torque_rotated}")
            #rospy.loginfo(f"Time: {self.current_time-self.start_time}")
            #rospy.loginfo(f"Dynamic Threshold:  {self.velocity_calculator.dynamic_threshold}")

            if self.fall_state != "No fall":
                rospy.loginfo("Fall Detected")
                rospy.loginfo(f"Deviation: {self.ft_dev}")
                rospy.loginfo(f"Rate of change: {self.ft_rate}")
                rospy.loginfo(f"Fall Force/Torque: {self.force_torque_rotated}")
                force_x = self.force_torque_rotated[0]
                #force_y = -self.force_torque_rotated[1]
                force_y = -self.force_torque_rotated[3]/0.2794
                #if self.force_torque_rotated[0] > 0:
                #    force_x = self.force_torque_rotated[0]
                #else:
                #    force_x = self.force_torque_rotated[0]
                
                #if self.force_torque_rotated[1] > 0:
                #    force_y = self.force_torque_rotated[1]*1.5
                #else:
                #    force_y = self.force_torque_rotated[1]
                    
                fall_direction = (math.atan2(force_y,force_x))
                fall_mag = -(math.sqrt(self.force_torque_rotated[0]**2 + self.force_torque_rotated[1]**2))
                client = actionlib.SimpleActionClient('fall_counter_server', FallCounterAction)
                client.wait_for_server()
                rospy.loginfo("Fall Counter Action Server Connected")
                goal = FallCounterGoal()
                goal.linear_mag = fall_mag/100.00
                goal.theta_traj = fall_direction
                rospy.loginfo(f"Fall direction: {math.degrees(goal.theta_traj)}")
                rospy.loginfo(f"Fall magnitude: {goal.linear_mag}")
                client.send_goal(goal)
                client.wait_for_result()
                result=client.get_result()
                rospy.loginfo(f"Counteraction result: {result}")
                self.linear_velocity = 0
                self.angular_velocity = 0                
                self.pub_linear_velocity.publish(Float64(-self.linear_velocity))
                self.pub_angular_velocity.publish(Float64(self.angular_velocity))
                self.fall_state = "No fall"
                rospy.signal_shutdown("Fall countermeasure successful")

            else:
                self.velocity_calculator.update_recent_readings(self.force_torque)
                self.velocity_calculator.check_for_resting_state(self.current_time)

                self.linear_velocity = self.velocity_calculator.compute_linear_velocity(self.force_torque[:])
                self.angular_velocity = self.velocity_calculator.compute_angular_velocity(self.force_torque)

                #rospy.loginfo(f"Linear Velocity: {linear_velocity}, Angular Velocity: {angular_velocity}")
                if self.velocity_calculator.resting_state_detected:
                    self.linear_velocity = 0
                    self.angular_velocity = 0

                self.pub_linear_velocity.publish(Float64(-self.linear_velocity))
                self.pub_angular_velocity.publish(Float64(self.angular_velocity))
                self.pub_force_torque.publish(self.force_torque_rotated)
                self.pub_time.publish(Float64(self.current_time-self.start_time))

                # Storing data
                self.times.append(self.current_time - self.start_time)
                #forces.append(force_torque_rotated[:3])  # Assuming the first three values are forces
                #torques.append(force_torque_rotated[3:])  # Assuming the last three values are torques
                self.linear_velocities.append(self.linear_velocity)
                self.angular_velocities.append(self.angular_velocity)

                # Debugging data changes
                #rospy.loginfo(f"Time: {times[-1]}, Forces: {forces[-1]}, Torques: {torques[-1]}, Linear Velocity: {linear_velocities[-1]}, Angular Velocity: {angular_velocities[-1]}")

                # Save plots every 10 seconds
                #if int(current_time - start_time) % 10 == 0:
                #  plot_data(times, forces, torques)

                self.rate.sleep()
            
if __name__ == '__main__':
    try:
        VelocityPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
