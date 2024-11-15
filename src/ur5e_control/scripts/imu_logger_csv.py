#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Vector3Stamped, QuaternionStamped, PoseStamped, Pose2D
from sensor_msgs.msg import Imu, MagneticField, TimeReference, JointState
from nav_msgs.msg import Odometry
import csv
import os
from datetime import datetime

class IMULogger:
    def __init__(self):
        rospy.init_node('imu_logger', anonymous=True)
        
        # Create main imu_logs directory
        self.log_dir = 'imu_logs'
        if not os.path.exists(self.log_dir):
            os.makedirs(self.log_dir)
        
        # Create individual folders for each data type
        self.data_folders = {
            '/filter/euler': 'euler_data',
            '/filter/free_acceleration': 'free_acceleration_data',
            '/filter/quaternion': 'quaternion_data',
            '/imu/acceleration': 'acceleration_data',
            '/imu/angular_velocity': 'angular_velocity_data',
            '/imu/data': 'imu_data',
            '/imu/mag': 'magnetic_data',
            '/imu/time_ref': 'time_reference_data',
            '/joint_states': 'joint_states_data',
            '/Robot_1/pose': 'mocap_pose_data',
            '/Robot_1/ground_pose': 'mocap_ground_pose_data',
            '/Robot_1/Odom': 'mocap_odom_data'
        }
        
        # Create all subdirectories
        for folder in self.data_folders.values():
            folder_path = os.path.join(self.log_dir, folder)
            if not os.path.exists(folder_path):
                os.makedirs(folder_path)
        
        # Define subscribers with their message types
        self.subscribers = {
            '/filter/euler': Vector3Stamped,
            '/filter/free_acceleration': Vector3Stamped,
            '/filter/quaternion': QuaternionStamped,
            '/imu/acceleration': Vector3Stamped,
            '/imu/angular_velocity': Vector3Stamped,
            '/imu/data': Imu,
            '/imu/mag': MagneticField,
            '/imu/time_ref': TimeReference,
            '/joint_states': JointState,
            '/Robot_1/pose': PoseStamped,
            '/Robot_1/ground_pose': Pose2D,
            '/Robot_1/Odom': Odometry
        }
        
        # Initialize CSV writers for each topic
        self.csv_files = {}
        self.csv_writers = {}
        for topic, msg_type in self.subscribers.items():
            filename = self._create_filename(topic)
            file = open(filename, 'w', newline='')
            writer = csv.writer(file)
            headers = self._get_headers(msg_type)
            writer.writerow(['Timestamp'] + headers)
            self.csv_files[topic] = file
            self.csv_writers[topic] = writer
        
        # Store subscribers in a list for cleanup
        self.subscribers_list = []
        
        # Subscribe to each topic with its specific callback
        for topic, msg_type in self.subscribers.items():
            sub = rospy.Subscriber(topic, msg_type, self._get_callback(topic, msg_type))
            self.subscribers_list.append(sub)
        
        # Log that recording has started
        rospy.loginfo("IMU Logger: Recording started. Data is being saved to: %s", self.log_dir)
        rospy.loginfo("IMU Logger: Recording the following topics:")
        for topic in self.subscribers.keys():
            rospy.loginfo("  - %s", topic)
        
        # Add shutdown hook
        rospy.on_shutdown(self.shutdown_hook)
    
    def _create_filename(self, topic):
        # Get the specific folder for this topic
        folder_name = self.data_folders[topic]
        
        # Create the full path without duplicate imu_logs
        folder_path = os.path.join(self.log_dir, folder_name)
        
        # Sanitize topic name for filename
        sanitized_topic = topic.strip('/').replace('/', '_')
        timestamp = datetime.now().strftime('%d-%m-%Y-%H-%M')
        filename = f'{sanitized_topic}_data_{timestamp}.csv'
        
        # Return full path
        return os.path.join(folder_path, filename)
    
    def _get_headers(self, msg_type):
        if msg_type == Vector3Stamped:
            return ['x', 'y', 'z']
        elif msg_type == QuaternionStamped:
            return ['quaternion_x', 'quaternion_y', 'quaternion_z', 'quaternion_w']
        elif msg_type == Imu:
            return [
                'linear_acceleration_x', 'linear_acceleration_y', 'linear_acceleration_z',
                'angular_velocity_x', 'angular_velocity_y', 'angular_velocity_z',
                'orientation_x', 'orientation_y', 'orientation_z', 'orientation_w'
            ]
        elif msg_type == MagneticField:
            return ['magnetic_field_x', 'magnetic_field_y', 'magnetic_field_z']
        elif msg_type == TimeReference:
            return ['time_ref', 'source']
        elif msg_type == JointState:
            return ['shoulder_pan', 'shoulder_lift', 'elbow', 'wrist_1', 'wrist_2', 'wrist_3']
        elif msg_type == PoseStamped:
            return ['position_x', 'position_y', 'position_z',
                   'orientation_x', 'orientation_y', 'orientation_z', 'orientation_w']
        elif msg_type == Pose2D:
            return ['x', 'y', 'theta']
        elif msg_type == Odometry:
            return ['position_x', 'position_y', 'position_z',
                   'orientation_x', 'orientation_y', 'orientation_z', 'orientation_w',
                   'linear_vel_x', 'linear_vel_y', 'linear_vel_z',
                   'angular_vel_x', 'angular_vel_y', 'angular_vel_z']
        else:
            return []
    
    def _get_callback(self, topic, msg_type):
        def callback(data):
            timestamp = rospy.get_time()
            writer = self.csv_writers[topic]
            if msg_type == Vector3Stamped:
                row = [data.vector.x, data.vector.y, data.vector.z]
            elif msg_type == QuaternionStamped:
                row = [
                    data.quaternion.x, data.quaternion.y,
                    data.quaternion.z, data.quaternion.w
                ]
            elif msg_type == Imu:
                row = [
                    data.linear_acceleration.x, data.linear_acceleration.y, data.linear_acceleration.z,
                    data.angular_velocity.x, data.angular_velocity.y, data.angular_velocity.z,
                    data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w
                ]
            elif msg_type == MagneticField:
                row = [
                    data.magnetic_field.x,
                    data.magnetic_field.y,
                    data.magnetic_field.z
                ]
            elif msg_type == TimeReference:
                row = [
                    data.time_ref.to_sec(),
                    data.source
                ]
            elif msg_type == JointState:
                row = [data.position[0], data.position[1], data.position[2], data.position[3], data.position[4], data.position[5]]
            elif msg_type == PoseStamped:
                row = [
                    data.pose.position.x, data.pose.position.y, data.pose.position.z,
                    data.pose.orientation.x, data.pose.orientation.y,
                    data.pose.orientation.z, data.pose.orientation.w
                ]
            elif msg_type == Pose2D:
                row = [data.x, data.y, data.theta]
            elif msg_type == Odometry:
                row = [
                    data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z,
                    data.pose.pose.orientation.x, data.pose.pose.orientation.y,
                    data.pose.pose.orientation.z, data.pose.pose.orientation.w,
                    data.twist.twist.linear.x, data.twist.twist.linear.y, data.twist.twist.linear.z,
                    data.twist.twist.angular.x, data.twist.twist.angular.y, data.twist.twist.angular.z
                ]
            else:
                row = []
            
            # Prepend timestamp and write to CSV
            writer.writerow([timestamp] + row)
        
        return callback
    
    def shutdown_hook(self):
        """Clean shutdown handling"""
        rospy.loginfo("IMU Logger: Recording interrupted. Saving and closing files...")
        
        # Unsubscribe from all topics first
        for sub in self.subscribers_list:
            sub.unregister()
        
        # Close all CSV files
        for file in self.csv_files.values():
            file.close()
            
        rospy.loginfo("IMU Logger: All files have been saved and closed.")

if __name__ == '__main__':
    try:
        logger = IMULogger()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass