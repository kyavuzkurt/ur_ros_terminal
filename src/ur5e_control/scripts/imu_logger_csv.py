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
        self.log_dir = 'imu_logs'
        if not os.path.exists(self.log_dir):
            os.makedirs(self.log_dir)
            rospy.loginfo(f"Created log directory: {self.log_dir}")
        else:
            rospy.loginfo(f"Log directory already exists: {self.log_dir}")
        
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
            '/mocap_node/Robot_1/pose': 'mocap_pose_data',
            '/mocap_node/Robot_1/ground_pose': 'mocap_pose2d_data',
            '/mocap_node/Robot_1/Odom': 'mocap_odom_data'
        }
        
        for folder in self.data_folders.values():
            folder_path = os.path.join(self.log_dir, folder)
            if not os.path.exists(folder_path):
                os.makedirs(folder_path)
                rospy.loginfo(f"Created subdirectory: {folder_path}")
            else:
                rospy.loginfo(f"Subdirectory already exists: {folder_path}")
        
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
            '/mocap_node/Robot_1/pose': PoseStamped,
            '/mocap_node/Robot_1/ground_pose': Pose2D,
            '/mocap_node/Robot_1/Odom': Odometry
        }
        
        self.csv_files = {}
        self.csv_writers = {}
        for topic, msg_type in self.subscribers.items():
            filename = self._create_filename(topic)
            try:
                file = open(filename, 'w', newline='')
                writer = csv.writer(file)
                headers = self._get_headers(msg_type)
                writer.writerow(['Timestamp'] + headers)
                self.csv_files[topic] = file
                self.csv_writers[topic] = writer
                rospy.loginfo(f"Initialized CSV writer for topic: {topic} -> {filename}")
            except Exception as e:
                rospy.logerr(f"Failed to open file {filename} for topic {topic}: {e}")
        
        self.subscribers_list = []
        
        for topic, msg_type in self.subscribers.items():
            try:
                sub = rospy.Subscriber(topic, msg_type, self._get_callback(topic, msg_type))
                self.subscribers_list.append(sub)
                rospy.loginfo(f"Subscribed to topic: {topic} with message type: {msg_type.__name__}")
            except Exception as e:
                rospy.logerr(f"Failed to subscribe to topic {topic}: {e}")
        
        # Log that recording has started
        #rospy.loginfo("IMU Logger: Recording started. Data is being saved to: %s", self.log_dir)
        #rospy.loginfo("IMU Logger: Recording the following topics:")
        for topic in self.subscribers.keys():
            rospy.loginfo("  - %s", topic)
        
        rospy.on_shutdown(self.shutdown_hook)
    
    def _create_filename(self, topic):
        folder_name = self.data_folders[topic]
        
        folder_path = os.path.join(self.log_dir, folder_name)
        
        sanitized_topic = topic.strip('/').replace('/', '_')
        timestamp = datetime.now().strftime('%d-%m-%Y-%H-%M-%S')
        filename = f'{sanitized_topic}_data_{timestamp}.csv'
        
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
            return [
                'position_x', 'position_y', 'position_z',
                'orientation_x', 'orientation_y', 'orientation_z', 'orientation_w',
                'linear_vel_x', 'linear_vel_y', 'linear_vel_z',
                'angular_vel_x', 'angular_vel_y', 'angular_vel_z'
            ]
        else:
            return []
    
    def _get_callback(self, topic, msg_type):
        def callback(data):
            try:
                timestamp = rospy.get_time()
                writer = self.csv_writers.get(topic)
                
                if writer is None:
                    rospy.logerr(f"No CSV writer found for topic {topic}")
                    return
                
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
                        data.orientation.x, data.orientation.y, data.orientation.z, data.orientation_w
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
                    row = list(data.position)[:6] 
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
                
                if row:
                    # Commented out logging 
                    #rospy.logdebug(f"Logged data for topic {topic}: {row}")
                    writer.writerow([timestamp] + row)
                else:
                    #rospy.logwarn(f"No data extracted for topic {topic} with message type {msg_type.__name__}")
                    pass
            except Exception as e:
                rospy.logerr(f"Error in callback for topic {topic}: {e}")
        
        return callback
    
    def shutdown_hook(self):
        """Clean shutdown handling"""
        rospy.loginfo("IMU Logger: Recording interrupted. Saving and closing files...")
        
        for sub in self.subscribers_list:
            sub.unregister()
            rospy.loginfo(f"Unsubscribed from topic.")
        
        for topic, file in self.csv_files.items():
            try:
                file.close()
                rospy.loginfo(f"Closed CSV file for topic {topic}")
            except Exception as e:
                rospy.logerr(f"Failed to close file for topic {topic}: {e}")
            
        rospy.loginfo("IMU Logger: All files have been saved and closed.")

if __name__ == '__main__':
    try:
        rospy.init_node('imu_logger', anonymous=True, log_level=rospy.DEBUG)
        logger = IMULogger()
        rospy.spin()
    except rospy.ROSException as e:
        rospy.logerr(f"ROS Exception: {e}")
    except rospy.ROSInterruptException:
        pass