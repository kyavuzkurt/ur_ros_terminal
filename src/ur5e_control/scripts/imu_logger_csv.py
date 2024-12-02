#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Vector3Stamped, QuaternionStamped, Pose, Pose2D
from sensor_msgs.msg import Imu, MagneticField, TimeReference, JointState
from nav_msgs.msg import Odometry
import csv
import os
from datetime import datetime
from threading import Timer

from ur5e_control.srv import SetLogFolder, SetLogFolderResponse
from ur5e_control.srv import SetLoggingDuration, SetLoggingDurationResponse
from ur5e_control.srv import StartLogging, StartLoggingResponse
from ur5e_control.srv import StopLogging, StopLoggingResponse

class IMULogger:
    def __init__(self):
        self.default_log_dir = 'imu_logs'
        self.log_dir = self.default_log_dir
        self.duration_timer = None  # Timer for logging duration
        self.logging_duration = None  # Duration in seconds
        self.logging_active = False  # Logging state

        self.csv_files = {}
        self.csv_writers = {}
        self.subscribers_list = []

        self.initialize_logging_directories()
        self.initialize_subscribers()
        self.initialize_csv_files()
        self.setup_services()

        rospy.on_shutdown(self.shutdown_hook)

    def initialize_logging_directories(self):
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
            '/multicast_parser/Robot_1/pose': 'mocap_pose_data',
            '/multicast_parser/Robot_1/ground_pose': 'mocap_pose2d_data',
            '/multicast_parser/Robot_1/odom': 'mocap_odom_data'
        }

        for folder in self.data_folders.values():
            folder_path = os.path.join(self.log_dir, folder)
            if not os.path.exists(folder_path):
                os.makedirs(folder_path)
                rospy.loginfo(f"Created subdirectory: {folder_path}")
            else:
                rospy.loginfo(f"Subdirectory already exists: {folder_path}")

    def initialize_subscribers(self):
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
            '/multicast_parser/Robot_1/pose': Pose,
            '/multicast_parser/Robot_1/ground_pose': Pose2D,
            '/multicast_parser/Robot_1/odom': Odometry
        }
        self.subscribers_list = []

        for topic, msg_type in self.subscribers.items():
            try:
                sub = rospy.Subscriber(topic, msg_type, self._get_callback(topic, msg_type))
                self.subscribers_list.append(sub)
                rospy.loginfo(f"Subscribed to topic: {topic} with message type: {msg_type.__name__}")
            except Exception as e:
                rospy.logerr(f"Failed to subscribe to topic {topic}: {e}")

    def initialize_csv_files(self):
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

    def setup_services(self):
        self.set_log_folder_service = rospy.Service('set_log_folder', SetLogFolder, self.handle_set_log_folder)
        self.set_logging_duration_service = rospy.Service('set_logging_duration', SetLoggingDuration, self.handle_set_logging_duration)
        self.start_logging_service = rospy.Service('start_logging', StartLogging, self.handle_start_logging)
        self.stop_logging_service = rospy.Service('stop_logging', StopLogging, self.handle_stop_logging)
        rospy.loginfo("Services 'set_log_folder', 'set_logging_duration', 'start_logging', and 'stop_logging' are ready.")

    def handle_set_log_folder(self, req):
        try:
            new_log_dir = req.folder_name
            if not os.path.isabs(new_log_dir):
                # Make the path absolute relative to current directory
                new_log_dir = os.path.abspath(new_log_dir)
            if not os.path.exists(new_log_dir):
                os.makedirs(new_log_dir)
                rospy.loginfo(f"Created new log directory: {new_log_dir}")
            else:
                rospy.loginfo(f"Log directory already exists: {new_log_dir}")

            self.log_dir = new_log_dir
            # Reinitialize directories and CSV files
            self.initialize_logging_directories()
            self.close_all_files()
            self.initialize_csv_files()
            rospy.loginfo(f"Log folder set to: {self.log_dir}")
            return SetLogFolderResponse(success=True, message="Log folder updated successfully.")
        except Exception as e:
            rospy.logerr(f"Failed to set log folder: {e}")
            return SetLogFolderResponse(success=False, message=str(e))

    def handle_set_logging_duration(self, req):
        try:
            duration = req.duration_sec
            if duration <= 0:
                message = "Duration must be positive."
                rospy.logwarn(message)
                return SetLoggingDurationResponse(success=False, message=message)

            if self.duration_timer:
                self.duration_timer.cancel()
                rospy.loginfo("Existing duration timer canceled.")

            self.logging_duration = duration
            self.duration_timer = Timer(duration, self.stop_logging_internal)
            self.duration_timer.start()
            rospy.loginfo(f"Logging will stop after {duration} seconds.")

            # Activate logging
            if not self.logging_active:
                self.logging_active = True
                rospy.loginfo("Logging started via set_logging_duration.")

            return SetLoggingDurationResponse(success=True, message=f"Logging will stop after {duration} seconds.")
        except Exception as e:
            rospy.logerr(f"Failed to set logging duration: {e}")
            return SetLoggingDurationResponse(success=False, message=str(e))

    def handle_start_logging(self, req):
        if self.logging_active:
            message = "Logging is already active."
            rospy.logwarn(message)
            return StartLoggingResponse(success=False, message=message)
        try:
            self.logging_active = True
            rospy.loginfo("Logging started.")
            return StartLoggingResponse(success=True, message="Logging started successfully.")
        except Exception as e:
            rospy.logerr(f"Failed to start logging: {e}")
            return StartLoggingResponse(success=False, message=str(e))

    def handle_stop_logging(self, req):
        if not self.logging_active:
            message = "Logging is not active."
            rospy.logwarn(message)
            return StopLoggingResponse(success=False, message=message)
        try:
            self.logging_active = False
            if self.duration_timer:
                self.duration_timer.cancel()
                self.duration_timer = None
                rospy.loginfo("Duration timer canceled.")
            rospy.loginfo("Logging stopped.")
            return StopLoggingResponse(success=True, message="Logging stopped successfully.")
        except Exception as e:
            rospy.logerr(f"Failed to stop logging: {e}")
            return StopLoggingResponse(success=False, message=str(e))

    def stop_logging_internal(self):
        if self.logging_active:
            self.logging_active = False
            rospy.loginfo("Logging duration elapsed. Logging stopped.")

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
        elif msg_type == Pose:
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
            if not self.logging_active:
                return  # Do not log if logging is inactive

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
                    row = list(data.position)[:6] 
                elif msg_type == Pose:
                    row = [
                        data.position.x, data.position.y, data.position.z,
                        data.orientation.x, data.orientation.y,
                        data.orientation.z, data.orientation.w
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
                    writer.writerow([timestamp] + row)
                else:
                    pass
            except Exception as e:
                rospy.logerr(f"Error in callback for topic {topic}: {e}")

        return callback

    def close_all_files(self):
        for topic, file in self.csv_files.items():
            try:
                file.close()
                rospy.loginfo(f"Closed CSV file for topic {topic}")
            except Exception as e:
                rospy.logerr(f"Failed to close file for topic {topic}: {e}")
        self.csv_files = {}
        self.csv_writers = {}

    def shutdown_hook(self):
        """Clean shutdown handling"""
        rospy.loginfo("IMU Logger: Recording interrupted. Saving and closing files...")

        for sub in self.subscribers_list:
            sub.unregister()
            rospy.loginfo(f"Unsubscribed from topic.")

        self.close_all_files()

        # Cancel any active timers
        if self.duration_timer:
            self.duration_timer.cancel()

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