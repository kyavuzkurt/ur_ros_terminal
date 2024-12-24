#!/usr/bin/env python

import sys
import os
import rospy
from PyQt5 import QtWidgets, QtGui, QtCore
from ur5e_control.srv import SetLogFolder, SetLoggingDuration
from ur5e_control.srv import StartLogging, StopLogging  # Assumed service imports
from pyqtgraph import PlotWidget, mkPen
import pyqtgraph as pg
from geometry_msgs.msg import Vector3Stamped, QuaternionStamped, Pose, Pose2D
from sensor_msgs.msg import Imu, MagneticField, TimeReference, JointState
from nav_msgs.msg import Odometry
import subprocess

class IMULoggerGUI(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.init_ros()
        self.init_ui()
        self.selected_topics = []
        self.canvas_dict = {}
        self.subscriber_dict = {}
        self.logging_active = False  # Logging state
        self.folder_set = False  # Track if folder is set
        self.logging_duration = 0  # Store logging duration

    def init_ros(self):
        rospy.init_node('imu_logger_gui', anonymous=True)
        rospy.wait_for_service('set_log_folder')
        rospy.wait_for_service('set_logging_duration')
        rospy.wait_for_service('start_logging')  # Wait for start_logging service
        rospy.wait_for_service('stop_logging')   # Wait for stop_logging service
        self.set_log_folder_srv = rospy.ServiceProxy('set_log_folder', SetLogFolder)
        self.set_logging_duration_srv = rospy.ServiceProxy('set_logging_duration', SetLoggingDuration)
        self.start_logging_srv = rospy.ServiceProxy('start_logging', StartLogging)  # Service proxy
        self.stop_logging_srv = rospy.ServiceProxy('stop_logging', StopLogging)     # Service proxy

    def init_ui(self):
        self.setWindowTitle('IMU Logger GUI')
        self.setGeometry(100, 100, 1400, 800)  
        
        layout = QtWidgets.QVBoxLayout()
        
        # Top Controls
        controls_layout = QtWidgets.QHBoxLayout()
        
        # Log Folder Name
        self.folder_label = QtWidgets.QLabel('Log Folder:')
        self.folder_input = QtWidgets.QLineEdit('imu_logs')
        self.folder_button = QtWidgets.QPushButton('Set Folder')
        self.folder_button.clicked.connect(self.set_log_folder)
        
        controls_layout.addWidget(self.folder_label)
        controls_layout.addWidget(self.folder_input)
        controls_layout.addWidget(self.folder_button)
        
        # Logging Duration
        self.duration_label = QtWidgets.QLabel('Logging Duration (s):')
        self.duration_input = QtWidgets.QLineEdit()
        self.duration_button = QtWidgets.QPushButton('Set Duration')
        self.duration_button.clicked.connect(self.set_logging_duration)
        
        controls_layout.addWidget(self.duration_label)
        controls_layout.addWidget(self.duration_input)
        controls_layout.addWidget(self.duration_button)
        
        layout.addLayout(controls_layout)

        # Centered Start and Stop Logging Buttons
        logging_buttons_layout = QtWidgets.QHBoxLayout()
        logging_buttons_layout.addStretch()  # Add stretchable space before buttons
        
        self.start_logging_button = QtWidgets.QPushButton('Start Logging')
        self.start_logging_button.clicked.connect(self.start_logging)
        logging_buttons_layout.addWidget(self.start_logging_button)
        
        self.stop_logging_button = QtWidgets.QPushButton('Stop Logging')
        self.stop_logging_button.clicked.connect(self.stop_logging)
        self.stop_logging_button.setEnabled(False)  # Initially disabled
        logging_buttons_layout.addWidget(self.stop_logging_button)
        
        logging_buttons_layout.addStretch()  # Add stretchable space after buttons
        layout.addLayout(logging_buttons_layout)

        # Centered Logging Indicator Light
        logging_status_layout = QtWidgets.QHBoxLayout()
        logging_status_layout.addStretch()  # Add stretchable space before the status
        
        logging_status_label = QtWidgets.QLabel("Logging Status:")
        self.logging_indicator = QtWidgets.QLabel()
        self.logging_indicator.setFixedSize(20, 20)
        self.logging_indicator.setStyleSheet("background-color: red; border-radius: 10px;")
        self.logging_indicator.setToolTip("Logging Status")
        
        logging_status_layout.addWidget(logging_status_label)
        logging_status_layout.addWidget(self.logging_indicator)
        
        logging_status_layout.addStretch()  # Add stretchable space after the status
        layout.addLayout(logging_status_layout)

        # Open PlotJuggler Button
        self.plotjuggler_button = QtWidgets.QPushButton('Open PlotJuggler')
        self.plotjuggler_button.setFixedSize(150, 40)  # Adjusted size for better text fit
        self.plotjuggler_button.clicked.connect(self.launch_plotjuggler)
        layout.addWidget(self.plotjuggler_button, alignment=QtCore.Qt.AlignCenter)

        # Logo at the bottom right corner
        logo_path = os.path.join(os.path.dirname(__file__), '../resources/logo.png')
        if os.path.exists(logo_path):
            logo = QtGui.QPixmap(logo_path)
            logo_label = QtWidgets.QLabel()
            logo_label.setPixmap(logo.scaled(100, 100, QtCore.Qt.KeepAspectRatio))
            logo_layout = QtWidgets.QHBoxLayout()
            logo_layout.addStretch()
            logo_layout.addWidget(logo_label, alignment=QtCore.Qt.AlignBottom | QtCore.Qt.AlignRight)
            layout.addLayout(logo_layout)
        else:
            rospy.logwarn(f"Logo file not found at {logo_path}")
        
        self.setLayout(layout)
    
    def set_log_folder(self):
        folder_name = self.folder_input.text()
        try:
            resp = self.set_log_folder_srv(folder_name)
            if resp.success:
                self.folder_set = True  # Mark folder as set
                QtWidgets.QMessageBox.information(self, "Success", resp.message)
            else:
                QtWidgets.QMessageBox.warning(self, "Failure", resp.message)
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            QtWidgets.QMessageBox.critical(self, "Error", f"Service call failed: {e}")
    
    def set_logging_duration(self):
        duration_text = self.duration_input.text()
        try:
            self.logging_duration = float(duration_text)  # Store duration
            QtWidgets.QMessageBox.information(self, "Success", "Duration set successfully.")
        except ValueError:
            QtWidgets.QMessageBox.warning(self, "Invalid Input", "Please enter a valid number for duration.")
    
    def on_logging_duration_elapsed(self):
        self.stop_logging()
        self.update_logging_indicator()
        self.start_logging_button.setEnabled(True)
        self.stop_logging_button.setEnabled(False)
        QtWidgets.QMessageBox.information(self, "Logging Duration", "Logging duration has elapsed.")

    def start_logging(self):
        if not self.folder_set:
            QtWidgets.QMessageBox.warning(self, "Folder Not Set", "Please set the log folder before starting logging.")
            return

        try:
            resp = self.start_logging_srv()
            if resp.success:
                self.logging_active = True
                self.update_logging_indicator()
                self.start_logging_button.setEnabled(False)
                self.stop_logging_button.setEnabled(True)
                QtWidgets.QMessageBox.information(self, "Logging Started", resp.message)
                # Start the timer for logging duration
                QtCore.QTimer.singleShot(int(self.logging_duration * 1000), self.on_logging_duration_elapsed)
            else:
                QtWidgets.QMessageBox.warning(self, "Failed to Start Logging", resp.message)
        except rospy.ServiceException as e:
            rospy.logerr(f"Start Logging service failed: {e}")
            QtWidgets.QMessageBox.critical(self, "Error", f"Start Logging service failed: {e}")
    
    def stop_logging(self):
        try:
            resp = self.stop_logging_srv()
            if resp.success:
                self.logging_active = False
                self.update_logging_indicator()
                self.start_logging_button.setEnabled(True)
                self.stop_logging_button.setEnabled(False)
                QtWidgets.QMessageBox.information(self, "Logging Stopped", resp.message)
            else:
                QtWidgets.QMessageBox.warning(self, "Failed to Stop Logging", resp.message)
        except rospy.ServiceException as e:
            rospy.logerr(f"Stop Logging service failed: {e}")
            QtWidgets.QMessageBox.critical(self, "Error", f"Stop Logging service failed: {e}")
    
    def update_logging_indicator(self):
        if self.logging_active:
            self.logging_indicator.setStyleSheet("background-color: green; border-radius: 10px;")
        else:
            self.logging_indicator.setStyleSheet("background-color: red; border-radius: 10px;")
    

    def closeEvent(self, event):
        # Unsubscribe from all topics on close
        for subscriber in self.subscriber_dict.values():
            subscriber.unregister()
        event.accept()

    def launch_plotjuggler(self):
        try:
            # Ensure ROS environment is sourced
            ros_env = os.environ.copy()
            ros_env['ROS_MASTER_URI'] = rospy.get_param('/ros_master_uri', 'http://localhost:11311')
            ros_env['ROS_PACKAGE_PATH'] = rospy.get_param('/ros_package_path', '')

            # Launch PlotJuggler using rosrun
            subprocess.Popen(['rosrun', 'plotjuggler', 'plotjuggler'], env=ros_env)
        except Exception as e:
            rospy.logerr(f"Failed to launch PlotJuggler: {e}")
            QtWidgets.QMessageBox.critical(self, "Error", f"Failed to launch PlotJuggler: {e}")

def main():
    app = QtWidgets.QApplication(sys.argv)
    gui = IMULoggerGUI()
    gui.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
