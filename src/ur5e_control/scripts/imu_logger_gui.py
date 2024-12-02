#!/usr/bin/env python

import sys
import os
import rospy
from PyQt5 import QtWidgets, QtGui, QtCore
from ur5e_control.srv import SetLogFolder, SetLoggingDuration
from ur5e_control.srv import StartLogging, StopLogging  # Assumed service imports
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from geometry_msgs.msg import Vector3Stamped, QuaternionStamped, Pose, Pose2D
from sensor_msgs.msg import Imu, MagneticField, TimeReference, JointState
from nav_msgs.msg import Odometry

class MplCanvas(FigureCanvas):
    def __init__(self, parent=None, width=5, height=4, dpi=100, topic_name=''):
        fig = Figure(figsize=(width, height), dpi=dpi)
        self.axes = fig.add_subplot(111)
        super(MplCanvas, self).__init__(fig)
        self.topic_name = topic_name
        self.x_data = []
        self.y_data = []
        self.line, = self.axes.plot([], [], 'r-')
        self.axes.set_title(topic_name)
        self.axes.set_xlabel('Time (s)')
        self.axes.set_ylabel('Value')
    
    def update_plot(self, timestamp, value):
        self.x_data.append(timestamp)
        self.y_data.append(value)
        self.line.set_data(self.x_data, self.y_data)
        self.axes.relim()
        self.axes.autoscale_view()
        self.draw()

class IMULoggerGUI(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.init_ros()
        self.init_ui()
        self.selected_topics = []
        self.canvas_dict = {}
        self.subscriber_dict = {}
        self.logging_active = False  # Logging state

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
        self.setGeometry(100, 100, 1400, 800)  # Increased width to accommodate new controls
        
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
        
        # Start and Stop Logging Buttons
        self.start_logging_button = QtWidgets.QPushButton('Start Logging')
        self.start_logging_button.clicked.connect(self.start_logging)
        self.stop_logging_button = QtWidgets.QPushButton('Stop Logging')
        self.stop_logging_button.clicked.connect(self.stop_logging)
        self.stop_logging_button.setEnabled(False)  # Initially disabled
        
        controls_layout.addWidget(self.start_logging_button)
        controls_layout.addWidget(self.stop_logging_button)
        
        # Logging Indicator Light
        self.logging_indicator = QtWidgets.QLabel()
        self.logging_indicator.setFixedSize(20, 20)
        self.logging_indicator.setStyleSheet("background-color: red; border-radius: 10px;")
        self.logging_indicator.setToolTip("Logging Status")
        
        controls_layout.addWidget(QtWidgets.QLabel("Logging Status:"))
        controls_layout.addWidget(self.logging_indicator)
        
        layout.addLayout(controls_layout)
        
        # Middle Controls: Topic Selection
        topics_layout = QtWidgets.QHBoxLayout()
        
        self.topic_checkboxes = {}
        topics = [
            '/filter/euler',
            '/filter/free_acceleration',
            '/filter/quaternion',
            '/imu/acceleration',
            '/imu/angular_velocity',
            '/imu/data',
            '/imu/mag',
            '/imu/time_ref',
            '/joint_states',
            '/multicast_parser/Robot_1/pose',
            '/multicast_parser/Robot_1/ground_pose',
            '/multicast_parser/Robot_1/odom'
        ]
        
        topics_group = QtWidgets.QGroupBox("Select Topics to Plot")
        topics_box = QtWidgets.QVBoxLayout()
        for topic in topics:
            cb = QtWidgets.QCheckBox(topic)
            cb.stateChanged.connect(self.topic_selection_changed)
            self.topic_checkboxes[topic] = cb
            topics_box.addWidget(cb)
        topics_group.setLayout(topics_box)
        topics_layout.addWidget(topics_group)
        
        layout.addLayout(topics_layout)
        
        # Real-time Plots Area
        self.plots_layout = QtWidgets.QGridLayout()
        layout.addLayout(self.plots_layout)
        
        # Logo
        logo_path = os.path.join(os.path.dirname(__file__), '../resources/logo.png')
        if os.path.exists(logo_path):
            logo = QtGui.QPixmap(logo_path)
            logo_label = QtWidgets.QLabel()
            logo_label.setPixmap(logo.scaled(200, 200, QtCore.Qt.KeepAspectRatio))
            layout.addWidget(logo_label, alignment=QtCore.Qt.AlignCenter)
        else:
            rospy.logwarn(f"Logo file not found at {logo_path}")
        
        self.setLayout(layout)
    
    def set_log_folder(self):
        folder_name = self.folder_input.text()
        try:
            resp = self.set_log_folder_srv(folder_name)
            if resp.success:
                QtWidgets.QMessageBox.information(self, "Success", resp.message)
            else:
                QtWidgets.QMessageBox.warning(self, "Failure", resp.message)
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            QtWidgets.QMessageBox.critical(self, "Error", f"Service call failed: {e}")
    
    def set_logging_duration(self):
        duration_text = self.duration_input.text()
        try:
            duration = float(duration_text)
            resp = self.set_logging_duration_srv(duration)
            if resp.success:
                self.logging_active = True
                self.update_logging_indicator()
                self.start_logging_button.setEnabled(False)  # Disable start button
                self.stop_logging_button.setEnabled(True)
                QtWidgets.QMessageBox.information(self, "Success", resp.message)
                # Set a timer to update the UI after the duration
                QtCore.QTimer.singleShot(int(duration * 1000), self.on_logging_duration_elapsed)
            else:
                QtWidgets.QMessageBox.warning(self, "Failure", resp.message)
        except ValueError:
            QtWidgets.QMessageBox.warning(self, "Invalid Input", "Please enter a valid number for duration.")
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            QtWidgets.QMessageBox.critical(self, "Error", f"Service call failed: {e}")
    
    def on_logging_duration_elapsed(self):
        self.logging_active = False
        self.update_logging_indicator()
        self.start_logging_button.setEnabled(True)
        self.stop_logging_button.setEnabled(False)
        QtWidgets.QMessageBox.information(self, "Logging Duration", "Logging duration has elapsed.")

    def start_logging(self):
        try:
            resp = self.start_logging_srv()
            if resp.success:
                self.logging_active = True
                self.update_logging_indicator()
                self.start_logging_button.setEnabled(False)
                self.stop_logging_button.setEnabled(True)
                QtWidgets.QMessageBox.information(self, "Logging Started", resp.message)
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
    
    def topic_selection_changed(self, state):
        checkbox = self.sender()
        topic = checkbox.text()
        if state == QtCore.Qt.Checked:
            self.add_topic_plot(topic)
        else:
            self.remove_topic_plot(topic)
    
    def add_topic_plot(self, topic):
        if topic in self.canvas_dict:
            return  # Already added
        
        # Create a new plot canvas
        canvas = MplCanvas(self, width=5, height=4, dpi=100, topic_name=topic)
        row = len(self.canvas_dict) // 2
        col = len(self.canvas_dict) % 2
        self.plots_layout.addWidget(canvas, row, col)
        self.canvas_dict[topic] = canvas
        
        # Subscribe to the topic
        callback = self.create_matplotlib_callback(topic, canvas)
        msg_type = self.get_msg_type(topic)
        if msg_type is not None:
            subscriber = rospy.Subscriber(topic, msg_type, callback)
            self.subscriber_dict[topic] = subscriber
        else:
            rospy.logwarn(f"No message type found for topic {topic}")
    
    def remove_topic_plot(self, topic):
        if topic not in self.canvas_dict:
            return
        
        # Remove plot from layout
        canvas = self.canvas_dict.pop(topic)
        self.plots_layout.removeWidget(canvas)
        canvas.setParent(None)
        
        # Unsubscribe from the topic
        subscriber = self.subscriber_dict.pop(topic, None)
        if subscriber:
            subscriber.unregister()
    
    def get_msg_type(self, topic):
        topic_types = {
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
        return topic_types.get(topic, None)
    
    def create_matplotlib_callback(self, topic, canvas):
        def callback(msg):
            timestamp = rospy.get_time()
            value = self.extract_value(msg, topic)
            if value is not None:
                canvas.update_plot(timestamp, value)
        return callback
    
    def extract_value(self, msg, topic):
        # Extract a single value for plotting; customize based on topic
        if topic == '/filter/euler':
            return msg.vector.z  # Example: yaw
        elif topic == '/filter/free_acceleration':
            return (msg.vector.x**2 + msg.vector.y**2 + msg.vector.z**2)**0.5
        elif topic == '/filter/quaternion':
            return msg.quaternion.w  # Example: w component
        elif topic == '/imu/acceleration':
            return (msg.vector.x**2 + msg.vector.y**2 + msg.vector.z**2)**0.5
        elif topic == '/imu/angular_velocity':
            return (msg.vector.x**2 + msg.vector.y**2 + msg.vector.z**2)**0.5
        elif topic == '/imu/data':
            return msg.linear_acceleration.z  # Example: linear acceleration Z
        elif topic == '/imu/mag':
            return (msg.magnetic_field.x**2 + msg.magnetic_field.y**2 + msg.magnetic_field.z**2)**0.5
        elif topic == '/imu/time_ref':
            return msg.time_ref.to_sec()
        elif topic == '/joint_states':
            return msg.position[2]  # Example: elbow joint
        elif topic == '/multicast_parser/Robot_1/pose':
            return msg.position.z  # Example: Z position
        elif topic == '/multicast_parser/Robot_1/ground_pose':
            return msg.theta
        elif topic == '/multicast_parser/Robot_1/odom':
            return msg.twist.twist.linear.x
        else:
            return None
    
    def closeEvent(self, event):
        # Unsubscribe from all topics on close
        for subscriber in self.subscriber_dict.values():
            subscriber.unregister()
        event.accept()

def main():
    app = QtWidgets.QApplication(sys.argv)
    gui = IMULoggerGUI()
    gui.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
