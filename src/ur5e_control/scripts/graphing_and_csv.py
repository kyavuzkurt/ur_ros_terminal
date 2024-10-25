#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
import csv
import time
from datetime import datetime
import threading
import os

class JointStateLogger:
    def __init__(self):
        rospy.init_node('joint_state_logger', anonymous=True)
        
        # Create filename with timestamp
        timestamp = datetime.now().strftime('%d-%m-%Y-%H:%M')
        self.filename = f'joint_states_{timestamp}.csv'
        
        
        with open(self.filename, 'w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['Timestamp', 'shoulder_pan', 'shoulder_lift', 'elbow',
                           'wrist_1', 'wrist_2', 'wrist_3'])
        
        # Subscribe to joint states topic
        self.sub = rospy.Subscriber('/joint_states', JointState, self.callback)
        
    def callback(self, data):
        timestamp = rospy.get_time()
        
        with open(self.filename, 'a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([timestamp] + list(data.position))

if __name__ == '__main__':
    try:
        logger = JointStateLogger()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS node interrupted by user")
        pass
