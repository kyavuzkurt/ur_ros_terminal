#!/usr/bin/env python

import rospy
from geometry_msgs.msg import WrenchStamped
import csv
import time
from datetime import datetime
import threading
import os

class RosWrenchLogger:
    def __init__(self, command_file, output_dir):
        self.command_file = command_file
        self.output_dir = output_dir
        self.logging = False
        self.wrench_data = []
        

    def command_listener(self):
        while not rospy.is_shutdown():
            with open(self.command_file, 'r') as file:
                lines = file.readlines()
                if lines:
                    command = lines[-1].strip()
                    if command == 'start' and not self.logging:
                        self.logging = True
                        now = datetime.now()
                        rospy.loginfo("Logging started. Start time:")
                        rospy.loginfo(now)
                        self.start_time = now
                    elif command == 'stop' and self.logging:
                        self.logging = False
                        now = datetime.now()
                        rospy.loginfo("Logging stopped. Stop time:")
                        rospy.loginfo(now)
                        self.save_to_csv()
                        self.wrench_data = []  # Clear data after saving to CSV
                        self.end_time = now

            # time.sleep(1)  # Check the command file every second

    def wrench_callback(self, data):
        if self.logging:
            self.wrench_data.append([
                data.wrench.force.x, data.wrench.force.y, data.wrench.force.z,
                data.wrench.torque.x, data.wrench.torque.y, data.wrench.torque.z,
                datetime.now()
            ])

    def generate_filename(self):
        now = datetime.now()
        filename = now.strftime("%d-%m-%Y-%H:%M:%S.csv")
        return os.path.join(self.output_dir, filename)

    def save_to_csv(self):
        output_csv = self.generate_filename()
        with open(output_csv, 'w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['force_x', 'force_y', 'force_z', 'torque_x', 'torque_y', 'torque_z', 'time', 'realtime'])
            writer.writerows(self.wrench_data)
        rospy.loginfo(f"Data saved to {output_csv}")

    def start(self):
        rospy.init_node('wrench_logger', anonymous=True)
        rospy.Subscriber("/wrench", WrenchStamped, self.wrench_callback)

        rospy.loginfo("Starting command listener")
        command_thread = threading.Thread(target=self.command_listener)
        command_thread.start()

        rospy.spin()

        
if __name__ == '__main__':
    command_file = '/home/moscat/Desktop/test.txt'  # Komut dosyasının yolu
    output_dir = '/home/moscat/outputlogs'    # Çıkış CSV dosyasının kaydedileceği dizin

    logger = RosWrenchLogger(command_file, output_dir)
    logger.start()

