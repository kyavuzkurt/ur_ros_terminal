#!/usr/bin/env python

import rospy
from geometry_msgs.msg import WrenchStamped
import csv
import math
from datetime import datetime
import threading
import os
import tkinter as tk
from tkinter import ttk

class RosWrenchLogger:
    def __init__(self, output_dir):
        self.output_dir = output_dir
        self.logging = False
        self.wrench_data = []
        self.amplitude = None
        self.frequency = None
        self.a_value = None
        self.start_time = None
        self.end_time = None

    def wrench_callback(self, msg):
        if self.logging:
            force_x = msg.wrench.force.x
            force_y = msg.wrench.force.y
            force_z = msg.wrench.force.z
            torque_x = msg.wrench.torque.x
            torque_y = msg.wrench.torque.y
            torque_z = msg.wrench.torque.z
            current_time = rospy.get_time()
            self.wrench_data.append([force_x, force_y, force_z, torque_x, torque_y, torque_z, current_time])

    def generate_filename(self):
        now = datetime.now()
        filename = f"forcetorque_amp_{self.amplitude}_f_{self.frequency}_a_{self.a_value}_{now.strftime('%d-%m-%Y-%H:%M:%S')}.csv"
        return os.path.join(self.output_dir, filename)

    def save_to_csv(self):
        output_csv = self.generate_filename()
        with open(output_csv, 'w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['force_x', 'force_y', 'force_z', 'torque_x', 'torque_y', 'torque_z', 'time'])
            writer.writerows(self.wrench_data)
        rospy.loginfo(f"Data saved to {output_csv}")

    def start_logging(self):
        if not self.logging:
            self.logging = True
            self.wrench_data = []
            self.start_time = datetime.now()
            rospy.loginfo("Logging started at: " + str(self.start_time))

    def stop_logging(self):
        if self.logging:
            self.logging = False
            self.end_time = datetime.now()
            rospy.loginfo("Logging stopped at: " + str(self.end_time))
            self.save_to_csv()

    def start(self):
        rospy.init_node('wrench_logger', anonymous=True)
        rospy.Subscriber("/wrench", WrenchStamped, self.wrench_callback)

        self.create_gui()

        rospy.spin()

    def create_gui(self):
        root = tk.Tk()
        root.title("Wrench Logger")

        tk.Label(root, text="Amplitude:").grid(row=0, column=0, padx=10, pady=5)
        amplitude_entry = tk.Entry(root)
        amplitude_entry.grid(row=0, column=1, padx=10, pady=5)

        tk.Label(root, text="Frequency:").grid(row=1, column=0, padx=10, pady=5)
        frequency_var = tk.DoubleVar(value=0.1)
        frequency_menu = ttk.Combobox(root, textvariable=frequency_var)
        frequency_menu['values'] = (0.1, 0.2, 0.3, 0.4, 0.5)
        frequency_menu.grid(row=1, column=1, padx=10, pady=5)

        tk.Label(root, text="a:").grid(row=2, column=0, padx=10, pady=5)
        a_var = tk.DoubleVar(value=0.0)
        a_menu = ttk.Combobox(root, textvariable=a_var)
        a_menu['values'] = (0.0, -0.05, -0.1, -0.15, -0.2)
        a_menu.grid(row=2, column=1, padx=10, pady=5)

        def start_logging():
            self.amplitude = amplitude_entry.get()
            self.frequency = frequency_var.get()
            self.a_value = a_var.get()
            self.start_logging()

        def stop_logging():
            self.stop_logging()

        start_button = tk.Button(root, text="Start Logging", command=start_logging)
        start_button.grid(row=3, column=0, padx=10, pady=10)

        stop_button = tk.Button(root, text="Stop Logging", command=stop_logging)
        stop_button.grid(row=3, column=1, padx=10, pady=10)

        root.mainloop()

if __name__ == '__main__':
    output_dir = '/home/moscat/outputlogs'  # Directory to save output CSV

    logger = RosWrenchLogger(output_dir)
    logger.start()

