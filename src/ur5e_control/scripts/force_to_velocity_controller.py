#!/usr/bin/env python

import rospy
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Float64MultiArray
import threading

class ForceToVelocityController:
    def __init__(self):
        rospy.init_node('force_to_velocity_controller_node', anonymous=True)

        # Universal Robots'un velocity controller'ı olan scaled_pos_joint_traj_controller'a geçiş yapıyoruz
        self.velocity_pub = rospy.Publisher('/scaled_pos_joint_traj_controller/command', Float64MultiArray, queue_size=10)

        rospy.Subscriber('/wrench', WrenchStamped, self.wrench_callback)

        self.force_to_velocity_scale = rospy.get_param('~force_to_velocity_scale', 0.1)
        self.force_threshold = rospy.get_param('~force_threshold', 10.0)
        self.move_frequency = rospy.get_param('~move_frequency', 0.5)

        self.force_data = None
        self.lock = threading.Lock()
        self.last_move_time = rospy.Time.now()

        rospy.loginfo("Force to velocity controller node started.")

    def wrench_callback(self, msg):
        with self.lock:
            self.force_data = msg.wrench.force

    def process_force(self):
        with self.lock:
            if self.force_data is None:
                return

            current_time = rospy.Time.now()
            if (current_time - self.last_move_time).to_sec() < self.move_frequency:
                return

            force_x = self.force_data.x
            force_y = self.force_data.y
            force_z = self.force_data.z

            if abs(force_x) < self.force_threshold and abs(force_y) < self.force_threshold and abs(force_z) < self.force_threshold:
                return

            velocity_command = Float64MultiArray()
            velocity_command.data = [
                force_x * self.force_to_velocity_scale,
                force_y * self.force_to_velocity_scale,
                force_z * self.force_to_velocity_scale,
                0.0,  # İhtiyacınıza göre bunları özelleştirebilirsiniz
                0.0,
                0.0
            ]

            self.velocity_pub.publish(velocity_command)
            rospy.loginfo(f"Published velocity command: {velocity_command.data}")

            self.last_move_time = rospy.Time.now()

    def run(self):
        rate = rospy.Rate(1 / self.move_frequency)
        while not rospy.is_shutdown():
            self.process_force()
            rate.sleep()

if __name__ == '__main__':
    try:
        node = ForceToVelocityController()
        node.run()
    except rospy.ROSInterruptException:
        pass

