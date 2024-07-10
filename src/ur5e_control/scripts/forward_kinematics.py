#!/usr/bin/env python

import rospy
import sys
from geometry_msgs.msg import WrenchStamped
from sensor_msgs.msg import JointState
from moveit_commander import RobotCommander, MoveGroupCommander, roscpp_initialize, roscpp_shutdown
import threading

# Eşik değerleri
FORCE_THRESHOLD = 10.0
# Hareket frekansı (saniye cinsinden)
MOVE_FREQUENCY = 0.5
# Hareket büyüklüğü
MOVE_AMOUNT = 0.1  # Radyan cinsinden

class ForwardKinematicsDemo:
    def __init__(self):
        roscpp_initialize(sys.argv)
        rospy.init_node('forward_kinematics_demo_node', anonymous=True)
        
        self.robot = RobotCommander()
        self.group = MoveGroupCommander("manipulator")

        self.axis_choice = rospy.get_param('~axis_choice', 'xyz')

        self.force_data = None
        self.lock = threading.Lock()
        self.last_move_time = rospy.Time.now()

        rospy.Subscriber('/wrench', WrenchStamped, self.wrench_callback)

        rospy.sleep(1)  

    def wrench_callback(self, msg):
        with self.lock:
            self.force_data = msg.wrench.force

    def process_force(self):
        with self.lock:
            if self.force_data is None:
                return

            current_time = rospy.Time.now()
            if (current_time - self.last_move_time).to_sec() < MOVE_FREQUENCY:
                return

            force_x = self.force_data.x
            force_y = self.force_data.y
            force_z = self.force_data.z

            if 'x' in self.axis_choice and abs(force_x) > FORCE_THRESHOLD:
                self.apply_joint_movements('x', force_x)
            if 'y' in self.axis_choice and abs(force_y) > FORCE_THRESHOLD:
                self.apply_joint_movements('y', force_y)
            if 'z' in self.axis_choice and abs(force_z) > FORCE_THRESHOLD:
                self.apply_joint_movements('z', force_z)

            self.last_move_time = rospy.Time.now()

    def apply_joint_movements(self, axis, force):
        rospy.loginfo(f"Applying joint movements for {axis}-axis with force {force}")

        joint_positions = self.group.get_current_joint_values()

        if axis == 'x':
            joint_positions[0] += MOVE_AMOUNT if force > 0 else -MOVE_AMOUNT  
            joint_positions[1] -= MOVE_AMOUNT if force > 0 else MOVE_AMOUNT 
        elif axis == 'y':
            joint_positions[2] += MOVE_AMOUNT if force > 0 else -MOVE_AMOUNT  
            joint_positions[3] -= MOVE_AMOUNT if force > 0 else MOVE_AMOUNT  
        elif axis == 'z':
            joint_positions[4] += MOVE_AMOUNT if force > 0 else -MOVE_AMOUNT  
            joint_positions[5] -= MOVE_AMOUNT if force > 0 else MOVE_AMOUNT  


        self.send_joint_positions(joint_positions)

    def send_joint_positions(self, joint_positions):
        self.group.go(joint_positions, wait=True)
        self.group.stop()
        rospy.sleep(MOVE_FREQUENCY)

    def run(self):
        rate = rospy.Rate(1 / MOVE_FREQUENCY)
        while not rospy.is_shutdown():
            self.process_force()
            rate.sleep()

if __name__ == '__main__':
    try:
        demo = ForwardKinematicsDemo()
        demo.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        roscpp_shutdown()

