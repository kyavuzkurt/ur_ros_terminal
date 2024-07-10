#!/usr/bin/env python

import rospy
from geometry_msgs.msg import WrenchStamped
from moveit_commander import MoveGroupCommander, RobotCommander, PlanningSceneInterface, roscpp_initialize, roscpp_shutdown
import sys

# Eşik değerleri
FORCE_THRESHOLD_X = 10.0
FORCE_THRESHOLD_Y = 10.0
FORCE_THRESHOLD_Z = 10.0

# Hareket frekansı (saniye cinsinden)
MOVE_FREQUENCY = 0.1

class ForceAvoidance:
    def __init__(self):
        roscpp_initialize(sys.argv)
        rospy.init_node('force_avoidance_node', anonymous=True)
        
        self.robot = RobotCommander()
        self.scene = PlanningSceneInterface()
        self.group = MoveGroupCommander("manipulator")
        self.group.set_planning_time(10)  # Planlama süresini artırma

        self.force_data = None

        # Kuvvet verilerini dinlemek için subscriber
        rospy.Subscriber('/wrench', WrenchStamped, self.wrench_callback)

    def wrench_callback(self, msg):
        self.force_data = msg.wrench.force

    def avoid_force(self):
        if self.force_data is None:
            return

        force_x = self.force_data.x
        force_y = self.force_data.y
        force_z = self.force_data.z

        # Kuvvet eşik değerlerini kontrol etme
        if abs(force_x) > FORCE_THRESHOLD_X or abs(force_y) > FORCE_THRESHOLD_Y or abs(force_z) > FORCE_THRESHOLD_Z:
            rospy.loginfo(f"Force received: x={force_x}, y={force_y}, z={force_z}")

            # Kuvvetten kaçış mesafesini lineer olarak hesaplama
            move_amount_x = 0.01 * abs(force_x) if abs(force_x) > FORCE_THRESHOLD_X else 0
            move_amount_y = 0.01 * abs(force_y) if abs(force_y) > FORCE_THRESHOLD_Y else 0
            move_amount_z = 0.01 * abs(force_z) if abs(force_z) > FORCE_THRESHOLD_Z else 0

            # Mevcut pozisyonu al
            current_pose = self.group.get_current_pose().pose

            # Ters yönde hareket et
            if abs(force_x) > FORCE_THRESHOLD_X:
                current_pose.position.x += move_amount_x if force_x < 0 else -move_amount_x
            if abs(force_y) > FORCE_THRESHOLD_Y:
                current_pose.position.y -= move_amount_y if force_y < 0 else -move_amount_y  
            if abs(force_z) > FORCE_THRESHOLD_Z:
                current_pose.position.z += move_amount_z if force_z < 0 else -move_amount_z

            # Yeni pozisyonu belirle ve hareket ettir
            self.group.set_pose_target(current_pose)
            plan_tuple = self.group.plan()
            plan = plan_tuple[1]  # Planı tuple'dan çıkar
            if plan and hasattr(plan, 'joint_trajectory') and plan.joint_trajectory.points:
                self.group.execute(plan, wait=True)
                self.group.stop()
                self.group.clear_pose_targets()
            else:
                rospy.logwarn("Planning failed or no valid points in the trajectory.")

    def run(self):
        rate = rospy.Rate(1 / MOVE_FREQUENCY)
        while not rospy.is_shutdown():
            self.avoid_force()
            rate.sleep()

if __name__ == '__main__':
    try:
        force_avoidance = ForceAvoidance()
        force_avoidance.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        roscpp_shutdown()

