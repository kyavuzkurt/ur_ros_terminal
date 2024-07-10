#!/usr/bin/env python

import rospy
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Float32
import math

class ForceMagnitudePublisher:
    def __init__(self):
        rospy.init_node('force_magnitude_publisher_node', anonymous=True)
        
        # Kuvvet büyüklüğünü yayınlayan publisher
        self.force_magnitude_pub = rospy.Publisher('/force_magnitude', Float32, queue_size=10)
        
        # Kuvvet verilerini dinlemek için subscriber
        rospy.Subscriber('/wrench', WrenchStamped, self.wrench_callback)
        
        rospy.loginfo("Force magnitude publisher node started.")
    
    def wrench_callback(self, msg):
        force_x = msg.wrench.force.x
        force_y = msg.wrench.force.y
        force_z = msg.wrench.force.z
        
        # Kuvvet büyüklüğünü hesapla (Euclidean norm)
        force_magnitude = math.sqrt(force_x**2 + force_y**2 + force_z**2)
        
        # Kuvvet büyüklüğünü yayınla
        self.force_magnitude_pub.publish(force_magnitude)
        rospy.loginfo(f"Force magnitude: {force_magnitude}")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = ForceMagnitudePublisher()
        node.run()
    except rospy.ROSInterruptException:
        pass
