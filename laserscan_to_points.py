#!/usr/bin/env python3
import numpy as np
import rospy
from geometry_msgs.msg import Point
import math

from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker

import tf2_ros

class Laser:
    def __init__(self):
        self.scan = rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        self.pub = rospy.Publisher('/point_positions', Marker, queue_size=1)
        
        self.marker = Marker()
        self.marker.header.frame_id = ""
        self.marker.header.stamp = rospy.Time.now()
        self.marker.type = Marker.POINTS
        self.marker.color.a = 1.0
        self.marker.color.r, self.marker.color.g, self.marker.color.b = (1., 0., 0.)
        self.marker.scale.x = 0.5
        self.marker.scale.y = 0.5

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        

    def laser_callback(self, msg):
        
        self.marker.points = []
        self.marker.header.frame_id = msg.header.frame_id
        self.marker.header.stamp = msg.header.stamp
        
        for i in range(len(msg.ranges)):
            p = Point()
            r = msg.ranges[i]
            alpha = msg.angle_min + i * msg.angle_increment
            p.x = r * math.cos(alpha)
            p.y = r * math.sin(alpha)
            self.marker.points.append(p)
        
        self.pub.publish(self.marker)


    def run(self):
        while not rospy.is_shutdown():
            rospy.spin()



if __name__ == "__main__":
    rospy.init_node('laserscan_points')
    global_frame = rospy.get_param('~global_frame', 'map')
    
    laser = Laser()
    
    try:
        laser.run()
    except rospy.ROSInterruptException:
        pass

