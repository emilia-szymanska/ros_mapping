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
        self.marker.header.stamp = rospy.Time(0)
        self.marker.type = Marker.POINTS
        self.marker.color.a = 1.0
        self.marker.color.r, self.marker.color.g, self.marker.color.b = (1., 0., 0.)
        self.marker.scale.x = 0.25
        self.marker.scale.y = 0.25
        self.marker.pose.orientation.w = 1

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        self.n = 0

    def laser_callback(self, msg):
        
        if msg.header.stamp < self.marker.header.stamp:
            print('Timestamp has jumped backwards, clearing the buffer.')
            self.marker.header.stamp = msg.header.stamp
            self.marker.points.clear()
            self.tf_buffer.clear()
            self.n = 0
            return
        
        self.n += 1
        if accumulate_points == True and accumulate_every_n != self.n:
            return

        try:
            tf_odom_laser = self.tf_buffer.lookup_transform(global_frame,
                                                            msg.header.frame_id, 
                                                            msg.header.stamp)
            
            quaternion = tf_odom_laser.transform.rotation
            trans = tf_odom_laser.transform.translation
            yaw = 2 * math.atan2(quaternion.z, quaternion.w)

            if accumulate_points == False:
                self.marker.points.clear()
            else:
                self.n = 0

            self.marker.header.stamp = msg.header.stamp
            self.marker.header.frame_id = global_frame
        
            for i in range(len(msg.ranges)):
                p = Point()
                r = msg.ranges[i]
                alpha = msg.angle_min + i * msg.angle_increment + yaw
                p.x = r * math.cos(alpha) + trans.x
                p.y = r * math.sin(alpha) + trans.y
                self.marker.points.append(p)
        
            self.pub.publish(self.marker)
        
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            print("Tf exception")
            return


    def run(self):
        while not rospy.is_shutdown():
            rospy.spin()



if __name__ == "__main__":
    rospy.init_node('laserscan_points')
    global_frame = rospy.get_param('~global_frame', 'map')
    accumulate_points = rospy.get_param('~accumulate_points', False)
    accumulate_every_n = rospy.get_param('~accumulate_every_n', 50)
    
    print('Starting the laser scan visualizer node.')
    print(f'global_frame: {global_frame}, accumulate_points: {accumulate_points}, accumulate_every_n: {accumulate_every_n}')
    
    laser = Laser()
    
    try:
        laser.run()
    except rospy.ROSInterruptException:
        pass

