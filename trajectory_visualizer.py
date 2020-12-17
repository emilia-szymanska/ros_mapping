#!/usr/bin/env python3

import rospy
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import Point, TransformStamped
from visualization_msgs.msg import Marker


class TrajectoryVisualizer:
    def __init__(self):
        self.tf_sub = rospy.Subscriber('/tf', TFMessage, self.tf_callback)
        self.robot_pos = rospy.Publisher('/robot_positions', Marker, queue_size = 1)
        self.marker = Marker()
        self.marker.type = Marker.LINE_STRIP
        self.marker.header.stamp = rospy.Time(0)
        self.marker.color.a = 1.0
        self.marker.color.r, self.marker.color.g, self.marker.color.b = (0., 0.47, 0.75)
        self.marker.scale.x = 0.5
        self.marker.scale.y = 0.5
        self.marker.pose.orientation.w = 1.


    def tf_callback(self, msg):
        trans = msg.transforms
        for tf in trans:
            if tf.header.frame_id == frame_id and tf.child_frame_id == child_frame_id:
                if tf.header.stamp < self.marker.header.stamp:
                    print('Timestamp has jumped backwards, clearing the trajectory')
                    self.marker.points = []
                else:
                    tl = tf.transform.translation
                    point = Point(tl.x, tl.y, tl.z) 
                    self.marker.points.append(point)
                    
                self.marker.header.frame_id = tf.header.frame_id
                self.marker.header.stamp = tf.header.stamp
        # here or inside for?
        self.robot_pos.publish(self.marker)

    def run(self):
        while not rospy.is_shutdown():
            rospy.spin()


if __name__ == '__main__':
    rospy.init_node('trajectory_vis')
    #here?
    frame_id = rospy.get_param('~frame_id', 'map')
    child_frame_id = rospy.get_param('~child_frame_id', 'base_link')

    print('Starting the trajectory visualiser node.')
    print(f'frame_id: {frame_id}, child_frame_id: {child_frame_id}')

    traj_vis = TrajectoryVisualizer()

    try:
        traj_vis.run()
    except rospy.ROSInterruptException:
        pass

