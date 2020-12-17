#!/usr/bin/env python3

import rospy
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import Point, TransformStamped
from visualization_msgs.msg import Marker


class TrajectoryVisualizer:
    def __init__(self):
        self.tf_sub = rospy.Subscriber('/tf', TFMessage, self.tf_callback)
        self.marker = Marker()

    def tf_callback(self, msg):
        trans = msg.transforms
        points = []
        for tf in trans:
            #print(f'frame_id: {frame_id}, child_frame_id: {child_frame_id}')
            if tf.header.frame_id == frame_id and tf.child_frame_id == child_frame_id:
                tl = tf.transform.translation
                point = Point(tl.x, tl.y, tl.z) 
                self.marker.points.append(point)
                print(self.marker.points)


    def run(self):
        while not rospy.is_shutdown():
            rospy.spin()


if __name__ == '__main__':
    rospy.init_node('trajectory_vis')
    frame_id = rospy.get_param('~frame_id', 'map')
    child_frame_id = rospy.get_param('~child_frame_id', 'base_link')

    print('Starting the trajectory visualiser node.')
    print(f'frame_id: {frame_id}, child_frame_id: {child_frame_id}')

    traj_vis = TrajectoryVisualizer()

    try:
        traj_vis.run()
    except rospy.ROSInterruptException:
        pass

