#!/usr/bin/env python

import rospy
import numpy as np
import tf

from ar_pose.msg import ARMarker
from geometry_msgs.msg import Twist, PoseStamped


import math


def computeSpeed(dist):
    v_max = 1.0  # m/s
    dist_min = 0.20  # m
    dist_max = 2.0  # m
    if dist < dist_min:
        return 0.0
    else:
        return v_max * min(1.0, dist / dist_max)


class FollowarController:

    def __init__(self):
        self.sub = rospy.Subscriber("ar_pose_marker", ARMarker, self.callback)
        self.pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        self.tfl = tf.TransformListener()

    def callback(self, marker):
        # Build a PoseStamped from the marker
        try:
            self.tfl.waitForTransform(marker.header.frame_id, "base_link",
                marker.header.stamp, rospy.Duration(0.1))
            pose = PoseStamped(header=marker.header, pose=marker.pose.pose)
            pose = self.tfl.transformPose("base_link", pose).pose
        except Exception as e:
            rospy.logwarn(str(e))
            return

        # Convert the position to a numpy vector, calculate the angle
        pos = np.array([pose.position.x, pose.position.y, pose.position.z])
        marker_angle = math.atan2(pos[1], pos[0])

        # Turn the robot such that the angle gets closer to zero
        t = Twist()
        t.linear.x = computeSpeed(math.hypot(pos[0], pos[1]))
        t.angular.z = np.sign(marker_angle) * min(
            abs(marker_angle) * 4, math.pi)

        self.pub.publish(t)


if __name__ == '__main__':
    rospy.init_node('followar')
    controller = FollowarController()
    rospy.spin()

