#!/usr/bin/env python

import rospy
import numpy as np

from ar_pose.msg import ARMarker
from geometry_msgs.msg import Twist


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

    def callback(self, marker):
        # Fetch the marker pose
        pose = marker.pose.pose

        # Convert the position to a numpy vector, calculate the angle
        pos = np.array([pose.position.x, pose.position.y, pose.position.z])
        marker_angle = math.asin(pos[0] / np.linalg.norm(pos))

        # Turn the robot such that the angle gets closer to zero
        t = Twist()
        t.linear.x = computeSpeed(pos[2])
        t.angular.z = -np.sign(marker_angle) * min(
            abs(marker_angle) * 2, math.pi)

        self.pub.publish(t)


if __name__ == '__main__':
    rospy.init_node('followar')
    controller = FollowarController()
    rospy.spin()

