#!/usr/bin/env python
import math
import numpy as np

import rospy
import numpy as np
import time
import utils
import tf

from geometry_msgs.msg import PoseArray, PoseStamped
from visualization_msgs.msg import Marker
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry

class PurePursuit(object):
    """ Implements Pure Pursuit trajectory tracking with a fixed lookahead and speed.
    """
    def __init__(self):
        self.odom_topic       = rospy.get_param("~odom_topic")
        self.lookahead        = # FILL IN #
        self.speed            = # FILL IN #
        self.wrap             = # FILL IN #
        self.wheelbase_length = # FILL IN #
        self.trajectory  = utils.LineTrajectory("/followed_trajectory")
        self.traj_sub = rospy.Subscriber("/trajectory/current", PoseArray, self.trajectory_callback, queue_size=1)
        self.drive_pub = rospy.Publisher("/drive", AckermannDriveStamped, queue_size=1)

    def trajectory_callback(self, msg):
        ''' Clears the currently followed trajectory, and loads the new one from the message
        '''
        print("Receiving new trajectory:", len(msg.poses), "points")
        self.trajectory.clear()
        self.trajectory.fromPoseArray(msg)
        self.trajectory.publish_viz(duration=0.0)

    def get_lookahead_point(self, msg): #TODO NEED ANGELS SECTION TO KNOW CLOSEST TRAJECTORY SEGMENT
        Q = self.p  # Centre of circle
        r = self.r  # Radius of circle
        # vector
        P1 = constraint.point1  # Start of line segment
        V = constraint.point2 - P1  # Vector along line segment

        #compute coefficients
        # a = np.dot(vec_v, vec_v)
        # b = 2 * np.dot(vec_v, vec_w)
        # c = np.dot(vec_w, vec_w) - self.lookahead ** 2


        a = V.dot(V)
        b = 2 * V.dot(P1 - Q)
        c = P1.dot(P1) + Q.dot(Q) - 2 * P1.dot(Q) - r ** 2

        #find discriminant. if neg then line misses circle and no real soln
        disc = b ** 2 - 4 * a * c
        if disc < 0:
            return False, None

        #2 solns. if btw 0  and 1 the line doesnt hit the circle but would if extended
        sqrt_disc = math.sqrt(disc)
        t1 = (-b + sqrt_disc) / (2 * a)
        t2 = (-b - sqrt_disc) / (2 * a)
        if not (0 <= t1 <= 1 or 0 <= t2 <= 1):
            return False, None

        #get point on the line
        t = max(0, min(1, - b / (2 * a)))
        return True, P1 + t * V


if __name__=="__main__":
    rospy.init_node("pure_pursuit")
    pf = PurePursuit()
    rospy.spin()
