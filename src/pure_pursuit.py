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
        self.speed            =  10
        self.Klook             = 1.5  #constant that affects lookahead
                                      #based on speed
        self.lookahead        = Klook * self.speed
        self.wrap             = # FILL IN #
        self.wheelbase_length = # FILL IN #
        self.trajectory  = utils.LineTrajectory("/followed_trajectory")
        self.traj_sub = rospy.Subscriber("/trajectory/current", PoseArray, self.trajectory_callback, queue_size=1)
        self.drive_pub = rospy.Publisher("/drive", AckermannDriveStamped, queue_size=1)
        self.odom_sub=rospy.Subscriber(self.odom_topic,Odometry,self.find_closest_point,queue_size=1)
    def trajectory_callback(self, msg):
        ''' Clears the currently followed trajectory, and loads the new one from the message
        '''
<<<<<<< HEAD
        print("Receiving new trajectory:", len(msg.poses), "points")
=======
        print "Receiving new trajectory:", len(msg.poses), "points"

>>>>>>> d5fe8b51a95d31482f8a21fcfda1054876878416
        self.trajectory.clear()
        self.trajectory.fromPoseArray(msg)
        self.trajectory.publish_viz(duration=0.0)

<<<<<<< HEAD
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
=======
    def find_closest_point(self, msg):
        """
        Finds "start point" for your robot from the trajectory at
        the current time
        """
        #car_x=msg.pose.pose.x
        #car_y= msg.pose.pose.y

        points=np.asarray(self.trajectory.points)

        points1=points[:-1]
        points2=points[1:]

        p = points2-points1

        car_x=np.full((points1.shape[0],),msg.pose.pose.x)
        car_y=np.full((points1.shape[0],)msg.pose.pose.y)

        n= np.dot(p,np.transpose(p))

        u=((np.matmul(car_x-points1[:,0],np.transpose(p[:,0])))+(np.matmul(car_y-points1[:,1],np.transpose(p[:,1]))))/n

        u[u>1]=1
        u[u<0]=0
        
        #to facilitate dist calc
        x_sec=points1[:,0]+np.dot(u,np.transpose(p[:,0])) -car_x
        y_sec=points1[:,1]+np.dot(u,np.transpose(p[:,1]))- car_y
        dist_sqrd=np.matmul(x_sec,np.transpose(x_sec)) +np.matmul(y_sec,np.transpose(y_sec))


        current_point_index=np.argmin(dist_sqrd)

        
>>>>>>> d5fe8b51a95d31482f8a21fcfda1054876878416


if __name__=="__main__":
    rospy.init_node("pure_pursuit")
    pf = PurePursuit()
    rospy.spin()
