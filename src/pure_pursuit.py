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
        self.odom_topic = rospy.get_param("~odom_topic")
        self.speed = 10
        self.Klook = 1.5  # constant that affects lookahead
        # based on speed
        self.lookahead = self.Klook * self.speed
        self.wheelbase_length = .17  # TODO: FILL IN
        self.trajectory = utils.LineTrajectory("/followed_trajectory")
        self.traj_sub = rospy.Subscriber("/trajectory/current", PoseArray, self.trajectory_callback, queue_size=1)
        self.drive_pub = rospy.Publisher("/drive", AckermannDriveStamped, queue_size=1)
        self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self.Pursuiter, queue_size=1)

    def trajectory_callback(self, msg):
        ''' Clears the currently followed trajectory, and loads the new one from the message
        '''
        print("Receiving new trajectory:", len(msg.poses), "points")
        self.trajectory.clear()
        self.trajectory.fromPoseArray(msg)
        self.trajectory.publish_viz(duration=0.0)

    def find_closest_point(self, robot):
        """
        Finds "start point" for your robot from the trajectory at
        the current time.

        Input:
        Current trajectory of robot
        ----------------------------------
        Output:
        Tuple of (New trajectory starting at closest point, distances of segments)
        """
        # car_x=msg.pose.pose.x
        # car_y= msg.pose.pose.y

        points = np.asarray(self.trajectory.points)

        points1 = points[:-1]
        points2 = points[1:]

        p = points2 - points1

        car_x = np.full((points1.shape[0],), robot.pose.pose.x)
        car_y = np.full((points1.shape[0],), robot.pose.pose.y)

        n = np.dot(p, np.transpose(p))

        u = ((np.matmul(car_x - points1[:, 0], np.transpose(p[:, 0]))) + (
            np.matmul(car_y - points1[:, 1], np.transpose(p[:, 1])))) / n

        u[u > 1] = 1
        u[u < 0] = 0

        # to facilitate dist calc
        x_sec = points1[:, 0] + np.dot(u, np.transpose(p[:, 0])) - car_x
        y_sec = points1[:, 1] + np.dot(u, np.transpose(p[:, 1])) - car_y
        dist_sqrd = np.matmul(x_sec, np.transpose(x_sec)) + np.matmul(y_sec, np.transpose(y_sec))

        current_point_index = np.argmin(dist_sqrd)
        closest_info = (points[current_point_index:], self.trajectory.distances[current_point_index:])
        return self.get_lookahead_point(closest_info)

    def get_lookahead_point(self, closest_info):  # TODO NEED ANGELS SECTION TO KNOW CLOSEST TRAJECTORY SEGMENT
        current_pos = np.asarray((self.odom_sub.pose.pose.x, self.odom_sub.pose.pose.y))  # Centre of circle
        lookahead_dist = self.lookahead  # Radius of circle

        current_traj, current_dist = closest_info
        if (abs(current_dist[0] - self.lookahead) < 1e-6):
            return current_traj[0]

        for i in range(len(current_traj)):
            pt = current_traj[i][:]
            squared_dist = np.dot(current_pos - pt, current_pos - pt)
            if (squared_dist > self.lookahead ** 2):
                break

        P1 = closest_info[0][i-1]  # last point inside circle
        P2 = closest_info[0][i]
        segment_start = closest_info[0][i-1][:]
        segment_end = closest_info[0][i][:]
1           
        vector_traj = segment_end - segment_start  # vectors along line segment
        vector_points = segment_start - current_pos  # vectors from robot to points

        # compute coefficients

        a = np.dot(vector_traj, vector_traj)
        b = 2 * np.dot(vector_traj, vector_points)
        c = np.dot(vector_points, vector_points) - lookahead_dist ** 2

        # find discriminant. if neg then line misses circle and no real soln
        disc = b ** 2 - 4 * a * c
        if disc < 0:
            self.lookahead *= 2
            return self.get_lookahead_point(closest_info)

        # 2 intersection solns. if btw 0  and 1 the line doesnt hit the circle but would if extended
        sqrt_disc = math.sqrt(disc)
        t1 = (-b + sqrt_disc) / (2 * a)
        t2 = (-b - sqrt_disc) / (2 * a)
        if not (0 <= t1 <= 1 or 0 <= t2 <= 1):
            self.lookahead *= 2
            return self.get_lookahead_point(closest_info)

        # get point on the line
        t = max(0, min(1, - b / (2 * a)))
        return P1 + t * (P2-P1)

        
    def Pursuiter(self,msg):
        """

        """
        goal= self.find_closest_point(msg)

        position= self.odom_topic.pose.pose.position
        orientation=self.odom_topic.pose.pose.orientation
        current_pos= np.asarray([position.x,position.y])

        goal_vec= goal-current_pos

        current_rot=tf.transformations.euler_from_quaternion([orientation.x,orientation.y,orientation.z,orientation.w])[2]

        rot_matrix= np.array([[np.cos(-current_rot),-np.sin(-current_rot)], [np.sin(-current_rot), np.cos(-current_rot)]])

        rot_point= np.dot(rot_matrix, goal_vec.T)

        L1 = np.linalg.norm(rot_point)
        
        alpha= np.arctan2(rot_point[1],rot_point[0])

        steer_angle= np.arctan2((2*self.wheelbase_length * np.sin(alpha)),L1)

        new_vel=3.06(-steer_angle+1.4)(steer_angle+1.4) # the 3.06 is the speed constant (assuming 6 m/s max speed), and the 1.4 is max turning angle (assuming 1.4)

        ack=AckermannDriveStamped()
        ack.drive.steering_angle=steer_angle
        ack.drive.speed=new_vel

        self.drive_pub.publish(ack)








if __name__ == "__main__":
    rospy.init_node("pure_pursuit")
    pf = PurePursuit()
    rospy.spin()
