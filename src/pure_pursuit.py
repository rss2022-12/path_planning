#!/usr/bin/env python
import math
import numpy as np

import rospy
import numpy as np
import time
import utils
import tf

from geometry_msgs.msg import PoseArray, PoseStamped, Point
from visualization_msgs.msg import Marker
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry


class PurePursuit(object):
    """ Implements Pure Pursuit trajectory tracking with a fixed lookahead and speed.
    """

    def __init__(self):
        print("INITIALIZED")
        #self.odom_topic = rospy.get_param("~odom_topic")
        self.speed = 1
        self.max_speed=5
        #self.Klook = 0.2  # constant that affects lookahead
        self.received_trajectory = False
        # based on speed
        self.lookahead = 0.7
        self.max_look= 4.2
        self.wheelbase_length = .3
        self.trajectory = utils.LineTrajectory("/followed_trajectory")
        self.traj_sub = rospy.Subscriber("/trajectory/current", PoseArray, self.trajectory_callback, queue_size=1)
        self.drive_pub = rospy.Publisher("/drive", AckermannDriveStamped, queue_size=1)
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.Pursuiter, queue_size=1)

        self.look_index = 0 
        self.last_waypoint = 0 
        self.last_goal=np.array([0,0])

        self.closestpub=rospy.Publisher("/closest",Marker,queue_size=1)
        self.look_circle=rospy.Publisher("/look_circle",Marker,queue_size=1)
        
        
    def trajectory_callback(self, msg):
        ''' Clears the currently followed trajectory, and loads the new one from the message
        '''
        print("Receiving new trajectory:", len(msg.poses), "points")
        self.trajectory.clear()
        self.trajectory.fromPoseArray(msg)
        self.trajectory.publish_viz(duration=0.0)
        self.last_goal=np.array([0,0])
        self.last_waypoint=0
        self.look_index=0
        self.received_trajectory = True


    def visualizer(self, pos_x, pos_y, color=[0,1,0], shape=SPHERE):
        poii=Marker()
        poii.header.frame_id="map"
        poii.pose.position.x=pos_x
        poii.pose.position.y=pos_y
        poii.pose.position.z=0
        poii.pose.orientation.x=0
        poii.pose.orientation.y=0
        poii.pose.orientation.z=0
        poii.pose.orientation.w=0
        poii.color.a=1
        poii.color.r=color[0]
        poii.color.g=color[1]
        poii.color.g=color[2]
        poii.type=shape
        return poii
        
    def find_closest_point(self, robot):
        
        Finds "start point" for your robot from the trajectory at
        the current time.

        Input:
        Current trajectory of robot
        ----------------------------------
        Output:
        Tuple of (New trajectory starting at closest point, distances of segments)
        
        # car_x=msg.pose.pose.x
        # car_y= msg.pose.pose.y

        points = np.asarray(self.trajectory.points)

        points1 = points[:-1]
        points2 = points[1:]

        p = points2 - points1

        print("p shape:", p.shape)
        print("point1 shape:", points1.shape)
        print("points2 shape:", points2.shape)

        car_x = np.full((points1.shape[0],), robot.pose.pose.position.x)
        car_y = np.full((points1.shape[0],), robot.pose.pose.position.y)

        n = p[:,0]**2 + p[:,1]**2

        u = (((car_x - points1[:, 0]) * p[:,0]) + (car_y - points1[:, 1]) * p[:,1]) / n

        u[u > 1] = 1
        u[u < 0] = 0

        # to facilitate dist calc
        x_sec = points1[:, 0] + u*np.transpose(p[:, 0]) - car_x
        y_sec = points1[:, 1] + u*np.transpose(p[:, 1]) - car_y
        dist_sqrd = x_sec**2 + y_sec**2

        current_point_index = np.argmin(dist_sqrd)
        #closest_info = (points[current_point_index:], self.trajectory.distances[current_point_index:])
        closest_info=current_point_index

        poii=self.visualizer(points[current_point_index][0],points[current_point_index][1])
        self.closestpub.publish(poii)
        
        return self.get_lookahead_point(closest_info, robot)
    
    def get_lookahead_point(self,robot):
        current_pos = np.asarray((robot.pose.pose.position.x, robot.pose.pose.position.y))  # Centre of circle
        look = self.lookahead  # Radius of circle

        
        #current_traj, current_dist = closest_info
        current_traj=np.asarray(self.trajectory.points)
        #current_dist=np.asarray(self.trajectory.distances)

        #if len(current_traj) <= 3:
        #if len(current_traj)-self.last_waypoint<=5:
        #    return (current_traj[-1], True)

        #if (abs(current_dist[c_i] - self.lookahead) < 1e-6):
        #    return (current_traj[c_i+1], False)
        
        for i in range(self.last_waypoint,len(current_traj)-1):
            P1 = current_traj[i]
            P2 = current_traj[i+1]

            segment_v=P2-P1
            vector_points= P2- current_pos

            a=np.dot(segment_v,segment_v)
            b=2*np.dot(vector_points,segment_v)
            c=np.dot(vector_points,vector_points)-look**2

            disc= b**2-4*a*c

            if disc>=0:

                t1=(-b + np.sqrt(disc))/(2*a)
                t2 = (-b - np.sqrt(disc))/(2*a)

                if (0<=t1<=1) and (i+t1>self.look_index):
                    self.look_index = i+t1
                    self.last_waypoint = i
                    self.last_goal= t1 *segment_v+P1
                    return (self.last_goal,False)

                elif (0<=t2<=1) and (i+t2>self.look_index):
                    self.look_index = i+t1
                    self.last_waypoint = i
                    self.last_goal= t1 *segment_v+P1
                    return (self.last_goal,False)

        return (self.last_goal,False)
            
                

        """
            #pt = current_traj[i][:]
            #squared_dist = np.dot(current_pos - pt, current_pos - pt)
            #if (squared_dist > self.lookahead ** 2):
            #    ind=i
            #    break
        
        P1 = closest_info[0][ind-1]  # last point inside circle
        P2 = closest_info[0][ind]
        segment_start = closest_info[0][ind-1][:]
        segment_end = closest_info[0][ind][:]
        
        P1=current_traj[ind-1]
        P2= current_traj[ind]
        segment_start = current_traj[ind-1][:]
        segment_end= current_traj[ind][:]
        
        vector_traj = segment_end - segment_start  # vectors along line segment
        vector_points = segment_start - current_pos  # vectors from robot to points

        # compute coefficients

        a = np.dot(vector_traj, vector_traj)
        b = 2 * np.dot(vector_traj, vector_points)
        c = np.dot(vector_points, vector_points) - lookahead_dist ** 2

        # find discriminant. if neg then line misses circle and no real soln
        disc = b ** 2 - 4 * a * c
        if disc < 0:
            #self.lookahead *= 2
            return (self.last_goal,False)

        # 2 intersection solns. if btw 0  and 1 the line doesnt hit the circle but would if extended
        sqrt_disc = math.sqrt(disc)
        t1 = (-b + sqrt_disc) / (2 * a)
        t2 = (-b - sqrt_disc) / (2 * a)
        if not (0 <= t1 <= 1 or 0 <= t2 <= 1):
            #self.lookahead *= 0.5
            return (self.last_goal,False)

        # get point on the line
        #if ind+t1 >self.look_index:
        #t = max(0, min(1, - b / (2 * a)))
        #return (P1 + t * (P2-P1), False)

        """
    def Pursuiter(self,pose_msg):

        # If trajectory isn't set, don't update drive messages.
        if not self.received_trajectory:
            return

        position = pose_msg.pose.pose.position
        goal_map ,end= self.get_lookahead_point(pose_msg) # Instantaneous goal - lookahead point

        # Transform from world coord to robot.
        quat = pose_msg.pose.pose.orientation
        euler = tf.transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        sint = np.sin(euler[2])
        cost = np.cos(euler[2])
        robot_transform = np.array([[cost, -sint, 0, position.x], [sint, cost, 0, position.y], [0,0,1,0], [0,0,0,1]])
        goal_transform = np.array([[1, 0, 0, goal_map[0]], [0, 1, 0, goal_map[1]], [0,0,1,0], [0,0,0,1]])

        goal_car = np.linalg.inv(np.matmul(np.linalg.inv(goal_transform), robot_transform))
        goal = np.array([goal_car[0, -1], goal_car[1,-1]])

        # Calculate the proper steering angle.
        R = (self.lookahead*self.lookahead)/(2*goal[1])# Radius of curvature connecting these points
        eta = np.arctan(self.wheelbase_length / R)

        # Create the drive message.
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "base_link"
        drive_msg.drive.steering_angle = eta

        # Check to see if we are close enough to the final destination to stop.
        pos = np.array([position.x, position.y])
        if np.linalg.norm(pos - self.last_goal) < 0.25:
            drive_msg.drive.speed = 0
        else:
            drive_msg.drive.speed = self.speed

        self.drive_pub.publish(drive_msg)

        # Update lookahead distance and velocity?
        #self.update_from_turn(eta)








if __name__ == "__main__":
    rospy.init_node("pure_pursuit")
    pf = PurePursuit()
    rospy.spin()
