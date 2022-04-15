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
        self.max_speed=4
        #self.Klook = 0.2  # constant that affects lookahead
        self.received_trajectory = False
        # based on speed
        self.lookahead = 0.7
        self.max_look= 3.5
        self.wheelbase_length = .3
        self.trajectory = utils.LineTrajectory("/followed_trajectory")
        self.traj_sub = rospy.Subscriber("/trajectory/current", PoseArray, self.trajectory_callback, queue_size=1)
        self.drive_pub = rospy.Publisher("/drive", AckermannDriveStamped, queue_size=1)
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.Pursuiter, queue_size=1)

        self.look_index=0
        self.last_goal=np.array([0,0])

        self.closestpub=rospy.Publisher("/closest",Marker,queue_size=1)
        self.look_circle=rospy.Publisher("/look_circle",Marker,queue_size=1)
        self.goal_pt=rospy.Publisher("/goal_pt",Marker,queue_size=1)
        #self.goal_vec=rospy.Publisher("/goal_vec",Marker,queue_size=1);
        
        
    def trajectory_callback(self, msg):
        ''' Clears the currently followed trajectory, and loads the new one from the message
        '''
        print("Receiving new trajectory:", len(msg.poses), "points")
        self.trajectory.clear()
        self.trajectory.fromPoseArray(msg)
        self.trajectory.publish_viz(duration=0.0)
        self.last_goal=np.asarray(self.trajectory.points[0])
        self.look_index=0
        self.received_trajectory = True


    def visualizer(self, pos_x, pos_y, color=[0,1,0], shape=2,or_x=0,or_y=0,or_z=0,or_w=0):
        poii=Marker()
        poii.header.frame_id="map"
        poii.pose.position.x=pos_x
        poii.pose.position.y=pos_y
        poii.pose.position.z=0
        poii.pose.orientation.x=or_x
        poii.pose.orientation.y=or_y
        poii.pose.orientation.z=or_z
        poii.pose.orientation.w=or_w
        poii.color.a=1
        poii.color.r=color[0]
        poii.color.g=color[1]
        poii.color.g=color[2]
        poii.type=shape
        poii.scale.x=1
        poii.scale.y=1
        poii.scale.z=1
        return poii
        
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
    
    def get_lookahead_point(self,c_i,robot):
        current_pos = np.asarray((robot.pose.pose.position.x, robot.pose.pose.position.y))  # Centre of circle
        look = self.lookahead  # Radius of circle

        

        current_traj=np.asarray(self.trajectory.points)
        current_dist=np.asarray(self.trajectory.distances)


        if (abs(current_dist[c_i] - self.lookahead) < 1e-6):
            self.last_goal=np.asarray(current_traj[c_i+1])
            return self.last_goal

        for i in range(c_i,len(current_traj)-1):
            
            P1 = current_traj[i]
            P2 = current_traj[i+1]

            segment_v=P2-P1
            vector_points= P1- current_pos

            a=np.dot(segment_v,segment_v)
            b=2*np.dot(vector_points,segment_v)
            c=np.dot(vector_points,vector_points)-look**2

            disc= b*b-4*a*c

            if disc>=0:

                t1=(-b + np.sqrt(disc))/(2*a)
                t2 = (-b - np.sqrt(disc))/(2*a)

                if (0<=t1<=1) and (i+t1>=self.look_index):
                    self.look_index = i+t1
                    self.last_goal= t1 *segment_v+P1
                    return self.last_goal

                elif (0<=t2<=1) and (i+t2>=self.look_index):
                    self.look_index = i+t2
                    self.last_goal= t2 *segment_v+P1
                    return self.last_goal

        return self.last_goal
            
                

        
    def Pursuiter(self,msg):

        if self.received_trajectory:
            """

            """
            goal= self.find_closest_point(msg)

            position= msg.pose.pose.position
            orientation=msg.pose.pose.orientation
            current_pos= np.asarray([position.x,position.y])


            lookahead_circle=self.visualizer(current_pos[0],current_pos[1],color=[1,0,0],shape=3)
            lookahead_circle.scale.x=self.lookahead
            lookahead_circle.scale.y=self.lookahead
            self.look_circle.publish(lookahead_circle)

            goaler=self.visualizer(goal[0],goal[1],color=[0,0,1],shape=2)
            self.goal_pt.publish(goaler)
            goal_vec= goal-current_pos



            current_rot=tf.transformations.euler_from_quaternion([orientation.x,orientation.y,orientation.z,orientation.w])[2]

            rot_matrix= np.array([[np.cos(current_rot),-np.sin(current_rot)], [np.sin(current_rot), np.cos(current_rot)]])

            rot_point= np.dot(np.linalg.inv(rot_matrix), goal_vec.T)

            L1 = np.linalg.norm(rot_point)
                
            alpha= np.arctan2(rot_point[1],rot_point[0])

            steer_angle= np.arctan2((2*self.wheelbase_length * np.sin(alpha)),L1)

                #new_vel=3.06*(-steer_angle+1.4)*(steer_angle+1.4) # the 3.06 is the speed constant (assuming 6 m/s max speed), and the 1.4 is max turning angle (assuming 1.4)
            abs_steer=min(abs(steer_angle),0.5)

                #print("angle is : ",steer_angle)
                #print("goal is :" , goal)
            self.speed= (1-abs_steer)*self.max_speed
            self.lookahead=(1-abs_steer)*self.max_look
            ack=AckermannDriveStamped()
            ack.drive.steering_angle=steer_angle
            ack.drive.speed=self.speed #new_vel

            if goal.all()==np.asarray(self.trajectory.points[-1]).all() and L1<1.0:
                ack.drive.speed=0.0
                #if L1 < 1.0 and reached_end:
                #    ack.drive.speed = 0.0
            self.drive_pub.publish(ack)









if __name__ == "__main__":
    rospy.init_node("pure_pursuit")
    pf = PurePursuit()
    rospy.spin()
