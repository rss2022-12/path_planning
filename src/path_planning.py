#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, PoseArray, Point
from nav_msgs.msg import Odometry, OccupancyGrid
import rospkg
import time, os
from utils import LineTrajectory

# Custom imports
import skimage.morphology
import tf
import matplotlib.pyplot as plt

class PathPlan(object):
    """ Listens for goal pose published by RViz and uses it to plan a path from
    current car pose.
    """
    def __init__(self):
        self.odom_topic = rospy.get_param("~odom_topic")
        self.map_sub = rospy.Subscriber("/map", OccupancyGrid, self.map_cb)
        self.trajectory = LineTrajectory("/planned_trajectory")
        self.goal_sub = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goal_cb, queue_size=10)
        self.traj_pub = rospy.Publisher("/trajectory/current", PoseArray, queue_size=10)
        self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self.odom_cb)

        # Instance variables
        self.map_acquired = False
        self.map_msg = None
        self.map = None
        self.map_graph = None
        self.start_point = None
        self.end_point = None


    def map_cb(self, msg):
        # Initialize map
        if not self.map_acquired:
            self.map_msg = msg
            self.map = np.array(msg.data)
            self.map = np.reshape(self.map, (msg.info.height, msg.info.width))
            self.map[self.map == -1] = 100 # Convert unknowns to occupied
            # map_image = plt.imshow(self.map)
            # plt.show()
            self.map[self.map == 100] = 1 # Convert to binary for skimage
            self.map = skimage.morphology.dilation(self.map, skimage.morphology.disk(10))
            self.map[self.map == 1] = 100
            self.map_acquired = True
            # map_image = plt.imshow(self.map)
            # plt.show()
            print("INITIALIZED MAP")

            self.map_graph = {}
            for v in range(self.map.shape[0]):
                for u in range(self.map.shape[1]):
                    if self.map[v, u] == 0:
                        for i in [-1, 0, 1]:
                            for j in [-1, 0, 1]:
                                if abs(i) != abs(j) and self.map[v+i, u+j] == 0:
                                    if (u, v) not in self.map_graph:
                                        self.map_graph[(u, v)] = set()
                                    self.map_graph[(u, v)].add((u+j, v+i))
            print("INITIALIZED MAP GRAPH")

            self.plan_path(self.start_point, self.end_point, self.map)


    def odom_cb(self, msg):
        # Initialize starting position
        if not self.start_point:
            x, y = msg.pose.pose.position.x, msg.pose.pose.position.y
            quat = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
            theta = tf.transformations.euler_from_quaternion(quat)[2]
            self.start_point = (x, y, theta)
            print("START POINT INITIALIZED")

            self.plan_path(self.start_point, self.end_point, self.map)


    def goal_cb(self, msg):
        # Initialize ending position
        x, y = msg.pose.position.x, msg.pose.position.y
        quat = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
        theta = tf.transformations.euler_from_quaternion(quat)[2]
        self.end_point = (x, y, theta)
        print("END POINT INITIALIZED")

        self.plan_path(self.start_point, self.end_point, self.map)

    # Helper function: convert pixel to real coordinates
    def pixel_to_real(self, px, map):
        u, v = px
        u, v = u*map.info.resolution, v*map.info.resolution
        raw_pose = np.array([[np.sin(0.0), np.cos(0.0), 0.0, u]])
        raise NotImplementedError


    # Helper function: convert real to pixel coordinates
    def real_to_pixel(self, real, map):
        x, y, theta = real[0], real[1], 0.0
        real_pose = np.array([[np.cos(theta), -np.sin(theta), 0.0, x], [np.sin(theta), np.cos(theta), 0.0, y], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]])
        quat = [map.info.origin.orientation.x, map.info.origin.orientation.y, map.info.origin.orientation.z, map.info.origin.orientation.w]
        theta_t = tf.transformations.euler_from_quaternion(quat)[2]
        x_t, y_t, z_t = map.info.origin.position.x,map.info.origin.position.y, map.info.origin.position.z 
        transform = np.array([[np.cos(theta_t), -np.sin(theta_t), 0.0, x_t], [np.sin(theta_t), np.cos(theta_t), 0.0, y_t], [0.0, 0.0, 1.0, z_t], [0.0, 0.0, 0.0, 1.0]])
        pixel_pose = np.matmul(real_pose, np.linalg.inv(transform))

        return (np.round(pixel_pose[0, 3]/map.info.resolution).astype(int), np.round(pixel_pose[1, 3]/map.info.resolution).astype(int))


    def plan_path(self, start_point, end_point, map):
        if start_point != None and end_point != None and self.map_acquired:
            # Create path

            # BFS
            start_coordinate = self.real_to_pixel(start_point, self.map_msg)
            end_coordinate = self.real_to_pixel(end_point, self.map_msg)
            print(start_coordinate, end_coordinate)
            queue = [start_coordinate]
            visited = set()
            parents = {}
            end_found = False
            for tup in set(self.map_graph.keys()):
                if abs(tup[0] - end_coordinate[0]) < 10 and abs(tup[1] - end_coordinate[1]) < 10:
                    print(tup)
            while (not end_found) and len(queue) > 0:
                curr = queue.pop(0)
                visited.add(curr)
                for adj in self.map_graph[curr]:
                    if adj == end_coordinate:
                        end_found = True
                        parents[adj] = curr
                        break
                    if adj not in visited:
                        queue.append(adj)
                        visited.add(adj)
                        parents[adj] = curr
            
            # Extract path
            path = [end_coordinate]
            curr = end_coordinate
            while curr != start_coordinate:
                curr = parents[curr]
                path.append(curr)
            path.reverse()
            print("PATH:", path)

            # Convert to trajectory
            self.trajectory.clear()
            for p in path:
                point = Point(p[0], p[1], 0)
                self.trajectory.addPoint(point)

            # publish trajectory
            self.traj_pub.publish(self.trajectory.toPoseArray())

            # visualize trajectory Markers
            self.trajectory.publish_viz()


if __name__=="__main__":
    rospy.init_node("path_planning")
    pf = PathPlan()
    rospy.spin()
