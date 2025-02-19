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
        self.last_start_point = None
        self.start_point = None
        self.last_end_point = None
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
            self.map = skimage.morphology.dilation(self.map, skimage.morphology.disk(15))
            self.map[self.map == 1] = 100
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

            # Debug Code
            print("real:", (0.0, 0.0, 1.0))
            print("px:", self.real_to_pixel((0.0, 0.0, 0.0), self.map_msg))
            print("real:", self.pixel_to_real(self.real_to_pixel((0.0, 0.0, 0.0), self.map_msg), self.map_msg))

            # Set map flag
            self.map_acquired = True

            self.plan_path(self.start_point, self.end_point, self.map)


    def odom_cb(self, msg):
        # Initialize starting position
        x, y = msg.pose.pose.position.x, msg.pose.pose.position.y
        quat = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        theta = tf.transformations.euler_from_quaternion(quat)[2]
        self.start_point = (x, y, theta)
        # print("START POINT INITIALIZED")
	#self.start_point=(8.74579,13.22515,0.88023)
        if self.last_start_point == None:
            print(self.start_point)
            self.plan_path(self.start_point, self.end_point, self.map)
        
        self.last_start_point = self.start_point


    def goal_cb(self, msg):
        # Initialize ending position
        x, y = msg.pose.position.x, msg.pose.position.y
        quat = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
        theta = tf.transformations.euler_from_quaternion(quat)[2]
        self.end_point = (x, y, theta)
        print("END POINT INITIALIZED")
        #if self.last_end_point != self.end_point:
            #  print("hi")
        self.plan_path(self.start_point, self.end_point, self.map)
        
        self.last_end_point = self.end_point

    # Helper function: convert pixel to real coordinates
    def pixel_to_real(self, px, map):
        # u, v, theta = px[0], px[1], 0.0
        # u, v = u*map.info.resolution, v*map.info.resolution
        # pixel_pose = np.array([[np.cos(theta), -np.sin(theta), 0.0, u], [np.sin(theta), np.cos(theta), 0.0, v], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]])
        # quat = [map.info.origin.orientation.x, map.info.origin.orientation.y, map.info.origin.orientation.z, map.info.origin.orientation.w]
        # theta_t = tf.transformations.euler_from_quaternion(quat)[2]
        # x_t, y_t, z_t = map.info.origin.position.x,map.info.origin.position.y, map.info.origin.position.z 
        # transform = np.array([[np.cos(theta_t), -np.sin(theta_t), 0.0, x_t], [np.sin(theta_t), np.cos(theta_t), 0.0, y_t], [0.0, 0.0, 1.0, z_t], [0.0, 0.0, 0.0, 1.0]])
        # real_pose = np.matmul(pixel_pose, transform)

        pixel_pose = np.array([px[0]*map.info.resolution, px[1]*map.info.resolution, 0.0])
        quat = [map.info.origin.orientation.x, map.info.origin.orientation.y, map.info.origin.orientation.z, map.info.origin.orientation.w]
        map_theta = tf.transformations.euler_from_quaternion(quat)[2]
        map_rotation = np.array([[np.cos(map_theta), -np.sin(map_theta), 0.0], [np.sin(map_theta), np.cos(map_theta), 0.0], [0.0, 0.0, 1.0]])
        translation = np.array([map.info.origin.position.x, map.info.origin.position.y, 0.0])
        real_pose = np.matmul(pixel_pose, map_rotation) + translation

        return (real_pose[0], real_pose[1])


    # Helper function: convert real to pixel coordinates
    def real_to_pixel(self, real, map):
        # x, y, theta = real[0], real[1], 0.0
        # real_pose = np.array([[np.cos(theta), -np.sin(theta), 0.0, x], [np.sin(theta), np.cos(theta), 0.0, y], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]])
        # quat = [map.info.origin.orientation.x, map.info.origin.orientation.y, map.info.origin.orientation.z, map.info.origin.orientation.w]
        # theta_t = tf.transformations.euler_from_quaternion(quat)[2]
        # x_t, y_t, z_t = map.info.origin.position.x,map.info.origin.position.y, map.info.origin.position.z 
        # transform = np.array([[np.cos(theta_t), -np.sin(theta_t), 0.0, x_t], [np.sin(theta_t), np.cos(theta_t), 0.0, y_t], [0.0, 0.0, 1.0, z_t], [0.0, 0.0, 0.0, 1.0]])
        # pixel_pose = np.matmul(real_pose, np.linalg.inv(transform))

        real_pose = np.array([real[0], real[1], 0.0])
        quat = [map.info.origin.orientation.x, map.info.origin.orientation.y, map.info.origin.orientation.z, map.info.origin.orientation.w]
        map_theta = tf.transformations.euler_from_quaternion(quat)[2]
        map_rotation = np.array([[np.cos(map_theta), -np.sin(map_theta), 0.0], [np.sin(map_theta), np.cos(map_theta), 0.0], [0.0, 0.0, 1.0]])
        translation = np.array([map.info.origin.position.x, map.info.origin.position.y, 0.0])
        pixel_pose = np.matmul(real_pose-translation, np.linalg.inv(map_rotation))

        return (np.round(pixel_pose[0]/map.info.resolution).astype(int), np.round(pixel_pose[1]/map.info.resolution).astype(int))

    def distance(self, current_point, end_point):
        return ((end_point[0] - current_point[0])**2 + (end_point[1] - current_point[1])**2)**(1/2)

    def plan_path(self, start_point, end_point, map):
        #start_point=(8.74579,13.22515,0.88023)
        print(start_point)
        print(end_point)
        print(self.map_acquired)
        if start_point != None and end_point != None and self.map_acquired:
            # Create path
            print("in path plan")
            # A*
            start_coordinate = self.real_to_pixel(start_point, self.map_msg)
            end_coordinate = self.real_to_pixel(end_point, self.map_msg)
            print(start_coordinate, end_coordinate)
            queue = [start_coordinate]
            queue_priorities = np.array([0.0])
            costs = {start_coordinate: 0.0}
            parents = {}
            while len(queue) > 0:
                # print(queue[0])
                curr = queue.pop(0)
                queue_priorities = queue_priorities[1:]

                if curr == end_coordinate:
                    parents[curr]
                    break

                for adj in self.map_graph[curr]:
                    cost = costs[curr] + 1.0
                    if adj not in costs or cost < costs[adj]:
                        costs[adj] = cost
                        priority = cost + self.distance(adj, end_coordinate)
                        index = np.searchsorted(queue_priorities, priority)
                        # print("queue:", queue)
                        # print("queue priorities:", queue_priorities)
                        # print("current:", curr)
                        # print(index)
                        queue.insert(index, adj)
                        queue_priorities = np.insert(queue_priorities, index, np.array(priority))
                        # print("queue:", queue)
                        # print("queue priorities:", queue_priorities)
                        parents[adj] = curr
                        if adj == end_coordinate:
                            print("REACHED END")
            
            # Extract path
            path = [end_coordinate]
            curr = end_coordinate
            while curr != start_coordinate:
                curr = parents[curr]
                path.append(curr)
            path.reverse()

            # ~~~~~~~~~~~~~~~~~~~ START BFS CODE ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            # start_coordinate = self.real_to_pixel(start_point, self.map_msg)
            # end_coordinate = self.real_to_pixel(end_point, self.map_msg)
            # print(start_coordinate, end_coordinate)
            # queue = [start_coordinate]
            # visited = set()
            # parents = {}
            # end_found = False
            # # for tup in set(self.map_graph.keys()):
            # #     if abs(tup[0] - end_coordinate[0]) < 10 and abs(tup[1] - end_coordinate[1]) < 10:
            # #         print(tup)
            # while (not end_found) and len(queue) > 0:
            #     curr = queue.pop(0)
            #     visited.add(curr)
            #     for adj in self.map_graph[curr]:
            #         if adj == end_coordinate:
            #             end_found = True
            #             parents[adj] = curr
            #             break
            #         if adj not in visited:
            #             queue.append(adj)
            #             visited.add(adj)
            #             parents[adj] = curr
            
            # # Extract path
            # path = [end_coordinate]
            # curr = end_coordinate
            # while curr != start_coordinate:
            #     curr = parents[curr]
            #     path.append(curr)
            # path.reverse()
            # ~~~~~~~~~~~~~~~~~~~ END BFS CODE ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

            # Convert to real-world frame
            real_path = [self.pixel_to_real(px, self.map_msg) for px in path]

            # Convert to trajectory
            self.trajectory.clear()
            for p in real_path:
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
