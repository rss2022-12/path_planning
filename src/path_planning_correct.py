#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, PoseArray, Point, Pose
from nav_msgs.msg import Odometry, OccupancyGrid
import rospkg
import time, os
from utils import LineTrajectory
from visualization_msgs.msg import MarkerArray, Marker

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
        self.rrtpath = rospy.Publisher("/visualization_msgs",Marker,queue_size=20)
        self.endpoint=rospy.Publisher("visualization_msgs1",Marker,queue_size=20)

        #flags and things
        
        self.maxerror=100 #maxerror between final node and goal for rrt
        self.threshold_error=2
        self.algorithm="RRT" #or BFS\
        self.RRTdone=False

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
            self.map = skimage.morphology.dilation(self.map, skimage.morphology.disk(10))
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

            if self.algorithm=="RRT":
                self.rrt_algorithm(self.start_point,self.end_point)
            else:
                self.plan_path(self.start_point, self.end_point, self.map)


    def odom_cb(self, msg):
        # Initialize starting position
        x, y = msg.pose.pose.position.x, msg.pose.pose.position.y
        quat = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        theta = tf.transformations.euler_from_quaternion(quat)[2]
        self.start_point = (x, y, theta)
        # print("START POINT INITIALIZED")

        if self.start_point != self.last_start_point:
            
            if self.algorithm=="RRT":
                self.rrt_algorithm(self.start_point,self.end_point)
            else:
                self.plan_path(self.start_point, self.end_point, self.map)

        
        self.last_start_point = self.start_point
        
        if self.algorithm=="RRT":
            self.rrt_algorithm(self.start_point,self.end_point)
        else:
            self.plan_path(self.start_point, self.end_point, self.map)

        



    def goal_cb(self, msg):
        # Initialize ending position
        x, y = msg.pose.position.x, msg.pose.position.y
        quat = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
        theta = tf.transformations.euler_from_quaternion(quat)[2]
        self.end_point = (x, y, theta)
        print("END POINT INITIALIZED")

        if self.last_end_point != self.end_point:
            if self.algorithm=="RRT":
                self.RRTdone=False
                self.rrt_algorithm(self.start_point,self.end_point)
            else:
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


    def plan_path(self, start_point, end_point, map):
        return
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
            # for tup in set(self.map_graph.keys()):
            #     if abs(tup[0] - end_coordinate[0]) < 10 and abs(tup[1] - end_coordinate[1]) < 10:
            #         print(tup)
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
            
    def rrt_algorithm(self, start_point, end_point):
         if start_point != None and end_point != None and self.map_acquired and self.RRTdone==False:
            marker_end=Marker()
            marker_end.header.frame_id = "/map"
            marker_end.header.stamp=rospy.Time.now()
            # # marker_pt.id=0
            marker_end.pose.position.x = end_point[0]
            marker_end.pose.position.y = end_point[1]
            marker_end.pose.position.z=0
            marker_end.scale.x = 0.2
            marker_end.scale.y = 0.2
            marker_end.scale.z = 0.2
            marker_end.color.a = 1.0 # Don't forget to set  the alpha!
            marker_end.color.r = 1.0
            marker_end.color.g = 0.0
            marker_end.color.b = 0.0
            marker_end.type=2
            self.endpoint.publish(marker_end)
            # Create path
            # self.leaf_array=0
         #change to coordinates
            start_point = self.real_to_pixel(start_point, self.map_msg)
            end_point = self.real_to_pixel(end_point, self.map_msg)
            leaf2parent={start_point:-1}
     

            marker_pt=Marker()
            marker_pt.header.frame_id = "/map"
            marker_pt.header.stamp=rospy.Time.now()
            marker_pt.scale.x = 0.2
            marker_pt.scale.y = 0.2
            marker_pt.scale.z = 0.2
            marker_pt.color.a = 1.0 # Don't forget to set  the alpha!
            marker_pt.color.r = 0.0
            marker_pt.color.g = 1.0
            marker_pt.color.b = 0.0
            marker_pt.type=marker_pt.SPHERE_LIST
            marker_pt.action=marker_pt.ADD
        
            points=[]
            
            
            all_errs=np.array([1000])
            while True:
                # print(leaf2parent)
                random_point=self.random_sample(start_point,end_point)
                # print(random_point)
                nearest=self.find_nearest(random_point,leaf2parent.keys())
                if nearest == -1:
                    continue
            
                new_leaf=self.steering(random_point,nearest)
              
              
                new_leaf_real=self.pixel_to_real(new_leaf,self.map_msg)
                pt=Point()
                pt.x=new_leaf_real[0]
                pt.y=new_leaf_real[1]
                pt.z=0
                points.append(pt)
                
              
                marker_pt.points=points   
                self.rrtpath.publish(marker_pt)
            
              
                # print(new_leaf)
                if self.check_obstacle(nearest,new_leaf):
                    leaf2parent[(new_leaf[0],new_leaf[1])]=nearest
                    err=np.sqrt((new_leaf[0]-end_point[0])**2+(new_leaf[1]-end_point[1])**2)
                    # print("newlef:",new_leaf,end_point,err)
                    np.append(all_errs,err)
                    print(np.min(all_errs))
                    if err < self.maxerror:
                        print(leaf2parent)
                        # print(err)
                        self.RRT_path(leaf2parent,new_leaf,start_point,end_point)
                        self.RRTdone=True
                        break
            
    def random_sample(self,start_point,end_point):
        if np.random.randint(20)==7:
            return end_point
        else:
            
            random_point=np.random.randint(len(self.map_graph))
            random_point=self.map_graph.keys()[random_point]
           
            while random_point[0]==start_point[0] and random_point[1]==start_point[1]:
                 random_point=np.random.randint(len(self.map_graph))
                 random_point=self.map_graph.keys()[random_point]
            # right_random_point=(random_point[0],-random_point[1])     
            return random_point
        
    def find_nearest(self,random_point,leaf2parent):
        # print(random_point)
        # print(leaf2parent)
       
        error=np.sum((np.asarray(leaf2parent) - random_point)**2, axis=1)
        
        min_error_ind=np.argmin(error)

        
        if error[min_error_ind] > self.threshold_error:
            # print(leaf2parent[min_error_ind])
            return leaf2parent[min_error_ind]
        else:
            return -1
        
    def steering(self,random_point, nearest):
        
        err=np.sqrt((nearest[0]-random_point[0])**2+(nearest[1]-random_point[1])**2)
        if err < 200:
            return random_point
        direction = np.array([random_point[0]-nearest[0]/(err),random_point[1]-nearest[1]/err])  # normalization
        new_point = np.round(nearest + 100*direction)
        # print("sttering:",nearest,random_point,direction,new_point)
        return (new_point[0],new_point[1])
    
    def check_obstacle(self,nearest,new_leaf):
        
        if nearest[0]<new_leaf[0]:
            xmin=nearest[0]
            xmax=new_leaf[0]
        else:
            xmin=new_leaf[0]
            xmax=nearest[0]
            
        if nearest[1]<new_leaf[1]:
            ymin=nearest[1]
            ymax=new_leaf[1]
        else:
            ymin=new_leaf[1]
            ymax=nearest[1]
            
        xmin = (np.round(xmin)).astype(int)
        xmax = (np.round(xmax)).astype(int)+1
        ymin = (np.round(ymin)).astype(int)
        ymax = (np.round(ymax)).astype(int)+1
        
        # rectanglex=np.arange(xmin,xmax)
        # rectangley=np.arange(ymin,ymax)
        # meshed=np.meshgrid(rectanglex,rectangley)
        # print(meshed)
        # flattened=np.reshape(meshed,len(meshed)**2)
        rectangle = self.map[ymin:ymax,xmin:xmax]
        # print(rectangle)
        
        if np.max(rectangle) >3:  # ocupied or undefined
            return False
        else:
            return True

        
        
        # non_occupied=np.array(self.map_graph.keys())
        # print(non_occupied)
        # for x in np.arange(xmin,xmax):
        #     for y in np.arange(ymin,ymax):
        #         print(self.map[y,x])
        #         if self.map[y,x] !=0:
        #             print("hi")
        #             return False
                
        
        # return True
        
    
    def RRT_path(self,tree,final_leaf,start_leaf,endpoint):
        
        path=[endpoint,final_leaf]
        # leafend=0
        while final_leaf!=-1:
            leafend=tree[final_leaf]
            if leafend!=-1:
                path.append(leafend)
            
                # print("leafend:",leafend)
                final_leaf=tree[leafend]
            else:
                break
        path.append(start_leaf)
 
        path.reverse()
        print("path:",path)
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
