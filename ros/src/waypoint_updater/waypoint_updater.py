#!/usr/bin/env python

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32
from scipy.spatial import KDTree

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 50 # Number of waypoints we will publish. You can change this number
MAX_DEACC = 0.5
RATE_PUB=50

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.waypoints_tree = None
        self.base_waypoints = None
        self.waypoints_2D = None
        self.stopline_wp_idx = -1
        self.pose = None
        self.waypoint_generator()

        
    def waypoint_generator(self):
        rate = rospy.Rate(RATE_PUB)
        while not rospy.is_shutdown():
            if self.pose and self.base_waypoints:
                #Get closest waypoint
                nearest_index = self.get_closest_waypoint_id()
                self.publish_wp(nearest_index)
            rate.sleep()
	
    def get_closest_waypoint_id(self):
        x_pos= self.pose.pose.position.x
        y_pos = self.pose.pose.position.y
        closest_index = self.waypoints_tree.query([x_pos, y_pos], 1)[1]

        #Checking if closest is ahead or behind vehicle
        closest_coord = self.waypoints_2D[closest_index]
        prev_coord = self.waypoints_2D[closest_index - 1]

        #Getting the equation for hyperplane
        cl_vect = np.array(closest_coord)
        prev_vect = np.array(prev_coord)
        pos_vect = np.array([x_pos, y_pos])

        #Using dot product to see where point lies
        val = np.dot(cl_vect - prev_vect, pos_vect - cl_vect)

        if val > 0:
            closest_index = (closest_index - 1) % len(self.waypoints_2D)
        rospy.logdebug("Closest Index %d", closest_index)
        return closest_index

    # Publish the waypoints that the car has to follow  
    def publish_wp(self,closest_idx):
        generate_lane = self.get_lane()
        self.final_waypoints_pub.publish(generate_lane)
    # This  
    def get_lane(self):
      lane = Lane()
      closest_index = self.get_closest_waypoint_id()
      farthest_index = closest_index + LOOKAHEAD_WPS
      base_waypoints = self.base_waypoints.waypoints[closest_index:closest_index+LOOKAHEAD_WPS]
      if self.stopline_wp_idx == -1 or (self.stopline_wp_idx >=farthest_index): 
        lane.waypoints = base_waypoints
      else:
        lane.waypoints = self.decelerate_waypoints(base_waypoints,closest_index)	
      return lane
    
    
    def decelerate_waypoints(self,waypoints,closest_idx):
      updated_wp_vel = []
      for i,wp in enumerate(waypoints):
        p = Waypoint()
        p.pose = wp.pose
        stop_idx = max(self.stopline_wp_idx - closest_idx - 2, 0)
        dist = self.distance(waypoints,i,stop_idx)
        vel = math.sqrt(2* MAX_DECEL *dist)
        if(vel<1.):
          vel = 0.
          p.twist.twist.linear.x = min(vel,wp.twist.twist.linear.x)
          updated_wp_vel.append(p)
      return updated_wp_vel
       
    def pose_cb(self, msg):
        # TODO: Implement
        self.pose = msg

    def waypoints_cb(self, waypoints):
        # TODO: Implement
        self.base_waypoints = waypoints
        if not self.waypoints_2D:
            self.waypoints_2D = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]
            #Creating a K-Dimension Binary search tree
            self.waypoints_tree = KDTree(self.waypoints_2D)

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        self.stopline_wp_idx = msg.data


    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
