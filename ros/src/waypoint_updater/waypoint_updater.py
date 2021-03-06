#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32
from scipy.spatial import KDTree

import math

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

LOOKAHEAD_WPS = 75 # Number of waypoints we will publish. You can change this number
MAX_DECEL = 0.5 


class WaypointUpdater(object):
    def __init__(self):
        
        rospy.init_node('waypoint_updater')
        # Add other member variables you need below
        self.pose = None
        self.base_lane = None
        self.waypoints_2d = None
        self.waypoints_tree = None
        self.stopline_wp_index = -1


        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        #rospy.Subscriber('obstacle_waypoint', Lane, self.obstacle_cb, queue_size=1)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        #rospy.spin()
        #Writing a custom publish loop instead of using rospy.spin loop
        self.publish_loop()

    def publish_loop(self):
            rate = rospy.Rate(50)
            while not rospy.is_shutdown():
                if self.pose and self.base_lane:
                    #Get Closest waypoint
                    #closest_waypoint_idx = self.get_closest_waypoint_idx()
                    self.publish_wp()
                rate.sleep()

    def get_next_waypoint(self):
        x = self.pose.pose.position.x
        y = self.pose.pose.position.y

        closest_idx = self.waypoints_tree.query([x, y], 1)[1]

        #Check if the closest is ahead or behind the vehicle
        closest_coord = self.waypoints_2d[closest_idx]
        prev_coord = self.waypoints_2d[closest_idx-1]

        #Equation for hyperplane through closest coordinates
        cl_vect = np.array(closest_coord)
        prev_vect = np.array(prev_coord)
        pos_vect = np.array([x, y])

        val = np.dot(cl_vect - prev_vect, pos_vect - cl_vect)

        if val > 0:
             closest_idx = (closest_idx + 1) % len(self.waypoints_2d)

        return closest_idx

    def publish_wp(self):
        final_lane = self.chosen_lane()

        """ 
        #Debug
        for i in range(len(final_lane.waypoints)):
            rospy.logwarn("Next Waypoint= x=%d, y=%d",
                          final_lane.waypoints[i].pose.pose.position.x,
                          final_lane.waypoints[i].pose.pose.position.y)
        """ 
        self.final_waypoints_pub.publish(final_lane)

    def chosen_lane(self):
        lane = Lane()
        lane.header.frame_id = '/world'
        lane.header.stamp = rospy.Time(0)

        closest_idx = self.get_next_waypoint()
        farthest_idx = closest_idx + LOOKAHEAD_WPS


        base_waypoints = self.base_lane.waypoints[closest_idx:farthest_idx] 

        if self.stopline_wp_index == -1 or (self.stopline_wp_index >= farthest_idx):
            #rospy.logwarn("normal as stopline is %s and farthest is %s", self.stopline_wp_index, farthest_idx)
            #rospy.logwarn("GREEN or UNKNOWN")
            lane.waypoints = base_waypoints
        else:
            #rospy.logwarn("publish decelerate as stopline is %s and farthest is %s", self.stopline_wp_index, farthest_idx)
            #rospy.logwarn("RED OR YELLOW Ready to stop")
            lane.waypoints = self.decelerate_waypoints(base_waypoints, closest_idx)

        #Debug comment once done
        """
         
        rospy.logerr("closest waypoint waypoint=%d x=%d y=%d",
                       closest_idx,
                       self.base_lane.waypoints[closest_idx].pose.pose.position.x,
                       self.base_lane.waypoints[closest_idx].pose.pose.position.y)
        rospy.logerr("Curr pos = x= %d,  y=%d",
                      self.base_pose.position.x,
                      self.pose.position.y)
         
        """ 
        return lane

    def decelerate_waypoints(self, waypoints, closest_idx):
        temp = []

        for i, wp in enumerate(waypoints):
            p = Waypoint()
            p.pose = wp.pose

            # stop disance is -2  so that car stop at the line
            stop_idx = max(self.stopline_wp_index - closest_idx -2, 0)

            #i = min(i, stop_idx)
            dist = self.distance(waypoints, i,  stop_idx)

            vel = math.sqrt(2 * MAX_DECEL * dist)

            if vel < 1.:
                vel = 0.

            p.twist.twist.linear.x = min(vel, wp.twist.twist.linear.x)
            temp.append(p)

        return temp

    def pose_cb(self, msg):
        # Implement
        # Callback for update on position
        self.pose = msg
        '''
        if self.base_lane is not None:
            self.publish()
        '''
    def waypoints_cb(self, waypoints):
        # TODO: Implement
        # Callback for updating list of waypoints
        self.base_lane = waypoints

        if not self.waypoints_2d:
            self.waypoints_2d = [[ waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]
            self.waypoints_tree = KDTree(self.waypoints_2d)

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        self.stopline_wp_index = msg.data

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
