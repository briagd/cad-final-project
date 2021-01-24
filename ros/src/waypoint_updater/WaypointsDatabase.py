import numpy as np
from scipy.spatial import KDTree
import styx_msgs
import sys
import rospy

class WaypointsDatabase:
    """This class can be used to query the closest waypoint to a given (x,y) point"""
    def __init__(self, waypoints):
        self.waypoints = waypoints
        self.waypoints_as_lists = []
        if not self.waypoints_as_lists:
            self.waypoints_as_lists = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in waypoints]
            self.waypoint_tree = KDTree(self.waypoints_as_lists)
            
        
    def get_next_closest_idx(self, pose):
        # Find the closest waypoints to pose *that comes after pose on the track*
        x_car = pose[0]
        y_car = pose[1]

        closest_wpt_idx = self.waypoint_tree.query([x_car,y_car], 1)[1]
        
        # Retrieve [x, y] of closest waypoint
        closest_wypt = self.waypoints_as_lists[closest_wpt_idx]
        # Retrieve [x, y] of prior of the closest waypoint
        wypt_prior_closest = self.waypoints_as_lists[closest_wpt_idx - 1]
        
        # convert position arrays to np array
        closest_vect = np.array(closest_wypt) 
        prior_to_closest_vect = np.array(wypt_prior_closest)
        car_pos_vect = np.array([x_car,y_car])

        # Dot product to check if closest is ahead of the car
        val = np.dot(closest_vect -  prior_to_closest_vect, car_pos_vect - closest_vect)
        # if closest is behind
        if val > 0:
            closest_wpt_idx = (closest_wpt_idx + 1) % len(self.waypoints_as_lists)

        # Compute the distance between the car and the closest waypoint
        dist = np.sqrt((pose[0]-self.waypoints_as_lists[closest_wpt_idx][0])**2 + \
                           (pose[1]-self.waypoints_as_lists[closest_wpt_idx][1])**2)

        return dist, closest_wpt_idx