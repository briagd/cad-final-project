#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32
from WaypointsDatabase import WaypointsDatabase
import numpy as np

'''
This node will publish waypoints ahead of the car's current position.
Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.
'''

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        # Subscribe to input topics
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_callback)
        rospy.Subscriber('/base_waypoints', Lane, self.track_waypoints_callback)
        rospy.Subscriber("/traffic_waypoints", Int32, self.next_traffic_light_waypoint_callback)
        # rospy.Subscriber("/traffic_waypoints", Int32, self.traffic_lights_callback)


        # Publisher for computed final waypoints
        self.final_waypoints_publisher = rospy.Publisher('final_waypoints', Lane, queue_size=1)
        self.N = 100 # Number of waypoints to publish (planning horizon)
        self.max_deceleration = 0.5

        # Wait until the required info has been received
        rate = rospy.Rate(50) # 50Hz loop
        self.current_car_position = None
        self.waypoints_db = None
        self.next_traffic_light_stopline_index = -1
        self.tl_state = None
        rospy.loginfo("waiting for initial waypoint")
        while not rospy.is_shutdown():
            if self.current_car_position is not None and self.waypoints_db is not None:
                break
            rate.sleep()
        # Main loop for the node, running at a fixed rate
        while not rospy.is_shutdown():
            self.process()
            rate.sleep()
        
    def pose_callback(self, msg: PoseStamped):
        self.current_car_position = np.array([msg.pose.position.x, msg.pose.position.y]) # XYZ position of the car

    def track_waypoints_callback(self, msg: Lane):
        self.waypoints_db = WaypointsDatabase(msg.waypoints)
    
    def next_traffic_light_waypoint_callback(self, msg: Int32):
        self.next_traffic_light_stopline_index = msg.data

    def make_lane_msg(self):
        msg = Lane() 

        # Retrieve closest waypoint ahead of the car index
        _, closest_wpt_idx = self.waypoints_db.get_next_closest_idx(self.current_car_position)
        furthest_wpt_idx = closest_wpt_idx + self.N
        # Retrieve N waypoints ahead of the car
        waypoints_ahead = self.waypoints_db.waypoints[closest_wpt_idx:furthest_wpt_idx]

        # Check if there is a red traffic light ahead within 100 waypoints
        if self.next_traffic_light_stopline_index == -1:
            # no red traffic light ahead, then use waypoints
            msg.waypoints = waypoints_ahead
        else:
            # red traffic light ahead, then use waypoints with edited velocity to stop
            msg.waypoints = self.decelerate_waypoints(waypoints_ahead, closest_wpt_idx)
        
        return msg

    def decelerate_waypoints(self, waypoints, closest_idx):
        # make an array of waypoints with the same position but decreasing velocity
        waypts = []
        # Two waypoints back from stop line so that nose of car stops at line
        # max: used in case the car overshoots after the lane
        stop_idx = max(self.next_traffic_light_stopline_index - closest_idx - 3, 0)
        for i, wpt in enumerate(waypoints):
            p = Waypoint()
            # keep the same pose
            p.pose = wpt.pose
            # distance between p and stop waypoint
            brake_distance = self.dist_waypts(waypoints, i, stop_idx)
            # Kinematic equation: v²-u²=2as
            vel = np.sqrt(2 * self.max_deceleration * brake_distance)
            if vel < 1.:
                vel = 0.
            # Set waypoint speed
            p.twist.twist.linear.x = vel
            waypts.append(p)
        return waypts

    def dist_waypts(self, waypoints, wp1, wp2):
        # Loop through the waypoints and sum the distance between the consecutive points between wp1 and wp2
        dist = 0
        def distance_points(a,b):
            return np.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2)
        for i in range(wp1, wp2):
            dist += distance_points(waypoints[i].pose.pose.position, waypoints[i+1].pose.pose.position)
        return dist

    
    def process(self):
        # TODO: Use self.final_waypoints_pub to publish the next target waypoints
        # In phase 1: we can ignore traffic lights and simply output the next N waypoints *ahead of the car*, with their default velocity
        # In phase 2: you need to adjust target speeds on waypoints in order to smoothly brake until the car reaches the waypoint
        # corresponding to the next red light's stop line (stored in self.next_traffic_light_stopline_index, == -1 if no next traffic light).
        # Advice: make sure to complete dbw_node and have the car driving correctly while ignoring traffic lights before you tackle phase 2 
        final_lane = self.make_lane_msg()
        self.final_waypoints_publisher.publish(final_lane)

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except Exception as ex:
        rospy.logerr('Could not start waypoint updater node.')
        raise ex