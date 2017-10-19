#!/usr/bin/env python

from scipy.spatial import KDTree

import rospy
from std_msgs.msg import Float32, Int32
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint

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

LOOKAHEAD_WPS = 100 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below


        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.waypoints = self.pose = None
        self.waypoints_plane = None

        self.loop()

    def loop(self):
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            if self.pose is not None and self.waypoints is not None:
                closest_waypoint = self.get_closest_waypoint(self.pose.pose)
                self.publish_waypoints(closest_waypoint)
            rate.sleep()

    def pose_cb(self, msg):
        # TODO: Implement
        self.pose = msg

    def waypoints_cb(self, waypoints):
        if self.waypoints_plane is None:
            self.waypoints_plane = []
            for waypoint in waypoints.waypoints:
                self.waypoints_plane.append(
                    [waypoint.pose.pose.position.x, waypoint.pose.pose.position.y])

            self.waypoint_tree = KDTree(self.waypoints_plane)

        self.waypoints = waypoints

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

    def get_closest_waypoint(self, pose):
        return self.waypoint_tree.query(
            [pose.position.x, pose.position.y], 1)[1]

    def publish_waypoints(self, closest_waypoint):
        if closest_waypoint is None or self.waypoints is None or\
           closest_waypoint < 0 or closest_waypoint >= len(self.waypoints.waypoints):
            return

        temp = Lane()
        temp.header = self.waypoints.header

        current_waypoint = Waypoint()
        current_waypoint.pose = self.pose
        current_waypoint.twist = self.waypoints.waypoints[closest_waypoint].twist

        temp.waypoints.append(current_waypoint)

        for i in range(LOOKAHEAD_WPS):
            wp = closest_waypoint + i
            if wp >= len(self.waypoints.waypoints):
                break
            temp.waypoints.append(self.waypoints.waypoints[wp])
        self.final_waypoints_pub.publish(temp)

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
