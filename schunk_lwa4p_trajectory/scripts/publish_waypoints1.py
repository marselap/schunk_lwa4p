#!/usr/bin/env python

import rospy
from schunk_lwa4p_trajectory.msg import WaypointArray

class PublishWaypoints():
    def __init__(self):
        self.waypointsPub = rospy.Publisher('lwa4p_blue/waypoints', WaypointArray, queue_size = 10);

        self.msg2pub = WaypointArray()

        '''
        self.msg2pub.waypoint_Q1 = [0,   -0.5101,   -0.3973,   -0.2727,         0,    0.2727,    0.3973,    0.5101,    0.3973,    0.2727,         0,   -0.2727,   -0.3973,   -0.5101,         0];
        self.msg2pub.waypoint_Q2 = [0,   -1.7249,   -1.7532,   -1.7279,   -1.5827,   -1.7279,   -1.7532,   -1.7249,   -1.7532,   -1.7279,   -1.5827,   -1.7279,   -1.7532,   -1.7249,         0];
        self.msg2pub.waypoint_Q3 = [0,   -0.6254,   -0.8630,   -0.9787,   -0.9527,   -0.9787,   -0.8630,   -0.6254,   -0.8630,   -0.9787,   -0.9527,   -0.9787,   -0.8630,   -0.6254,         0];
        self.msg2pub.waypoint_Q4 = [0,    0.8890,    0.5881,    0.3649,         0,   -0.3649,   -0.5881,   -0.8890,   -0.5881,   -0.3649,         0,    0.3649,    0.5881,    0.8890,         0];
        self.msg2pub.waypoint_Q5 = [0,    0.6800,    0.7718,    0.8554,    0.9408,    0.8554,    0.7718,    0.6800,    0.7718,    0.8554,    0.9408,    0.8554,    0.7718,    0.6800,         0];
        self.msg2pub.waypoint_Q6 = [0,         0,         0,         0,         0,         0,         0,         0,         0,         0,         0,         0,         0,         0,         0];
         '''
        '''
        self.msg2pub.waypoint_Q1 = [0,   -0.5101,   -0.3973,         0];
        self.msg2pub.waypoint_Q2 = [0,   -1.7249,   -1.7532,         0];
        self.msg2pub.waypoint_Q3 = [0,   -0.6254,   -0.8630,         0];
        self.msg2pub.waypoint_Q4 = [0,    0.8890,    0.5881,         0];
        self.msg2pub.waypoint_Q5 = [0,    0.6800,    0.7718,         1.57];
        self.msg2pub.waypoint_Q6 = [0,         0,         0,         1.57];
        '''
        self.msg2pub.waypoint_Q1 = [0]
        self.msg2pub.waypoint_Q2 = [0.5]
        self.msg2pub.waypoint_Q3 = [-1.1]
        self.msg2pub.waypoint_Q4 = [0.0]
        self.msg2pub.waypoint_Q5 = [-0.25]
        self.msg2pub.waypoint_Q6 = [0.7]


    def run(self):



        while not rospy.is_shutdown():

            self.waypointsPub.publish(self.msg2pub)
            print self.msg2pub.waypoint_Q1
            print "Running!"
            rospy.sleep(5)


if __name__ == '__main__':

    rospy.init_node('PublishWaypoints')
    node = PublishWaypoints()
    node.run()
