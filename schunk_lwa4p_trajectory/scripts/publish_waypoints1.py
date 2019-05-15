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

        self.msg2pub.waypoint_Q1 = [0.0]
        self.msg2pub.waypoint_Q2 = [-0.52] #-0.5794365034946314
        self.msg2pub.waypoint_Q3 = [2.15] #1.967612461416539,
        self.msg2pub.waypoint_Q4 = [-2.82] # -2.82
        self.msg2pub.waypoint_Q5 = [1.12] #0.9745262353851335,
        self.msg2pub.waypoint_Q6 = [-0.22] #-0.153399588393914
        '''
        self.msg2pub.waypoint_Q1 = [0.00]
        self.msg2pub.waypoint_Q2 = [0.00]
        self.msg2pub.waypoint_Q3 = [0.0]
        self.msg2pub.waypoint_Q4 = [0.0]
        self.msg2pub.waypoint_Q5 = [-0.0]
        self.msg2pub.waypoint_Q6 = [0.0]
        '''

    def run(self):


        i = 0
        while not rospy.is_shutdown():

            if i < 2:
                self.waypointsPub.publish(self.msg2pub)
                print self.msg2pub.waypoint_Q1
                print "Running!"
            i += 1
            rospy.sleep(1)


if __name__ == '__main__':

    rospy.init_node('PublishWaypoints')
    node = PublishWaypoints()
    node.run()
