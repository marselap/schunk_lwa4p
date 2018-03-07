#!/usr/bin/env python

import rospy
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Float64
from std_msgs.msg import Empty
from std_msgs.msg import Header
import time
import math

class SchunkWallDistance():
    def __init__(self):
        rospy.Subscriber('gazebo/model_states', ModelStates, self.calc_dist_cb, queue_size=1);

        self.distance = rospy.Publisher('lwa4p_blue/wall_distance', Float64, queue_size = 10);

    def calc_dist_cb(self, msg):
        posSchunk = msg.pose[2].position.x
        posWall = msg.pose[1].position.x

        self.distance.publish(posSchunk - posWall)
        //print(posSchunk - posWall)

    def run(self):
        while not rospy.is_shutdown():
            #print "Running!"
            rospy.sleep(1)

if __name__ == '__main__':

    rospy.init_node('SchunkWallDistance')
    print 'started'
    node = SchunkWallDistance()
    node.run()
