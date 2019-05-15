#!/usr/bin/env python

import rospy
from schunk_lwa4p_trajectory.msg import WaypointArray, TrajectorySampled
from std_msgs.msg import Float64
from control_msgs.msg import FollowJointTrajectoryActionGoal
from control_msgs.msg import JointTrajectoryControllerState
from trajectory_msgs.msg import JointTrajectoryPoint
from trajectory_msgs.msg import JointTrajectory
from std_msgs.msg import Empty
from std_msgs.msg import Header
import time
import math

class ListenTrajectory():
    def __init__(self):
        rospy.Subscriber('lwa4p_blue/trajectory_sampled', TrajectorySampled, self.trajectoryBlue_cb, queue_size=1);
        
        #rospy.Subscriber('lwa4p_red/trajectory_sampled', TrajectorySampled, self.trajectoryRed_cb);
        #rospy.Subscriber('lwa4p_red/waypoints', WaypointArray, self.waypoints_cb);

        #self.trajPub = rospy.Publisher('/arm_controller/follow_joint_trajectory/goal', FollowJointTrajectoryActionGoal, queue_size=10)
        self.trajPub_blue = rospy.Publisher('/pos_based_pos_traj_controller_arm/follow_joint_trajectory/goal', FollowJointTrajectoryActionGoal, queue_size=10)
        #self.trajPub_red = rospy.Publisher('/red_robot/pos_based_pos_traj_controller_arm/follow_joint_trajectory/goal', FollowJointTrajectoryActionGoal, queue_size=10)

        """
        self.lwa4p_red_joint1_pub = rospy.Publisher('lwa4p_red/arm_1_joint_pos_controller/command', Float64, queue_size = 10);
        self.lwa4p_red_joint2_pub = rospy.Publisher('lwa4p_red/arm_2_joint_pos_controller/command', Float64, queue_size = 10);
        self.lwa4p_red_joint3_pub = rospy.Publisher('lwa4p_red/arm_3_joint_pos_controller/command', Float64, queue_size = 10);
        self.lwa4p_red_joint4_pub = rospy.Publisher('lwa4p_red/arm_4_joint_pos_controller/command', Float64, queue_size = 10);
        self.lwa4p_red_joint5_pub = rospy.Publisher('lwa4p_red/arm_5_joint_pos_controller/command', Float64, queue_size = 10);
        self.lwa4p_red_joint6_pub = rospy.Publisher('lwa4p_red/arm_6_joint_pos_controller/command', Float64, queue_size = 10);
        """
        self.lwa4p_blue_joint1_pub = rospy.Publisher('lwa4p_blue/arm_1_joint_pos_controller/command', Float64, queue_size = 10);
        self.lwa4p_blue_joint2_pub = rospy.Publisher('lwa4p_blue/arm_2_joint_pos_controller/command', Float64, queue_size = 10);
        self.lwa4p_blue_joint3_pub = rospy.Publisher('lwa4p_blue/arm_3_joint_pos_controller/command', Float64, queue_size = 10);
        self.lwa4p_blue_joint4_pub = rospy.Publisher('lwa4p_blue/arm_4_joint_pos_controller/command', Float64, queue_size = 10);
        self.lwa4p_blue_joint5_pub = rospy.Publisher('lwa4p_blue/arm_5_joint_pos_controller/command', Float64, queue_size = 10);
        self.lwa4p_blue_joint6_pub = rospy.Publisher('lwa4p_blue/arm_6_joint_pos_controller/command', Float64, queue_size = 10);

        self.Q1 = []
        self.Q2 = []
        self.Q3 = []
        self.Q4 = []
        self.Q5 = []
        self.Q6 = []


    def trajectoryRed_cb(self, msg):
        self.Q1 = msg.pose_joint_1
        self.Q2 = msg.pose_joint_2
        self.Q3 = msg.pose_joint_3
        self.Q4 = msg.pose_joint_4
        self.Q5 = msg.pose_joint_5
        self.Q6 = msg.pose_joint_6

        if len(self.Q1) > 1:
            start_time = 2
            step_time = 0.02
            print "CIJELA PORUKA"
            poruka = self.createRobotMsg(msg, start_time, step_time)

            #print len(self.Q1)
            for i in range(0, len(self.Q1)):
                if (abs(self.Q1[i]) < 3.14) and (abs(self.Q2[i]) < 3.14) and (abs(self.Q3[i]) < 3.14) and (abs(self.Q4[i]) < 3.14) and (abs(self.Q5[i]) < 3.14) and  (abs(self.Q6[i]) < 3.14):

                    self.lwa4p_red_joint1_pub.publish(self.Q1[i])
                    self.lwa4p_red_joint2_pub.publish(self.Q2[i])
                    self.lwa4p_red_joint3_pub.publish(self.Q3[i])
                    self.lwa4p_red_joint4_pub.publish(self.Q4[i])
                    self.lwa4p_red_joint5_pub.publish(self.Q5[i])
                    self.lwa4p_red_joint6_pub.publish(self.Q6[i])
                    #rospy.sleep(1.0)
                    #print (self.Q1[i], self.Q2[i], self.Q3[i], self.Q4[i], self.Q5[i], self.Q6[i])
                    rospy.sleep(1.0/50)
        else:
            start_time = 0
            step_time = 1
            poruka = self.createRobotMsg(msg, start_time, step_time)
            print "JEDNA TOCKA"
        #print poruka
        self.trajPub_red.publish(poruka)
        #print 'Published to red robot'
        #print self.Q1[0], self.Q2[0], self.Q3[0], self.Q4[0], self.Q5[0], self.Q6[0]
        print len(self.Q1)
        print self.Q1[0]
        print self.Q1[-1]


    def trajectoryBlue_cb(self, msg):
        self.Q1 = msg.pose_joint_1
        self.Q2 = msg.pose_joint_2
        self.Q3 = msg.pose_joint_3
        self.Q4 = msg.pose_joint_4
        self.Q5 = msg.pose_joint_5
        self.Q6 = msg.pose_joint_6

        start_time = 1.0
        step_time = 0.02

        poruka = self.createRobotMsg(msg, start_time, step_time)
        #print poruka
        self.trajPub_blue.publish(poruka)
        print 'Published to blue robot'

        print len(self.Q1)
        for i in range(0, len(self.Q1)):
            if (abs(self.Q1[i]) < 3.14) and (abs(self.Q2[i]) < 3.14) and (abs(self.Q3[i]) < 3.14) and (abs(self.Q4[i]) < 3.14) and (abs(self.Q5[i]) < 3.14) and  (abs(self.Q6[i]) < 3.14):

                self.lwa4p_blue_joint1_pub.publish(self.Q1[i])
                self.lwa4p_blue_joint2_pub.publish(self.Q2[i])
                self.lwa4p_blue_joint3_pub.publish(self.Q3[i])
                self.lwa4p_blue_joint4_pub.publish(self.Q4[i])
                self.lwa4p_blue_joint5_pub.publish(self.Q5[i])
                self.lwa4p_blue_joint6_pub.publish(self.Q6[i])

                #ospy.sleep(1.0/50)
                rospy.sleep(1.0/50)
            #print (self.Q1[i], self.Q2[i], self.Q3[i], self.Q4[i], self.Q5[i], self.Q6[i])
        print "End msg"


    def createRobotMsg(self, msg, start_time, step_time):

        poruka = FollowJointTrajectoryActionGoal()

        h = Header()
        h.stamp = rospy.get_rostime()
        #print 'Start H'
        #print h
        #print 'end H'
        poruka.header = h

        poruka.goal.trajectory.joint_names.append('arm_1_joint')
        poruka.goal.trajectory.joint_names.append('arm_2_joint')
        poruka.goal.trajectory.joint_names.append('arm_3_joint')
        poruka.goal.trajectory.joint_names.append('arm_4_joint')
        poruka.goal.trajectory.joint_names.append('arm_5_joint')
        poruka.goal.trajectory.joint_names.append('arm_6_joint')

        poruka.goal_id.id = 'HoCook trajectory ' + str(h.stamp.nsecs)

        #h.stamp = rospy.get_rostime()
        h2 = Header()
        h2.stamp.secs = 0
        h2.stamp.nsecs = 0

        poruka.goal.trajectory.header.stamp = h2.stamp;
        poruka.header.stamp = h.stamp

        temp_time = start_time

        for i in range(0, len(msg.pose_joint_1)):
        #for i in range(0, 100):
            tocka = JointTrajectoryPoint()
            '''
            tocka.positions.append(0-1.57)
            tocka.positions.append(0)
            tocka.positions.append(1.57)    # INVERTIRATI!!!!
            tocka.positions.append(i*0.5)
            tocka.positions.append(0)
            tocka.positions.append(0)
            '''
            tocka.positions.append(msg.pose_joint_1[i])
            tocka.positions.append(msg.pose_joint_2[i])
            tocka.positions.append(msg.pose_joint_3[i])     # INVERTIRATI!!!!
            tocka.positions.append(msg.pose_joint_4[i])
            tocka.positions.append(msg.pose_joint_5[i])
            tocka.positions.append(msg.pose_joint_6[i])

            tocka.velocities.append(msg.speed_joint_1[i])
            tocka.velocities.append(msg.speed_joint_2[i])
            tocka.velocities.append(msg.speed_joint_3[i])
            tocka.velocities.append(msg.speed_joint_4[i])
            tocka.velocities.append(msg.speed_joint_5[i])
            tocka.velocities.append(msg.speed_joint_6[i])

            tocka.accelerations.append(msg.acc_joint_1[i])
            tocka.accelerations.append(msg.acc_joint_2[i])
            tocka.accelerations.append(msg.acc_joint_3[i])
            tocka.accelerations.append(msg.acc_joint_4[i])
            tocka.accelerations.append(msg.acc_joint_5[i])
            tocka.accelerations.append(msg.acc_joint_6[i])

            '''
            tocka.velocities.append(0)
            tocka.velocities.append(0)
            tocka.velocities.append(0)
            tocka.velocities.append(0)
            tocka.velocities.append(0)
            tocka.velocities.append(0)

            tocka.accelerations.append(0)
            tocka.accelerations.append(0)
            tocka.accelerations.append(0)
            tocka.accelerations.append(0)
            tocka.accelerations.append(0)
            tocka.accelerations.append(0)
            '''
            temp_time = temp_time + step_time

            tocka.time_from_start.nsecs = round((temp_time - math.floor(temp_time))*pow(10, 9))
            tocka.time_from_start.secs = int(round(math.floor(temp_time)))
            #print 'time = ', str(temp_time), ', ', str(math.floor(temp_time)), ', ', str(round((temp_time - math.floor(temp_time))*pow(10, 9))), ', '
            poruka.goal.trajectory.points.append(tocka);

        return poruka

    def run(self):

        while not rospy.is_shutdown():
            #print "Running!"
            rospy.sleep(1)


if __name__ == '__main__':

    rospy.init_node('ListenTrajectory')
    node = ListenTrajectory()
    node.run()
