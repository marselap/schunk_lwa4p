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

class moveRobotClass():
	def __init__(self):
		#rospy.Subscriber('lwa4p_blue/trajectory_sampled', TrajectorySampled, self.trajectoryBlue_cb);
		#rospy.Subscriber('lwa4p_red/trajectory_sampled', TrajectorySampled, self.trajectoryRed_cb);
		#rospy.Subscriber('lwa4p_red/waypoints', WaypointArray, self.waypoints_cb);

		self.trajPub_blue = rospy.Publisher('/blue_robot/pos_based_pos_traj_controller_arm/follow_joint_trajectory/goal', FollowJointTrajectoryActionGoal, queue_size=10)
		self.trajPub_red = rospy.Publisher('/red_robot/pos_based_pos_traj_controller_arm/follow_joint_trajectory/goal', FollowJointTrajectoryActionGoal, queue_size=10)

		#self.trajPub_blue = rospy.Publisher('/blue_robot/schunk_canopen_node/follow_joint_trajectory/goal', FollowJointTrajectoryActionGoal, queue_size=10)
		#self.trajPub_red = rospy.Publisher('/red_robot/schunk_canopen_node/follow_joint_trajectory/goal', FollowJointTrajectoryActionGoal, queue_size=10)

		#self.trajPub = rospy.Publisher('/arm_controller/follow_joint_trajectory/goal', FollowJointTrajectoryActionGoal, queue_size=10)
		#self.trajPub = rospy.Publisher('/schunk_canopen_node/follow_joint_trajectory/goal', FollowJointTrajectoryActionGoal, queue_size=10)

		'''
		self.lwa4p_red_joint1_pub = rospy.Publisher('lwa4p_red/arm_1_joint_pos_controller/command', Float64, queue_size = 1);
		self.lwa4p_red_joint2_pub = rospy.Publisher('lwa4p_red/arm_2_joint_pos_controller/command', Float64, queue_size = 1);
		self.lwa4p_red_joint3_pub = rospy.Publisher('lwa4p_red/arm_3_joint_pos_controller/command', Float64, queue_size = 1);
		self.lwa4p_red_joint4_pub = rospy.Publisher('lwa4p_red/arm_4_joint_pos_controller/command', Float64, queue_size = 1);
		self.lwa4p_red_joint5_pub = rospy.Publisher('lwa4p_red/arm_5_joint_pos_controller/command', Float64, queue_size = 1);
		self.lwa4p_red_joint6_pub = rospy.Publisher('lwa4p_red/arm_6_joint_pos_controller/command', Float64, queue_size = 1);

		self.lwa4p_blue_joint1_pub = rospy.Publisher('lwa4p_blue/arm_1_joint_pos_controller/command', Float64, queue_size = 1);
		self.lwa4p_blue_joint2_pub = rospy.Publisher('lwa4p_blue/arm_2_joint_pos_controller/command', Float64, queue_size = 1);
		self.lwa4p_blue_joint3_pub = rospy.Publisher('lwa4p_blue/arm_3_joint_pos_controller/command', Float64, queue_size = 1);
		self.lwa4p_blue_joint4_pub = rospy.Publisher('lwa4p_blue/arm_4_joint_pos_controller/command', Float64, queue_size = 1);
		self.lwa4p_blue_joint5_pub = rospy.Publisher('lwa4p_blue/arm_5_joint_pos_controller/command', Float64, queue_size = 1);
		self.lwa4p_blue_joint6_pub = rospy.Publisher('lwa4p_blue/arm_6_joint_pos_controller/command', Float64, queue_size = 1);
		'''

		self.Q1 = []
		self.Q2 = []
		self.Q3 = []
		self.Q4 = []
		self.Q5 = []
		self.Q6 = []

	'''
	def trajectoryRed_cb(self, msg):
		self.Q1 = msg.pose_joint_1
		self.Q2 = msg.pose_joint_2
		self.Q3 = msg.pose_joint_3
		self.Q4 = msg.pose_joint_4
		self.Q5 = msg.pose_joint_5
		self.Q6 = msg.pose_joint_6



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
				print (self.Q1[i], self.Q2[i], self.Q3[i], self.Q4[i], self.Q5[i], self.Q6[i])




				rospy.sleep(1.0/50)


	def trajectoryBlue_cb(self, msg):
		self.Q1 = msg.pose_joint_1
		self.Q2 = msg.pose_joint_2
		self.Q3 = msg.pose_joint_3
		self.Q4 = msg.pose_joint_4
		self.Q5 = msg.pose_joint_5
		self.Q6 = msg.pose_joint_6

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
	'''

	def moveRobot(self):

		poruka = FollowJointTrajectoryActionGoal()

		h = Header()
		h.stamp = rospy.get_rostime()
		print 'Start H'
		print h
		print 'end H'
		poruka.header = h

		poruka.goal.trajectory.joint_names.append('arm_1_joint')
		poruka.goal.trajectory.joint_names.append('arm_2_joint')
		poruka.goal.trajectory.joint_names.append('arm_3_joint')
		poruka.goal.trajectory.joint_names.append('arm_4_joint')
		poruka.goal.trajectory.joint_names.append('arm_5_joint')
		poruka.goal.trajectory.joint_names.append('arm_6_joint')

		poruka.goal_id.id = 'Move trajectory ' + str(h.stamp.nsecs)

		h.stamp = rospy.get_rostime()
		poruka.goal.trajectory.header.stamp = h.stamp;
		poruka.header.stamp = h.stamp

		tocka = JointTrajectoryPoint()

		'''
		tocka.positions.append(0)
		tocka.positions.append(-1.58)		# 	INVERTIRATI NA CANOPEN
		tocka.positions.append(1.38005) 	# -1.38005 INVERTIRATI na IPI!!!!
		tocka.positions.append(0)
		tocka.positions.append(1.36365) # -1.36365  # 	INVERTIRATI NA CANOPEN
		tocka.positions.append(0)
		'''
		if (1==1):
			tocka.positions.append(0)
			tocka.positions.append(0)
			tocka.positions.append(0*-1.57) 	# INVERTIRATI!!!!
			tocka.positions.append(0)
			tocka.positions.append(0)
			tocka.positions.append(0)
		else:

			tocka.positions.append(0)
			tocka.positions.append(0.0)
			tocka.positions.append(1.77) 	# INVERTIRATI!!!!
			tocka.positions.append(0.0)
			tocka.positions.append(-0.2)
			tocka.positions.append(0.9)


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



		tocka.time_from_start.nsecs = 0*pow(10, 9)
		tocka.time_from_start.secs = 7

		poruka.goal.trajectory.points.append(tocka);



		#self.trajPub_red.publish(poruka)
		self.trajPub_blue.publish(poruka)


		print ""
		print poruka
		print rospy.get_rostime()
		print h.stamp



	def run(self):

		while not rospy.is_shutdown():


			#print "Running!"

			rospy.sleep(2)
			self.moveRobot()


if __name__ == '__main__':

	rospy.init_node('MoveRobot')
	node = moveRobotClass()
	node.run()
