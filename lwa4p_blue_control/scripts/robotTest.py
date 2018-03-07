#!/usr/bin/env python

import rospy
from lwa4p_red_control.msg import lwa4pRedPoint
from control_msgs.msg import FollowJointTrajectoryActionGoal
from control_msgs.msg import JointTrajectoryControllerState
from trajectory_msgs.msg import JointTrajectoryPoint
from trajectory_msgs.msg import JointTrajectory
from std_msgs.msg import Empty
from std_msgs.msg import Header

class PublishPoint():
	def __init__(self):
		self.trajPub = rospy.Publisher('/arm_controller/follow_joint_trajectory/goal', FollowJointTrajectoryActionGoal, queue_size=10)
		self.pointPub = rospy.Publisher('/arm_controller/command', JointTrajectory, queue_size=10)
	
		self.msg2pub = FollowJointTrajectoryActionGoal()
		self.msg2 = JointTrajectory()


		


	def publish_msg(self):
		# Set the message to publish as command.
		traj_vector = FollowJointTrajectoryActionGoal()
		# Current ROS time stamp
		h = Header()
		h.stamp = rospy.Time.now()
		traj_vector.header = h
		traj_vector.goal.trajectory.joint_names.append('arm_1_joint');
		traj_vector.goal.trajectory.joint_names.append('arm_2_joint');
		traj_vector.goal.trajectory.joint_names.append('arm_3_joint');
		traj_vector.goal.trajectory.joint_names.append('arm_4_joint');
		traj_vector.goal.trajectory.joint_names.append('arm_5_joint');
		traj_vector.goal.trajectory.joint_names.append('arm_6_joint');
		traj_vector.goal_id.stamp = h.stamp
		traj_vector.goal_id.id = 'rosPathPlanner' + str(h.stamp.secs)
		h2 = Header()
		h2.stamp.secs = h.stamp.secs +1
		h2.stamp.nsecs = h.stamp.nsecs + 3e8
		traj_vector.goal.trajectory.header.stamp = h2.stamp

		points2pub = [[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]]

		iter = 0.0
		for column in points2pub:
			point = JointTrajectoryPoint()
			iter +=1
			for q in column:
				point.positions.append(q)
				point.time_from_start.nsecs = 0
				point.time_from_start.secs = iter*10
			traj_vector.goal.trajectory.points.append(point)
	
 

		print traj_vector
		print 'All systems go!'
		self.trajPub.publish(traj_vector)	

	def run(self):

		

		while not rospy.is_shutdown():

			self.publish_msg()
			print "Running!"
			rospy.sleep(2)


if __name__ == '__main__':

	rospy.init_node('RobotTest')
	node = PublishPoint()
	node.run()