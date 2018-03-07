#!/usr/bin/env python

import rospy
from lwa4p_red_control.msg import lwa4pRedPoint

class PublishPoint():
	def __init__(self):
		self.pointPub = rospy.Publisher('/lwa4p_red/process_point', lwa4pRedPoint, queue_size = 10);

		self.msg2pub = lwa4pRedPoint()


		self.msg2pub.alpha.data = 1
		self.msg2pub.beta.data = 1



	def run(self):

		

		while not rospy.is_shutdown():

			self.pointPub.publish(self.msg2pub)
			print (self.msg2pub.alpha, self.msg2pub.beta)
			print "Running!"
			rospy.sleep(5)


if __name__ == '__main__':

	rospy.init_node('PublishPoint')
	node = PublishPoint()
	node.run()