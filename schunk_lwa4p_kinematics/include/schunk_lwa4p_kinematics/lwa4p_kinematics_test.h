/******************************************************************************
File name: lwa4p_kinematics_Test.h
Description: Node for testing Schunk LWA4P kinematics! 
Author: Bruno Maric
******************************************************************************/

#include "ros/ros.h"
#include <ros/package.h>

#include <sstream>
#include <fstream>
#include <vector>
#include <stdlib.h>
#include <math.h>
#include <string>
#include <iostream>
#include <stdio.h>
#include <signal.h>
#include <stdbool.h>
#include <unistd.h>
#include <errno.h>
#include <dirent.h>
#include <time.h>
#include <eigen3/Eigen/Eigen>

#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>

#include <schunk_lwa4p_trajectory/WaypointArray.h>
#include <schunk_lwa4p_kinematics/lwa4p_kinematics.h>

using namespace std;

class kinematicsTest{
	public:
		kinematicsTest();
		~kinematicsTest();
		void run();

	private:
		ros::Publisher lwa4pBlueDirectPositionResultPub, lwa4pBlueDirectOrientationResultPub;
		ros::Publisher lwa4pRedDirectPositionResultPub, lwa4pRedDirectOrientationResultPub;
		ros::Publisher lwa4pRedJoint1Pub, lwa4pRedJoint2Pub, lwa4pRedJoint3Pub, lwa4pRedJoint4Pub, lwa4pRedJoint5Pub, lwa4pRedJoint6Pub;
		ros::Publisher lwa4pBlueJoint1Pub, lwa4pBlueJoint2Pub, lwa4pBlueJoint3Pub, lwa4pBlueJoint4Pub, lwa4pBlueJoint5Pub, lwa4pBlueJoint6Pub;
		ros::Publisher lwa4pBlueWaypointsPub, lwa4pRedWaypointsPub;
		ros::Subscriber alphaSub, betaSub;
		ros::Subscriber lwa4pBlueJointStatesSub, lwa4pRedJointStatesSub; 
		
		lwa4p_kinematics lwa4p_blue, lwa4p_red;

		void lwa4pBlueJointStatesCallBack(const sensor_msgs::JointState &msg);
		void lwa4pRedJointStatesCallBack(const sensor_msgs::JointState &msg);
		void alphaCallBack(const std_msgs::Float64 &msg);
		void betaCallBack(const std_msgs::Float64 &msg);


		schunk_lwa4p_trajectory::WaypointArray makeWaypointsMsg(Eigen::MatrixXd waypoints);

		// Move group
	 	ros::NodeHandle nhParams;
	 	ros::NodeHandle nhTopics;

	 	Eigen::MatrixXd lwa4p_blue_temp_q, lwa4p_red_temp_q;
	 	

	 	geometry_msgs::Vector3 vector_temp;
	 	std_msgs::Float64 msg2pub;

	 	// Point test
	 	double alpha, beta; // used to specify point to mark
	 	double sphere_R;
	 	double point_pose_x, point_pose_y, point_pose_z;
	 	double point_z_x, point_z_y, point_z_z;
	 	double point_x_x, point_x_y, point_x_z;
	 	double point_y_x, point_y_y, point_y_z;

	 	

};