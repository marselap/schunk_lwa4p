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

#include <Eigen/Core>
#include <Eigen/SVD>

#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/WrenchStamped.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int64.h>
#include <sensor_msgs/JointState.h>

#include <schunk_lwa4p_trajectory/WaypointArray.h>
#include <schunk_lwa4p_trajectory/TrajectorySampled.h>
#include <schunk_lwa4p_kinematics/lwa4p_kinematics.h>
#include "lwa4p_blue_control/pid_controller_base.h"
#include "lwa4p_blue_control/adaptive_pd.h"

#include "control_msgs/FollowJointTrajectoryActionGoal.h"
#include "control_msgs/FollowJointTrajectoryActionFeedback.h"
#include "control_msgs/FollowJointTrajectoryActionResult.h"



#include "actionlib_msgs/GoalID.h"
#include "actionlib_msgs/GoalStatusArray.h"

#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <control_msgs/JointTrajectoryControllerState.h>




using namespace std;

class lwa4pBlueControl{
	public:
		lwa4pBlueControl();
		~lwa4pBlueControl();
		void run();

	private:
		ros::Publisher lwa4pBlueDirectPositionResultPub, lwa4pBlueDirectOrientationResultPub;
		ros::Publisher lwa4pRedDirectPositionResultPub, lwa4pRedDirectOrientationResultPub;
		ros::Publisher lwa4pRedJoint1Pub, lwa4pRedJoint2Pub, lwa4pRedJoint3Pub, lwa4pRedJoint4Pub, lwa4pRedJoint5Pub, lwa4pRedJoint6Pub;
		ros::Publisher lwa4pBlueJoint1Pub, lwa4pBlueJoint2Pub, lwa4pBlueJoint3Pub, lwa4pBlueJoint4Pub, lwa4pBlueJoint5Pub, lwa4pBlueJoint6Pub;
		ros::Publisher lwa4pBlueWaypointsPub, lwa4pRedWaypointsPub;
		ros::Publisher robotRedProfilePub, robotBlueProfilePub;
		ros::Subscriber processPointSub;
		ros::Subscriber lwa4pBlueJointStatesSub, lwa4pRedJointStatesSub;
		ros::Subscriber lwa4pBlueOperationModeSub;
		ros::Subscriber lwa4pBlueForceSensorSub;

		int operation_mode; // 0 - no force control, 1 - force control active
		double force_x_mv, force_y_mv, force_z_mv;

		lwa4p_kinematics lwa4p_blue;

		schunk_lwa4p_trajectory::WaypointArray makeWaypointsMsg(Eigen::MatrixXd waypoints);

		Eigen::MatrixXd calcJacobian(Eigen::MatrixXd temp_rob_q);

		void lwa4pBlueJointStatesCallBack(const sensor_msgs::JointState &msg);
		void lwa4pOperationModeCallBack(const std_msgs::Int64 &msg);
		void lwa4pBlueForceSensorCallBack(const geometry_msgs::WrenchStamped &msg);


		Eigen::MatrixXd lineMovement(Eigen::MatrixXd delta_, Eigen::MatrixXd temp_q);

		//trajectory_msgs::JointTrajectory createRobotTrajectoryMsg(Eigen::MatrixXd new_Q);
		control_msgs::FollowJointTrajectoryActionGoal createRobotTrajectoryMsg(Eigen::MatrixXd new_Q, int time_to_reach);


		double wrapToPi(double angle);

		PidControllerBase forceCtrlPID;

		// Move group
	 	ros::NodeHandle nhParams;
	 	ros::NodeHandle nhTopics;

	 	Eigen::MatrixXd lwa4p_blue_temp_q, lwa4p_red_temp_q;





};
