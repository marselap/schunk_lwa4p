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
#include <std_msgs/Float64MultiArray.h>
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

class adaptiveImpedanceControl{
    public:
        adaptiveImpedanceControl();
        ~adaptiveImpedanceControl();
        void run();

    private:
        ros::Publisher lwa4pBlueDirectPositionResultPub, lwa4pBlueDirectOrientationResultPub;
        ros::Publisher lwa4pBlueJoint1Pub, lwa4pBlueJoint2Pub, lwa4pBlueJoint3Pub, lwa4pBlueJoint4Pub, lwa4pBlueJoint5Pub, lwa4pBlueJoint6Pub;
        ros::Publisher lwa4pBlueWaypointsPub;
        ros::Publisher robotBlueProfilePub;
        ros::Subscriber lwa4pBlueJointStatesSub;
        ros::Subscriber lwa4pBlueOperationModeSub;
        ros::Subscriber lwa4pBlueForceSensorSub;
        ros::Subscriber lwa4pBlueForceReferenceSub;
        ros::Subscriber lwa4pBluePositionReferenceSub;
        ros::Subscriber lwa4pBlueOrientationXReferenceSub;
        ros::Subscriber lwa4pBlueOrientationZReferenceSub;
        ros::Publisher lwa4pBluePositionReferencePub;
        ros::Publisher lwa4pBlueXcPub;

        int operation_mode; // 0 - no force control, 1 - force control active
        double force_x_mv, force_y_mv, force_z_mv;

        lwa4p_kinematics lwa4p_blue;

        schunk_lwa4p_trajectory::WaypointArray makeWaypointsMsg(Eigen::MatrixXd waypoints);
        std_msgs::Float64MultiArray makePositionMsg(Eigen::MatrixXd vector9x1);

        void lwa4pBlueJointStatesCallBack(const sensor_msgs::JointState &msg);
        void lwa4pOperationModeCallBack(const std_msgs::Int64 &msg);
        void lwa4pBlueForceSensorCallBack(const geometry_msgs::WrenchStamped &msg);
        void lwa4pBlueForceReferenceCallBack(const geometry_msgs::Vector3 &msg);
        void lwa4pBluePositionReferenceCallBack(const geometry_msgs::Vector3 &msg);
        void lwa4pBlueOrientationXReferenceCallBack(const geometry_msgs::Vector3 &msg);
        void lwa4pBlueOrientationZReferenceCallBack(const geometry_msgs::Vector3 &msg);

        //trajectory_msgs::JointTrajectory createRobotTrajectoryMsg(Eigen::MatrixXd new_Q);
        control_msgs::FollowJointTrajectoryActionGoal createRobotTrajectoryMsg(Eigen::MatrixXd new_Q, int time_to_reach);

        double wrapToPi(double angle);

        Eigen::MatrixXd tMatrixToPosVector(Eigen::MatrixXd matrix_pos);

        AdaptivePd forceController[3];

        // Move group
        ros::NodeHandle nhParams;
        ros::NodeHandle nhTopics;

        Eigen::MatrixXd lwa4p_blue_temp_q;

        Eigen::MatrixXd force;

        Eigen::MatrixXd Fr, Xref, xOrientation, zOrientation;

        bool contact_force;
        bool new_reference_flag;
};
