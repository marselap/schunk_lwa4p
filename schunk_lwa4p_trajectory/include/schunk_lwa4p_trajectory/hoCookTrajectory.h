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
#include "yaml-cpp/yaml.h"

// Ros includes
#include "ros/ros.h"
#include <ros/package.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float64.h>

#include <schunk_lwa4p_trajectory/TrajectorySampled.h>
#include <schunk_lwa4p_trajectory/WaypointArray.h>

#include "control_msgs/FollowJointTrajectoryActionGoal.h"
#include "control_msgs/FollowJointTrajectoryActionFeedback.h"
#include "control_msgs/FollowJointTrajectoryActionResult.h"

#include "actionlib_msgs/GoalID.h"
#include "actionlib_msgs/GoalStatusArray.h"

#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <control_msgs/JointTrajectoryControllerState.h>

#include <moveit_msgs/ExecuteTrajectoryActionGoal.h>

using namespace std;

typedef struct DerivativeConditions
{
    double speed;
    double acceleration;
} DerivativeConditions;

typedef struct DerivativeConditionsStartEnd
{
    DerivativeConditions start;
    DerivativeConditions end;
} DerivativeConditionsStartEnd;

typedef struct TrajectoryDerivativeConditions
{
    DerivativeConditionsStartEnd Q1;
    DerivativeConditionsStartEnd Q2;
    DerivativeConditionsStartEnd Q3;
    DerivativeConditionsStartEnd Q4;
    DerivativeConditionsStartEnd Q5;
    DerivativeConditionsStartEnd Q6;
} TrajectoryDerivativeConditions;


class trajectoryPlanning
{
public:
	trajectoryPlanning();
	~trajectoryPlanning();
	void run();
	// mav_path_trajectory::TrajectorySampled plan(mav_path_trajectory::WaypointArray wp);
    void setTrajectorySamplingFrequency(int freq);
    void setJointMaxSpeed(double speed[6]);
    void setJointMaxAcc(double acc[6]);

private:
	// Node handles for topics and params
    ros::NodeHandle nhParams;
    ros::NodeHandle nhTopics;

    // Publishers and subscribers
    ros::Publisher trajectorySampledRedPub, trajectorySampledBluePub;
    ros::Subscriber waypoints_redSub, waypoints_blueSub, waypoints_moveit;

    ros::Publisher robotTrajectoryRedPub;

    // Schunk joints params
    double maxSpeed[6], maxAcc[6];
    int trajectorySamplingFrequency;

    // Trajectory params
    double trajectoryAccScaler, trajectorySpeedScaler;

    // HoCook dimensions
    int n;  // Joint number
    int m;  // Waypoint number

    // Waypoint callback and Eigen matrix where to save it
    void waypointRedCallback(const schunk_lwa4p_trajectory::WaypointArray &msg);
    void waypointBlueCallback(const schunk_lwa4p_trajectory::WaypointArray &msg);
    void waypointMoveitCallback(const moveit_msgs::ExecuteTrajectoryActionGoal &msg);
    Eigen::MatrixXd Waypoints;


    // Parametric time
    void computeParametricTime(
        Eigen::MatrixXd WP_Q,
        std::vector<double> &paramTimeArray);

    // Function for creating matrices M, A
    void createMatrixM(std::vector<double> t, Eigen::MatrixXd &M);
    void createMatrixA(std::vector<double> t, Eigen::MatrixXd &Q, Eigen::MatrixXd &A);

    // Compute polynom coeffs
    void computeCoeffsB(std::vector<Eigen::MatrixXd> &B,
        Eigen::MatrixXd WP_Q, Eigen::MatrixXd Dq,
        std::vector<double> t);

    void findMaxValues(Eigen::MatrixXd B,
        double &max_speed,
        double &max_acc,
        double t0,
        double tf);

    // Ho cook 34 related functions
    double hoCook34(
        std::vector<Eigen::MatrixXd> &B,
        Eigen::MatrixXd WP_Q,
        TrajectoryDerivativeConditions initialConditions,
        std::vector<double> paramTime);

    schunk_lwa4p_trajectory::TrajectorySampled optimizedPlan(
         Eigen::MatrixXd WP_Q);

    // Sample trajectory
    schunk_lwa4p_trajectory::TrajectorySampled sampleTrajectory(
        std::vector<Eigen::MatrixXd> &B,
        std::vector<double> &tf,
        int sampleFrequency);

    schunk_lwa4p_trajectory::TrajectorySampled point2point(
        Eigen::MatrixXd WP_Q);

    control_msgs::FollowJointTrajectoryActionGoal createRobotTrajectoryMsg(
        schunk_lwa4p_trajectory::TrajectorySampled sampledTrajectory);

    // Returns polynomial value for 3rd, 4th order polynomials
    // Older functions returning values for specific order polynomial
    void calculatePolynomialValueOrder3(std::vector<double> B, double t,
        double &position, double &speed, double &acceleration);
    void calculatePolynomialValueOrder4(std::vector<double> B, double t,
        double &position, double &speed, double &acceleration);
};
