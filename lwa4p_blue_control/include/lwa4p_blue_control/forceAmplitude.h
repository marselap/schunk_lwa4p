/******************************************************************************
File name: forceAmplitude.h
Description: Node for force sensor data filtering and analysis
Author: MP
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
#include <sensor_msgs/JointState.h>

#include <schunk_lwa4p_kinematics/lwa4p_kinematics.h>


using namespace std;

class forceAmplitude{
    public:
        forceAmplitude();
        ~forceAmplitude();
        void run();

    private:
        ros::Publisher lwa4pBlueForceLocal, lwa4pBlueForceGlobal;
        ros::Subscriber lwa4pBlueJointStatesSub;
        ros::Subscriber lwa4pBlueForceSensorSub;

        Eigen::MatrixXd force_reading;

        lwa4p_kinematics lwa4p_blue;

        void lwa4pBlueJointStatesCallBack(const sensor_msgs::JointState &msg);
        void lwa4pBlueForceSensorCallBack(const geometry_msgs::WrenchStamped &msg);

        double wrapToPi(double angle);
        Eigen::MatrixXd tMatrixToPosVector(Eigen::MatrixXd matrix_pos);

        // Move group
         ros::NodeHandle nhParams;
         ros::NodeHandle nhTopics;

         Eigen::MatrixXd lwa4p_blue_temp_q;

};
