/******************************************************************************
File name: current_position.h
Description: Node that publishes current robot position in global coordinate system
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

#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>

#include <schunk_lwa4p_kinematics/lwa4p_kinematics.h>

using namespace std;

class currentPosition{
    public:
        currentPosition();
        ~currentPosition();
        void run();

    private:
        ros::Publisher lwa4pBlueCurrentPositionPub;
        ros::Subscriber lwa4pBlueJointStatesSub;

        lwa4p_kinematics lwa4p_blue;

        void lwa4pBlueJointStatesCallBack(const sensor_msgs::JointState &msg);
        std_msgs::Float64MultiArray makePositionMsg(float* dk_w_result);
        // Move group
         ros::NodeHandle nhParams;
         ros::NodeHandle nhTopics;

         Eigen::MatrixXd lwa4p_blue_temp_q;

};
