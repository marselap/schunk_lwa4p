/******************************************************************************
File name: lwa4p_kinematics_Test.cpp
Description: Node for testing Schunk LWA4P kinematics!
Author: Bruno Maric
******************************************************************************/

#include <schunk_lwa4p_kinematics/gui_control.h>

using namespace std;

guiControl::guiControl(){

    std::string configFile;
    std::string path = ros::package::getPath("schunk_lwa4p_kinematics");

    ros::NodeHandle n;
    ros::NodeHandle private_node_handle_("~");

    //getting ros params
    private_node_handle_.param("config_file", configFile, path + std::string("/config/lwa4p_configuration.yaml"));

    //init robots kinematics
    lwa4p_blue.loadParameters(0, configFile);

    // init subscribers - ROBOT
    //lwa4pBlueJointStatesSub = n.subscribe("/blue_robot/joint_states", 10, &guiControl::lwa4pBlueJointStatesCallBack, this);
    // init subscribers - SIMULATOR
    lwa4pBlueJointStatesSub = n.subscribe("/lwa4p_blue/joint_states", 10, &guiControl::lwa4pBlueJointStatesCallBack, this);

    // init publishers
    lwa4pBlueDirectPositionResultPub = n.advertise<geometry_msgs::Vector3>("/lwa4p_blue/dk_pose_result", 1);
    lwa4pBlueDirectOrientationResultPub = n.advertise<geometry_msgs::Vector3>("/lwa4p_blue/dk_orientation_result", 1);

    // init subscribers - reference
    lwa4pBlueReferencePointSub = n.subscribe("/lwa4p_blue/reference_point", 10, &guiControl::lwa4pBlueReferencePointCallBack, this);

    // init subscribers - reference
    lwa4pBlueControlMode = n.subscribe("/lwa4p_blue/taylor_line", 10, &guiControl::lwa4pBlueControlModeCallBack, this);

    lwa4pBlueJoint1Pub = n.advertise<std_msgs::Float64>("/lwa4p_blue/arm_1_joint_pos_controller/command", 1);
    lwa4pBlueJoint2Pub = n.advertise<std_msgs::Float64>("/lwa4p_blue/arm_2_joint_pos_controller/command", 1);
    lwa4pBlueJoint3Pub = n.advertise<std_msgs::Float64>("/lwa4p_blue/arm_3_joint_pos_controller/command", 1);
    lwa4pBlueJoint4Pub = n.advertise<std_msgs::Float64>("/lwa4p_blue/arm_4_joint_pos_controller/command", 1);
    lwa4pBlueJoint5Pub = n.advertise<std_msgs::Float64>("/lwa4p_blue/arm_5_joint_pos_controller/command", 1);
    lwa4pBlueJoint6Pub = n.advertise<std_msgs::Float64>("/lwa4p_blue/arm_6_joint_pos_controller/command", 1);

    lwa4pBlueWaypointsPub = n.advertise<schunk_lwa4p_trajectory::WaypointArray>("/lwa4p_blue/waypoints", 1);

    lwa4p_blue_temp_q = Eigen::MatrixXd::Zero(6, 1);
    position_reference = Eigen::MatrixXd::Zero(9, 1);

    taylor_line = false; // false - point-to-point; true - line movement

    first_run = true;
}

guiControl::~guiControl(){

}

void guiControl::lwa4pBlueJointStatesCallBack(const sensor_msgs::JointState &msg){

    Eigen::MatrixXd dk_result;

    lwa4p_blue_temp_q = Eigen::MatrixXd::Zero(6, 1);

    lwa4p_blue_temp_q(0, 0) = msg.position[0];
    lwa4p_blue_temp_q(1, 0) = msg.position[1];
    lwa4p_blue_temp_q(2, 0) = msg.position[2];
    lwa4p_blue_temp_q(3, 0) = msg.position[3];
    lwa4p_blue_temp_q(4, 0) = msg.position[4];
    lwa4p_blue_temp_q(5, 0) = msg.position[5];
    if (first_run) {
        dk_result = lwa4p_blue.directKinematics(lwa4p_blue_temp_q, 6);
        position_reference(0, 0) = dk_result(0, 3);
        position_reference(1, 0) = dk_result(1, 3);
        position_reference(2, 0) = dk_result(2, 3);
        position_reference(3, 0) = dk_result(0, 0);
        position_reference(4, 0) = dk_result(1, 0);
        position_reference(5, 0) = dk_result(2, 0);
        position_reference(6, 0) = dk_result(0, 2);
        position_reference(7, 0) = dk_result(1, 2);
        position_reference(8, 0) = dk_result(2, 2);
        first_run = false;
    }
}

void guiControl::lwa4pBlueReferencePointCallBack(const std_msgs::Float64MultiArray &msg){

    position_reference = Eigen::MatrixXd::Zero(9, 1);

    if (msg.data.size() == 9)
        for(int i = 0; i < 9; i++){
            position_reference(i,0) = msg.data[i];
        }
    else
        std::cout << "Not enough input arguments. Expected vector size 9x1 [p^T x^T z^T]^T" << endl;
}

void guiControl::lwa4pBlueControlModeCallBack(const std_msgs::Bool &msg){
    taylor_line = msg.data;
    std::cout << "TAYLOR LINE " << taylor_line << endl;
}

schunk_lwa4p_trajectory::WaypointArray guiControl::makeWaypointsMsg(Eigen::MatrixXd waypoints)
{

    schunk_lwa4p_trajectory::WaypointArray returnValue;

    for (int i = 0; i < waypoints.cols(); i++){
        returnValue.waypoint_Q1.push_back(waypoints(0, i));
        returnValue.waypoint_Q2.push_back(waypoints(1, i));
        returnValue.waypoint_Q3.push_back(waypoints(2, i));
        returnValue.waypoint_Q4.push_back(waypoints(3, i));
        returnValue.waypoint_Q5.push_back(waypoints(4, i));
        returnValue.waypoint_Q6.push_back(waypoints(5, i));
    }

    return returnValue;

}

void guiControl::run(){

    ros::Rate r(20), r2(1);
    //test direct kinematics
    Eigen::MatrixXd joints_reference;

    while(ros::ok()){

        //cout << "Running!" << "\n";
        //after calling this function ROS will processes our callbacks
        ros::spinOnce();

        double temp_w[9], temp2[6], temp_q[6];
        Eigen::MatrixXd dk_result, dk_w_result, DK_result, IK_result;
        Eigen::MatrixXd ik_input;
        dk_w_result = Eigen::MatrixXd::Zero(9, 1);

        dk_result = lwa4p_blue.directKinematics(lwa4p_blue_temp_q, 6);
        dk_w_result(0, 0) = dk_result(0, 3);
        dk_w_result(1, 0) = dk_result(1, 3);
        dk_w_result(2, 0) = dk_result(2, 3);
        dk_w_result(3, 0) = dk_result(0, 0);
        dk_w_result(4, 0) = dk_result(1, 0);
        dk_w_result(5, 0) = dk_result(2, 0);
        dk_w_result(6, 0) = dk_result(0, 2);
        dk_w_result(7, 0) = dk_result(1, 2);
        dk_w_result(8, 0) = dk_result(2, 2);

        if (taylor_line == true) {

            std::cout << "Taylor process started" << endl;

            joints_reference = lwa4p_blue.taylorPathCheck(dk_w_result, position_reference, lwa4p_blue_temp_q, 1);

            /*
            std::cout << "=======================================" << endl;
            std::cout << "=======================================" << endl;
            std::cout << "=======================================" << endl;
            std::cout << "=======================================" << endl;
            */
            std::cout << "Current position: " << endl;
            std::cout << dk_w_result << endl;
            std::cout << "reference position: " << endl;
            std::cout << position_reference << endl;
            std::cout << "......................................." << endl;

            /*
            std::cout << "Current joint states: " << endl;
            std::cout << lwa4p_blue_temp_q << endl;

            std::cout << "......................................." << endl;
            std::cout << "Taylor result: " << endl;
            std::cout << "(" << joints_reference.rows() << ", " << joints_reference.cols() << ")" << endl;
            std::cout << "Taylor result START: " << endl;
            for(int i = 0; i < 6; i++) {
                std::cout << joints_reference(i, 0) << endl;
            }
            Eigen::MatrixXd taylor_end_q;
            taylor_end_q = Eigen::MatrixXd::Zero(6,1);
            std::cout << "Taylor result END: " << endl;
            for(int i = 0; i < 6; i++) {
                std::cout << joints_reference(i, joints_reference.cols()-1) << endl;
                taylor_end_q(i,0) = joints_reference(i, joints_reference.cols()-1);
            }

            std::cout << "......................................." << endl;
            std::cout << "Taylor end point: " << endl;
            dk_result = lwa4p_blue.directKinematics(taylor_end_q, 6);
            dk_w_result(0, 0) = dk_result(0, 3);
            dk_w_result(1, 0) = dk_result(1, 3);
            dk_w_result(2, 0) = dk_result(2, 3);
            dk_w_result(3, 0) = dk_result(0, 0);
            dk_w_result(4, 0) = dk_result(1, 0);
            dk_w_result(5, 0) = dk_result(2, 0);
            dk_w_result(6, 0) = dk_result(0, 2);
            dk_w_result(7, 0) = dk_result(1, 2);
            dk_w_result(8, 0) = dk_result(2, 2);
            std::cout << dk_w_result << endl;
            */

            //std::cout << joints_reference << endl;
        }
        else {

            if (first_run) {
                std::cout << "First run" << endl;
            }
            else {
                std::cout << "Point to point movement" << endl;

                Eigen::MatrixXd joints_reference_final;

                joints_reference_final = lwa4p_blue.inverseKinematics_closestQ(position_reference, lwa4p_blue_temp_q);
                //joints_reference = lwa4p_blue.taylorPathCheck(position_reference, position_reference, lwa4p_blue_temp_q, 10);
                std::cout << "Current position: " << endl;
                std::cout << dk_w_result << endl;

                std::cout << "Reference point: " << endl;
                std::cout << position_reference << endl;
                std::cout << "Reference joint states: " << endl;
                std::cout << joints_reference_final << endl;
                dk_result = lwa4p_blue.directKinematics(joints_reference_final, 6);
                //std::cout << dk_result << endl;
                dk_w_result(0, 0) = dk_result(0, 3);
                dk_w_result(1, 0) = dk_result(1, 3);
                dk_w_result(2, 0) = dk_result(2, 3);
                dk_w_result(3, 0) = dk_result(0, 0);
                dk_w_result(4, 0) = dk_result(1, 0);
                dk_w_result(5, 0) = dk_result(2, 0);
                dk_w_result(6, 0) = dk_result(0, 2);
                dk_w_result(7, 0) = dk_result(1, 2);
                dk_w_result(8, 0) = dk_result(2, 2);
                std::cout << "Dir(inv(reference point)): " << endl;
                std::cout << dk_w_result << endl;
                joints_reference = Eigen::MatrixXd::Zero(6,4);
                for (int j = 0; j < 4; j++) {
                    for (int i = 0; i < 6; i++) {
                        joints_reference(i, j) = 0.25 * (j + 1) * (joints_reference_final(i, 0) - lwa4p_blue_temp_q(i, 0)) + lwa4p_blue_temp_q(i, 0));
                    }
                }
                std::cout << "joints_reference" << std::endl;
                std::cout << joints_reference << std::endl;
            }

        }

        schunk_lwa4p_trajectory::WaypointArray msg2publish;
        msg2publish = makeWaypointsMsg(joints_reference);
        lwa4pBlueWaypointsPub.publish(msg2publish);
        std::cout << "Msg published!" << endl;

        int i;
        std::cin >> i;
        if (i == 0 )
            break;

        r.sleep();
    }

}
