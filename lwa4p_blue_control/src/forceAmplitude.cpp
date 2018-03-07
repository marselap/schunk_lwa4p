/******************************************************************************
File name: forceAmplitude.cpp
Description: Node for force sensor data filtering and analysis
Author: MP
******************************************************************************/

#include <lwa4p_blue_control/forceAmplitude.h>

using namespace std;

forceAmplitude::forceAmplitude(){

    std::string configFile;
    std::string path = ros::package::getPath("schunk_lwa4p_kinematics");

    ros::NodeHandle n;
    ros::NodeHandle private_node_handle_("~");

    //getting ros params
    private_node_handle_.param("config_file", configFile, path + std::string("/config/lwa4p_configuration.yaml"));

    //init robots kinematics
    lwa4p_blue.loadParameters(0, configFile);

    // init subscribers - SIMULATOR
    lwa4pBlueJointStatesSub = n.subscribe("/lwa4p_blue/joint_states", 10, &forceAmplitude::lwa4pBlueJointStatesCallBack, this);
    // init subscribers - ROBOT
    //lwa4pBlueJointStatesSub = n.subscribe("/blue_robot/joint_states", 10, &lwa4pBlueControl::lwa4pBlueJointStatesCallBack, this);

    // FORCE SENSOR - SIMULATOR
    lwa4pBlueForceSensorSub = n.subscribe("/lwa4p_blue/ft_sensor_topic", 2, &forceAmplitude::lwa4pBlueForceSensorCallBack, this);
    // FORCE SENSOR - ROBOT
    //lwa4pBlueForceSensorSub = n.subscribe("/optoforce_node/OptoForceWrench", 10, &forceAmplitude::lwa4pBlueForceSensorCallBack, this);

    // init publishers
    lwa4pBlueForceLocal = n.advertise<geometry_msgs::Vector3>("/lwa4p_blue/force_local", 1);
    lwa4pBlueForceGlobal = n.advertise<geometry_msgs::Vector3>("/lwa4p_blue/force_global", 1);

    lwa4p_blue_temp_q = Eigen::MatrixXd::Zero(6, 1);
    force_reading = Eigen::MatrixXd::Zero(3, 1);
}

forceAmplitude::~forceAmplitude(){
}

void forceAmplitude::lwa4pBlueForceSensorCallBack(const geometry_msgs::WrenchStamped &msg){

    force_reading(0,0) = msg.wrench.force.x;
    force_reading(1,0) = msg.wrench.force.y;
    force_reading(2,0) = -(msg.wrench.force.z);
}

void forceAmplitude::lwa4pBlueJointStatesCallBack(const sensor_msgs::JointState &msg){

    lwa4p_blue_temp_q = Eigen::MatrixXd::Zero(6, 1);

    lwa4p_blue_temp_q(0, 0) = msg.position[0];
    lwa4p_blue_temp_q(1, 0) = msg.position[1];
    lwa4p_blue_temp_q(2, 0) = msg.position[2];
    lwa4p_blue_temp_q(3, 0) = msg.position[3];
    lwa4p_blue_temp_q(4, 0) = msg.position[4];
    lwa4p_blue_temp_q(5, 0) = msg.position[5];
}

double forceAmplitude::wrapToPi(double angle)
{
    return angle - floor(angle/(2*M_PI) + 0.5)*2*M_PI;
}

Eigen::MatrixXd forceAmplitude::tMatrixToPosVector(Eigen::MatrixXd matrix_pos) {
    Eigen::MatrixXd vector_pos;
    vector_pos = Eigen::MatrixXd::Zero(9,1);
    int j = 0;
    vector_pos(j++,0) = matrix_pos(0, 3);
    vector_pos(j++,0) = matrix_pos(1, 3);
    vector_pos(j++,0) = matrix_pos(2, 3);
    vector_pos(j++,0) = matrix_pos(0, 0);
    vector_pos(j++,0) = matrix_pos(1, 0);
    vector_pos(j++,0) = matrix_pos(2, 0);
    vector_pos(j++,0) = matrix_pos(0, 2);
    vector_pos(j++,0) = matrix_pos(1, 2);
    vector_pos(j++,0) = matrix_pos(2, 2);
    return vector_pos;
}


void forceAmplitude::run(){

    ros::Rate r(100);

    Eigen::MatrixXd force, force_filtered;
    force = Eigen::MatrixXd::Zero(3, 1);
    force_filtered = Eigen::MatrixXd::Zero(3, 1);

    Eigen::MatrixXd force_local, force_global;
    force_local = Eigen::MatrixXd::Zero(4,1); // 4x1 instead of 3x1 for easier multiplication w/ 4x4 transformation matrix dk_result

    Eigen::MatrixXd dk_result, dk_w_result;

    double temp_result, amplitude;
    int dim;

    while(ros::ok()){

        //after calling this function ROS will processes our callbacks
        ros::spinOnce();

        dk_result = lwa4p_blue.directKinematics(lwa4p_blue_temp_q, 6);
        dk_w_result = tMatrixToPosVector(dk_result);

        //TODO:maybe remove; rather rewrite
        dim = 0;
        force(dim, 0) = force_reading(dim, 0);
        dim++;
        force(dim, 0) = force_reading(dim, 0);
        dim++;
        force(dim, 0) = force_reading(dim, 0) - dk_w_result(8,0) * 0.19;

        temp_result = 0.0;
        for(dim = 0; dim < 3; dim++) {
            temp_result += pow(force(dim,0), 2);
        }
        amplitude = 0.85 * amplitude + 0.15 * sqrt(temp_result);

        for(dim = 0; dim < 3; dim++) {
            force_filtered(dim, 0) = 0.85 * force_filtered(dim, 0) + 0.15 * force(dim, 0);
            force_local(dim, 0) = force_filtered(dim, 0);
        }

        force_global = dk_result * force_local;

        geometry_msgs::Vector3 force_local_publish, force_global_publish;
        dim = 0;
        force_local_publish.x  = force_local(dim, 0);
        force_global_publish.x  = force_global(dim, 0);
        dim++;
        force_local_publish.y  = force_local(dim, 0);
        force_global_publish.y  = force_global(dim, 0);
        dim++;
        force_local_publish.z  = force_local(dim, 0);
        force_global_publish.z  = force_global(dim, 0);

        lwa4pBlueForceLocal.publish(force_local_publish);
        lwa4pBlueForceGlobal.publish(force_global_publish);

        r.sleep();
    }

}
