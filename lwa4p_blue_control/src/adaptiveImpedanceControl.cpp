/******************************************************************************
File name: adaptiveImpedanceControl.cpp
Description: Node for Schunk LWA4P blue control!
Author: Bruno Maric
******************************************************************************/

#include <lwa4p_blue_control/adaptiveImpedanceControl.h>

using namespace std;

template<typename _Matrix_Type_>
_Matrix_Type_ pseudoInverse(const _Matrix_Type_ &a, double epsilon = std::numeric_limits<double>::epsilon())
{
    Eigen::JacobiSVD< _Matrix_Type_ > svd(a ,Eigen::ComputeThinU | Eigen::ComputeThinV);
    double tolerance = epsilon * std::max(a.cols(), a.rows()) *svd.singularValues().array().abs()(0);
    return svd.matrixV() *  (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal() * svd.matrixU().adjoint();
}

adaptiveImpedanceControl::adaptiveImpedanceControl(){

    std::string configFile;
    std::string path = ros::package::getPath("schunk_lwa4p_kinematics");

    ros::NodeHandle n;
    ros::NodeHandle private_node_handle_("~");

    //getting ros params
    private_node_handle_.param("config_file", configFile, path + std::string("/config/lwa4p_configuration.yaml"));

    operation_mode = 0;

    //init robots kinematics
    lwa4p_blue.loadParameters(0, configFile);

    // init subscribers - SIMULATOR
    lwa4pBlueJointStatesSub = n.subscribe("/lwa4p_blue/joint_states", 10, &adaptiveImpedanceControl::lwa4pBlueJointStatesCallBack, this);
    // init subscribers - ROBOT
    //lwa4pBlueJointStatesSub = n.subscribe("/joint_states", 10, &adaptiveImpedanceControl::lwa4pBlueJointStatesCallBack, this);

    lwa4pBlueOperationModeSub = n.subscribe("/lwa4p_blue/operation_mode", 10, &adaptiveImpedanceControl::lwa4pOperationModeCallBack, this);
    // FORCE SENSOR - SIMULATOR
    lwa4pBlueForceSensorSub = n.subscribe("/lwa4p_blue/ft_sensor_topic", 10, &adaptiveImpedanceControl::lwa4pBlueForceSensorCallBack, this);
    // FORCE SENSOR - ROBOT
    //lwa4pBlueForceSensorSub = n.subscribe("/force_sensor/filtered_ft_sensor", 10, &adaptiveImpedanceControl::lwa4pBlueForceSensorCallBack, this);

    lwa4pBlueForceReferenceSub = n.subscribe("/lwa4p_blue/force_reference", 2, &adaptiveImpedanceControl::lwa4pBlueForceReferenceCallBack, this);
    lwa4pBluePositionReferenceSub = n.subscribe("/lwa4p_blue/position_reference", 2, &adaptiveImpedanceControl::lwa4pBluePositionReferenceCallBack, this);
    lwa4pBlueOrientationXReferenceSub = n.subscribe("/lwa4p_blue/orientation_x", 2, &adaptiveImpedanceControl::lwa4pBlueOrientationXReferenceCallBack, this);
    lwa4pBlueOrientationZReferenceSub = n.subscribe("/lwa4p_blue/orientation_z", 2, &adaptiveImpedanceControl::lwa4pBlueOrientationZReferenceCallBack, this);

    robotBlueProfilePub = n.advertise<control_msgs::FollowJointTrajectoryActionGoal>("/pos_based_pos_traj_controller_arm/follow_joint_trajectory/goal", 1);
    //robotBlueProfilePub = n.advertise<trajectory_msgs::JointTrajectory>("/blue_robot/pos_based_pos_traj_controller_arm/command", 10);

    lwa4pBlueJoint1Pub = n.advertise<std_msgs::Float64>("/lwa4p_blue/arm_1_joint_pos_controller/command", 1);
    lwa4pBlueJoint2Pub = n.advertise<std_msgs::Float64>("/lwa4p_blue/arm_2_joint_pos_controller/command", 1);
    lwa4pBlueJoint3Pub = n.advertise<std_msgs::Float64>("/lwa4p_blue/arm_3_joint_pos_controller/command", 1);
    lwa4pBlueJoint4Pub = n.advertise<std_msgs::Float64>("/lwa4p_blue/arm_4_joint_pos_controller/command", 1);
    lwa4pBlueJoint5Pub = n.advertise<std_msgs::Float64>("/lwa4p_blue/arm_5_joint_pos_controller/command", 1);
    lwa4pBlueJoint6Pub = n.advertise<std_msgs::Float64>("/lwa4p_blue/arm_6_joint_pos_controller/command", 1);

    lwa4pBlueWaypointsPub = n.advertise<schunk_lwa4p_trajectory::WaypointArray>("/lwa4p_blue/waypoints", 1);

    lwa4pBluePositionReferencePub = n.advertise<std_msgs::Float64MultiArray>("/lwa4p_blue/dir_inv_xc", 1);

    lwa4pBlueXcPub = n.advertise<std_msgs::Float64MultiArray>("/lwa4p_blue/xc", 1);

    lwa4p_blue_temp_q = Eigen::MatrixXd::Zero(6, 1);
    force = Eigen::MatrixXd::Zero(3, 1);
    Fr = Eigen::MatrixXd::Zero(3, 1);
    Xref = Eigen::MatrixXd::Zero(3, 1);
    xOrientation = Eigen::MatrixXd::Zero(3, 1);
    zOrientation = Eigen::MatrixXd::Zero(3, 1);
    lwa4p_blue_prev_speed = Eigen::MatrixXd::Zero(6, 1);
}

adaptiveImpedanceControl::~adaptiveImpedanceControl(){

}

std_msgs::Float64MultiArray adaptiveImpedanceControl::makePositionMsg(Eigen::MatrixXd vector9x1) {

    std_msgs::Float64MultiArray msg;

    msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    msg.layout.dim[0].size = 9;

    for (int i = 0; i < 9; i++){
        msg.data.push_back(vector9x1(i,0));
    }

    return msg;
}

void adaptiveImpedanceControl::lwa4pOperationModeCallBack(const std_msgs::Int64 &msg){

    operation_mode = msg.data;
    if (operation_mode != 0) {
    }
}

void adaptiveImpedanceControl::lwa4pBlueForceSensorCallBack(const geometry_msgs::WrenchStamped &msg){

    force(0,0) = msg.wrench.force.x;
    force(1,0) = msg.wrench.force.y;
    //force(2,0) = 0.1*(msg.wrench.force.z-51.1) + 0.9*(force_z_mv);
    force(2,0) = 0.1*(-msg.wrench.force.z) + 0.9*(force(2,0));
    //force(2,0) = msg.wrench.force.z;

}

void adaptiveImpedanceControl::lwa4pBlueForceReferenceCallBack(const geometry_msgs::Vector3 &msg){

    // if new force reference - new_reference_flag for adaptation reset
    if ((msg.x != Fr(0, 0)) || (msg.y != Fr(1, 0)) || (msg.z != Fr(2, 0))) {
        new_reference_flag = true;
    }
    Fr(0, 0) = msg.x;
    Fr(1, 0) = msg.y;
    Fr(2, 0) = msg.z;
}

void adaptiveImpedanceControl::lwa4pBluePositionReferenceCallBack(const geometry_msgs::Vector3 &msg){

    // if new position reference - new_reference_flag for adaptation reset
    if ((msg.x != Xref(0, 0)) || (msg.y != Xref(1, 0)) || (msg.z != Xref(2, 0))) {
        new_reference_flag = true;
    }
    Xref(0, 0) = msg.x;
    Xref(1, 0) = msg.y;
    Xref(2, 0) = msg.z;
}


void adaptiveImpedanceControl::lwa4pBlueOrientationXReferenceCallBack(const geometry_msgs::Vector3 &msg){

    xOrientation(0, 0) = msg.x;
    xOrientation(1, 0) = msg.y;
    xOrientation(2, 0) = msg.z;
}

void adaptiveImpedanceControl::lwa4pBlueOrientationZReferenceCallBack(const geometry_msgs::Vector3 &msg){

    zOrientation(0, 0) = msg.x;
    zOrientation(1, 0) = msg.y;
    zOrientation(2, 0) = msg.z;
}

void adaptiveImpedanceControl::lwa4pBlueJointStatesCallBack(const sensor_msgs::JointState &msg){

    lwa4p_blue_temp_q = Eigen::MatrixXd::Zero(6, 1);

    lwa4p_blue_temp_q(0, 0) = msg.position[0];
    lwa4p_blue_temp_q(1, 0) = msg.position[1];
    lwa4p_blue_temp_q(2, 0) = msg.position[2];
    lwa4p_blue_temp_q(3, 0) = msg.position[3];
    lwa4p_blue_temp_q(4, 0) = msg.position[4];
    lwa4p_blue_temp_q(5, 0) = msg.position[5];
}

double adaptiveImpedanceControl::wrapToPi(double angle)
{
    return angle - floor(angle/(2*M_PI) + 0.5)*2*M_PI;
}

control_msgs::FollowJointTrajectoryActionGoal adaptiveImpedanceControl::createRobotTrajectoryMsg(Eigen::MatrixXd new_Q, int time_to_reach)
{

    control_msgs::FollowJointTrajectoryActionGoal return_value;

    std_msgs::Header h;
    h.stamp = ros::Time::now();
    return_value.goal.trajectory.joint_names.push_back("arm_1_joint");
    return_value.goal.trajectory.joint_names.push_back("arm_2_joint");
    return_value.goal.trajectory.joint_names.push_back("arm_3_joint");
    return_value.goal.trajectory.joint_names.push_back("arm_4_joint");
    return_value.goal.trajectory.joint_names.push_back("arm_5_joint");
    return_value.goal.trajectory.joint_names.push_back("arm_6_joint");
    std::ostringstream convert;
    convert << h.stamp.nsec;
    return_value.goal_id.id = "HoCook blue trajectory " + convert.str();
    std_msgs::Header h2, h_temp;
    ros::Time Tprep = ros::Time::now();
    h2.stamp.sec =  Tprep.sec ;
    h2.stamp.nsec =  Tprep.nsec ;
    return_value.goal.trajectory.header.stamp = h2.stamp; //kada da krene
    return_value.header.stamp = h2.stamp;

    double temp_time;
    double time_step = 1; //0.02; ///50.0;

    double new_time = 2;

    trajectory_msgs::JointTrajectoryPoint point_current;

    /*
    point_current.positions.push_back((double) lwa4p_blue_temp_q(0, 0) + (double) new_Q(0, 0));
    point_current.positions.push_back((double) lwa4p_blue_temp_q(1, 0) + (double) new_Q(1, 0));
    point_current.positions.push_back((double) lwa4p_blue_temp_q(2, 0) + (double) new_Q(2, 0));
    point_current.positions.push_back((double) lwa4p_blue_temp_q(3, 0) + (double) new_Q(3, 0));
    point_current.positions.push_back((double) lwa4p_blue_temp_q(4, 0) + (double) new_Q(4, 0));
    point_current.positions.push_back((double) lwa4p_blue_temp_q(5, 0) + (double) new_Q(5, 0));
    */
    point_current.positions.push_back( (double) new_Q(0, 0));
    point_current.positions.push_back( (double) new_Q(1, 0));
    point_current.positions.push_back( (double) new_Q(2, 0));
    point_current.positions.push_back( (double) new_Q(3, 0));
    point_current.positions.push_back( (double) new_Q(4, 0));
    point_current.positions.push_back( (double) new_Q(5, 0));

    double Td = 0.02;
    Eigen::MatrixXd lwa4p_blue_temp_speed;
    lwa4p_blue_temp_speed = Eigen::MatrixXd::Zero(6, 1);
    for(int i = 0; i < 6; i++) {
        lwa4p_blue_temp_speed(i, 0) = (new_Q(i, 0) - lwa4p_blue_temp_q(i, 0)) / Td;
    }

    point_current.velocities.push_back(lwa4p_blue_temp_speed(0, 0));
    point_current.velocities.push_back(lwa4p_blue_temp_speed(1, 0));
    point_current.velocities.push_back(lwa4p_blue_temp_speed(2, 0));
    point_current.velocities.push_back(lwa4p_blue_temp_speed(3, 0));
    point_current.velocities.push_back(lwa4p_blue_temp_speed(4, 0));
    point_current.velocities.push_back(lwa4p_blue_temp_speed(5, 0));

    /*
    point_current.accelerations.push_back((lwa4p_blue_temp_speed(0, 0) - lwa4p_blue_prev_speed(0, 0))/Td);
    point_current.accelerations.push_back((lwa4p_blue_temp_speed(1, 0) - lwa4p_blue_prev_speed(1, 0))/Td);
    point_current.accelerations.push_back((lwa4p_blue_temp_speed(2, 0) - lwa4p_blue_prev_speed(2, 0))/Td);
    point_current.accelerations.push_back((lwa4p_blue_temp_speed(3, 0) - lwa4p_blue_prev_speed(3, 0))/Td);
    point_current.accelerations.push_back((lwa4p_blue_temp_speed(4, 0) - lwa4p_blue_prev_speed(4, 0))/Td);
    point_current.accelerations.push_back((lwa4p_blue_temp_speed(5, 0) - lwa4p_blue_prev_speed(5, 0))/Td);
    */
    point_current.accelerations.push_back(lwa4p_blue_temp_speed(0, 0)/Td);
    point_current.accelerations.push_back(lwa4p_blue_temp_speed(1, 0)/Td);
    point_current.accelerations.push_back(lwa4p_blue_temp_speed(2, 0)/Td);
    point_current.accelerations.push_back(lwa4p_blue_temp_speed(3, 0)/Td);
    point_current.accelerations.push_back(lwa4p_blue_temp_speed(4, 0)/Td);
    point_current.accelerations.push_back(lwa4p_blue_temp_speed(5, 0)/Td);
    for(int i = 0; i < 6; i++) {
        lwa4p_blue_prev_speed(i, 0) = lwa4p_blue_temp_speed(i, 0);
    }

/*
    point_current.accelerations.push_back(0);
    point_current.accelerations.push_back(0);
    point_current.accelerations.push_back(0);
    point_current.accelerations.push_back(0);
    point_current.accelerations.push_back(0);
    point_current.accelerations.push_back(0);
*/
    new_time = new_time + time_step;
    point_current.time_from_start.nsec = (int) (0.15*pow(10, 9)); //(new_time - floor(new_time))*pow(10, 9); //(new_time % ceil(new_time))*pow(10, 9);
    point_current.time_from_start.sec = time_to_reach;
    return_value.goal.trajectory.points.push_back(point_current);

    return return_value;
};

schunk_lwa4p_trajectory::WaypointArray adaptiveImpedanceControl::makeWaypointsMsg(Eigen::MatrixXd waypoints)
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

Eigen::MatrixXd adaptiveImpedanceControl::tMatrixToPosVector(Eigen::MatrixXd matrix_pos) {
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

void adaptiveImpedanceControl::run(){

    ros::Rate r(50);

    Eigen::MatrixXd tool_vel, new_Q, position_reference;
    tool_vel = Eigen::MatrixXd::Zero(6, 1);
    position_reference = Eigen::MatrixXd::Zero(9,1);

    // Init impedance control
    double zeta_impedance = 20.0;
    double omega_impedance = 5.0;
    double M = 1 / (omega_impedance * omega_impedance); //0.01327;
    double b = 2.0*zeta_impedance/omega_impedance; //0.159;
    double K = 1.0;
    double Ts = 1.0/100.0;

    /* backward
    double a1 = (2*M + b*Ts)/(M + b*Ts + K*pow(Ts, 2));
    double a2 = -M/(M + b*Ts + K*pow(Ts, 2));
    double b1 = (K*pow(Ts, 2))/(M + b*Ts + K*pow(Ts, 2));
    double c1 = pow(Ts, 2)/(M + b*Ts + K*pow(Ts, 2)) ;
    */
    //TODO: NEW DISCRETIZATION
    double a_ = 1 / (4 * M / pow(Ts, 2) + 2 * b / Ts + K);
    double b_ = 8 * M / pow(Ts, 2) - 2 * K;
    double c_ = - 4 * M / pow(Ts, 2) + 2 * b / Ts - K;

    Eigen::MatrixXd Xr, Xc, E;
    // rows - dimensions; columns [t t-Td t-2Td]
    Xr = Eigen::MatrixXd::Zero(3, 3);
    Xc = Eigen::MatrixXd::Zero(3, 3);
    E = Eigen::MatrixXd::Zero(3, 3);

    int dim;
    for (dim = 0; dim < 3; dim++) {
        Xc(0, dim) = 500;
    }
    for (dim = 0; dim < 3; dim++) {
        Xc(1, dim) = 100;
    }
    for (dim = 0; dim < 3; dim++) {
        Xc(2, dim) = 580;
    }

    std_msgs::Float64 joint1, joint2, joint3, joint4, joint5, joint6;
    control_msgs::FollowJointTrajectoryActionGoal new_trajectory;


    int i;

    double amplitude = 0.0; // force amplitude
    double temp_result; // for force amplitude calculation
    Eigen::MatrixXd force_filtered;
    force_filtered = Eigen::MatrixXd::Zero(3, 1); // 3d vector - filtered sensor readings for each dimension
    Eigen::MatrixXd force_no_z_dc;
    force_no_z_dc = Eigen::MatrixXd::Zero(3, 1);

    double contact_force_threshold = 0.5;
    int operation_mode_prev = 0;

    dim = 0;
    forceController[dim].seta1(-1e-4); // 1e-3
    forceController[dim].seta2(0.0); // 0
    forceController[dim].setb1(1e-4); // 1e-4
    forceController[dim].setb2(1e-5); // 1e-5
    forceController[dim].setc1(1e-4); // 1e-5
    forceController[dim].setc2(1e-7); // 1e-9
    forceController[dim].setwp(10.0);
    forceController[dim].setModelZeta(4.0);
    forceController[dim].setModelOmega(3.0);
    dim++;
    forceController[dim].seta1(0.0); // 1e-3
    forceController[dim].seta2(0.0); // 1e-5
    forceController[dim].setb1(0.0); // 1e-3
    forceController[dim].setb2(0.0); // 1e-4
    forceController[dim].setc1(0.0); // 1e-6
    forceController[dim].setc2(0.0); // 1e-9
    forceController[dim].setwp(10.0);
    forceController[dim].setModelZeta(4.0);
    forceController[dim].setModelOmega(3.0);
    dim++;
    forceController[dim].seta1(-1e-3); // 1e-3
    forceController[dim].seta2(0.0); // 1e-5
    forceController[dim].setb1(1e-4); // 1e-4
    forceController[dim].setb2(1e-5); // 1e-5
    forceController[dim].setc1(1e-5); // 1e-5
    forceController[dim].setc2(1e-9); // 1e-9
    forceController[dim].setwp(10.0);
    forceController[dim].setModelZeta(4.0);
    forceController[dim].setModelOmega(3.0);
    /*
    forceController[dim].seta1(-1e-4); // -1e-4
    forceController[dim].seta2(0.0); // 0.0
    forceController[dim].setb1(1e-2); // 1e-2
    forceController[dim].setb2(1e-5); // 1e-5
    forceController[dim].setc1(2e-5); // 2e-5
    forceController[dim].setc2(1e-9); // 1e-9
    forceController[dim].setwp(5.0);
    forceController[dim].setModelZeta(2.0);
    forceController[dim].setModelOmega(5.0);
    */

    int counter = 0; // for debug output

    Eigen::MatrixXd dk_result, dk_w_result;
    dk_result = Eigen::MatrixXd::Zero(4,4);
    dk_w_result = Eigen::MatrixXd::Zero(9,1);

    while(ros::ok()){

        //after calling this function ROS will processes our callbacks
        ros::spinOnce();

        dk_result = lwa4p_blue.directKinematics(lwa4p_blue_temp_q, 6);
        dk_w_result = tMatrixToPosVector(dk_result);

        //TODO: DC filtering
        dim = 0;
        force_no_z_dc(dim, 0) = force(dim, 0);
        dim++;
        force_no_z_dc(dim, 0) = force(dim, 0);
        dim++;
        force_no_z_dc(dim, 0) = force(dim, 0);
        //force_no_z_dc(dim, 0) = force(dim, 0) - dk_w_result(8,0) * 0.19;

        temp_result = 0.0;
        for(dim = 0; dim < 3; dim++) {
            //TODO: DC FILTERING
            temp_result += pow(force_no_z_dc(dim,0), 2);
        }
        amplitude = 0.85 * amplitude + 0.15 * sqrt(temp_result);

        for(dim = 0; dim < 3; dim++) {
            //TODO: DC FILTERING
            force_filtered(dim, 0) = 0.85 * force_filtered(dim, 0) + 0.15 * force_no_z_dc(dim, 0);
        }

        Eigen::MatrixXd force_local, force_global;
        force_local = Eigen::MatrixXd::Zero(4,1); // 4x1 instead of 3x1 for easier multiplication w/ 4x4 transformation matrix dk_result
        for (dim = 0; dim < 3; dim++) {
             force_local(dim, 0) = force_filtered(dim, 0);
             force_local(dim, 0) = force_no_z_dc(dim, 0);
        }
        force_global = dk_result * force_local;

        /*TODO: remove this when test done. X axis control only */
        //force_global(0,0) = Fr(0,0);
        //force_global(1,0) = Fr(1,0);
        //force_global(2,0) = Fr(2,0);

        if (operation_mode != 0 )
        {
            if ((operation_mode_prev == 0) || (new_reference_flag)) {
                contact_force = false;
                for (dim = 0; dim < 3; dim++) {
                    forceController[dim].setf0(Xref(dim,0));
                    E(dim, 1) = Fr(dim, 0) - force_global(dim, 0);
                    E(dim, 2) = Fr(dim, 0) - force_global(dim, 0);
                    Xr(dim, 0) = Xref(dim, 0);
                    Xr(dim, 1) = Xref(dim, 0);
                    Xr(dim, 2) = Xref(dim, 0);
                    Xc(dim, 0) = dk_w_result(dim, 0);
                    Xc(dim, 1) = dk_w_result(dim, 0);
                    Xc(dim, 2) = dk_w_result(dim, 0);
                }
            }

            if (operation_mode_prev == 0) {
                for (dim = 0; dim < 3; dim++) {
                    forceController[dim].initController();
                }
            }

            if (new_reference_flag == true) {
                for (dim = 0; dim < 3; dim++) {
                    forceController[dim].resetController();
                }
                new_reference_flag = false;
            }

            if (fabs(force_global(0, 0)) > contact_force_threshold) {
                contact_force = true;
            }

            //TODO: ALL DIMS
            for (dim = 2; dim < 3; dim++) {
                    Xr(dim, 0) = forceController[dim].compute(Fr(dim, 0), force_global(dim, 0), contact_force);
                    E(dim, 0) = Fr(dim, 0) - force_global(dim, 0);

                    Xc(dim, 0) = (b_ * a_) * Xc(dim, 1) + (c_ * a_) * Xc(dim, 2) + a_ * (E(dim, 0) + 2 * E(dim, 1) + E(dim, 2) + Xr(dim, 0) + 2 * Xr(dim, 1) + Xr(dim, 2));
                    E(dim, 2) = E(dim, 1);
                    E(dim, 1) = E(dim, 0);
                    Xr(dim, 2) = Xr(dim, 1);
                    Xr(dim, 1) = Xr(dim, 0);
                    Xc(dim, 2) = Xc(dim, 1);
                    Xc(dim, 1) = Xc(dim, 0);
            }

            for(dim = 0; dim < 3; dim++) {
                position_reference(dim, 0) = Xc(dim, 0);
                position_reference(3 + dim, 0) = xOrientation(dim, 0);
                position_reference(6 + dim, 0) = zOrientation(dim, 0);
            }

            if (counter < 5000) {
                std::cout << "Xr = " << Xr(0,0) << " " << Xr(1,0) << " " << Xr(2,0) << std::endl;
                std::cout << "Xc = " << Xc(0,0) << " " << Xc(1,0) << " " << Xc(2,0) << std::endl;
            }

            new_Q = lwa4p_blue.inverseKinematics_closestQ(position_reference, lwa4p_blue_temp_q);

            joint1.data = new_Q(0, 0);
            joint2.data = new_Q(1, 0);
            joint3.data = new_Q(2, 0);
            joint4.data = new_Q(3, 0);
            joint5.data = new_Q(4, 0);
            joint6.data = new_Q(5, 0);

            lwa4pBlueJoint1Pub.publish(joint1);
            lwa4pBlueJoint2Pub.publish(joint2);
            lwa4pBlueJoint3Pub.publish(joint3);
            lwa4pBlueJoint4Pub.publish(joint4);
            lwa4pBlueJoint5Pub.publish(joint5);
            lwa4pBlueJoint6Pub.publish(joint6);

            //schunk_lwa4p_trajectory::WaypointArray msg2publish;
            //msg2publish = makeWaypointsMsg(new_Q);
            //lwa4pBlueWaypointsPub.publish(msg2publish);

            std_msgs::Float64MultiArray position_reference_msg;
            position_reference_msg = makePositionMsg(position_reference);
            lwa4pBlueXcPub.publish(position_reference_msg);

            new_trajectory = createRobotTrajectoryMsg(new_Q, 0);
            robotBlueProfilePub.publish(new_trajectory);

        }
        else
        {
        }
        operation_mode_prev = operation_mode;

        r.sleep();
    }

}
