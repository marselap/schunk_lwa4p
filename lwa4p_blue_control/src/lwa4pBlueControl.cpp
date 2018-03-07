/******************************************************************************
File name: lwa4pBlueControl.cpp
Description: Node for Schunk LWA4P blue control!
Author: Bruno Maric
******************************************************************************/

#include <lwa4p_blue_control/lwa4pBlueControl.h>

using namespace std;

template<typename _Matrix_Type_>
_Matrix_Type_ pseudoInverse(const _Matrix_Type_ &a, double epsilon = std::numeric_limits<double>::epsilon())
{
    Eigen::JacobiSVD< _Matrix_Type_ > svd(a ,Eigen::ComputeThinU | Eigen::ComputeThinV);
    double tolerance = epsilon * std::max(a.cols(), a.rows()) *svd.singularValues().array().abs()(0);
    return svd.matrixV() *  (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal() * svd.matrixU().adjoint();
}

lwa4pBlueControl::lwa4pBlueControl(){

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
	lwa4pBlueJointStatesSub = n.subscribe("/lwa4p_blue/joint_states", 10, &lwa4pBlueControl::lwa4pBlueJointStatesCallBack, this);
	//lwa4pRedJointStatesSub = n.subscribe("/lwa4p_red/joint_states", 10, &lwa4pRedControl::lwa4pRedJointStatesCallBack, this);
	// init subscribers - ROBOT
	//lwa4pBlueJointStatesSub = n.subscribe("/blue_robot/joint_states", 10, &lwa4pBlueControl::lwa4pBlueJointStatesCallBack, this);
	//lwa4pRedJointStatesSub = n.subscribe("/lwa4p_red/joint_states", 10, &lwa4pRedControl::lwa4pRedJointStatesCallBack, this);
	//processPointSub = n.subscribe("/lwa4p_red/process_point", 10, &lwa4pRedControl::processPointCallback, this);

	lwa4pBlueOperationModeSub = n.subscribe("/lwa4p_blue/operation_mode", 10, &lwa4pBlueControl::lwa4pOperationModeCallBack, this);
	// FORCE SENSOR - SIMULATOR
	lwa4pBlueForceSensorSub = n.subscribe("/lwa4p_blue/ft_sensor_topic", 10, &lwa4pBlueControl::lwa4pBlueForceSensorCallBack, this);
	// FORCE SENSOR - ROBOT
	//lwa4pBlueForceSensorSub = n.subscribe("/optoforce_node/OptoForceWrench", 10, &lwa4pBlueControl::lwa4pBlueForceSensorCallBack, this);


	//robotRedProfilePub = n.advertise<control_msgs::FollowJointTrajectoryActionGoal>("/red_robot/pos_based_pos_traj_controller_arm/follow_joint_trajectory/goal", 10);
	robotBlueProfilePub = n.advertise<control_msgs::FollowJointTrajectoryActionGoal>("/blue_robot/pos_based_pos_traj_controller_arm/follow_joint_trajectory/goal", 10);
    //robotBlueProfilePub = n.advertise<trajectory_msgs::JointTrajectory>("/blue_robot/pos_based_pos_traj_controller_arm/command", 10);



	lwa4pBlueJoint1Pub = n.advertise<std_msgs::Float64>("/lwa4p_blue/arm_1_joint_pos_controller/command", 1);
	lwa4pBlueJoint2Pub = n.advertise<std_msgs::Float64>("/lwa4p_blue/arm_2_joint_pos_controller/command", 1);
	lwa4pBlueJoint3Pub = n.advertise<std_msgs::Float64>("/lwa4p_blue/arm_3_joint_pos_controller/command", 1);
	lwa4pBlueJoint4Pub = n.advertise<std_msgs::Float64>("/lwa4p_blue/arm_4_joint_pos_controller/command", 1);
	lwa4pBlueJoint5Pub = n.advertise<std_msgs::Float64>("/lwa4p_blue/arm_5_joint_pos_controller/command", 1);
	lwa4pBlueJoint6Pub = n.advertise<std_msgs::Float64>("/lwa4p_blue/arm_6_joint_pos_controller/command", 1);

	//lwa4pRedWaypointsPub = n.advertise<schunk_lwa4p_trajectory::WaypointArray>("/lwa4p_red/waypoints", 1);
	lwa4p_blue_temp_q = Eigen::MatrixXd::Zero(6, 1);

/*
	// init publishers
	lwa4pBlueDirectPositionResultPub = n.advertise<geometry_msgs::Vector3>("/lwa4p_blue/dk_pose_result", 1);
	lwa4pBlueDirectOrientationResultPub = n.advertise<geometry_msgs::Vector3>("/lwa4p_blue/dk_orientation_result", 1);
	lwa4pRedDirectPositionResultPub = n.advertise<geometry_msgs::Vector3>("/lwa4p_red/dk_pose_result", 1);
	lwa4pRedDirectOrientationResultPub = n.advertise<geometry_msgs::Vector3>("/lwa4p_red/dk_orientation_result", 1);
*/

    //schunk_lwa4p_trajectory::WaypointArray nesto;
    //schunk_lwa4p_kinematics::lwa4p_kinematics nesto;
}

lwa4pBlueControl::~lwa4pBlueControl(){

}

Eigen::MatrixXd lwa4pBlueControl::calcJacobian(Eigen::MatrixXd temp_rob_q)
{
	//std::cout << "temp q = " << lwa4p_blue_temp_q(0, 0) << ", " << lwa4p_blue_temp_q(1, 0) << ", " << lwa4p_blue_temp_q(2, 0) << ", " << lwa4p_blue_temp_q(3, 0) << ", " << lwa4p_blue_temp_q(4, 0) << ", " << lwa4p_blue_temp_q(5, 0) << endl;


	Eigen::MatrixXd jacob, temp_q;
	jacob = Eigen::MatrixXd::Zero(6, 6);

	double l1, l2, l3, l4;
	l1 = 651;
	l2 = 350; //369.2;
	l3 = 305; //316;
	l4 = 85;

	temp_q = temp_rob_q;
	temp_q(2, 0) = -temp_q(2, 0) + 1.570796327;
	temp_q(1, 0) = temp_q(1, 0) + 1.570796327;

	jacob(0, 0) = l4*cos(temp_q(1, 0))*cos(temp_q(2, 0))*cos(temp_q(3, 0))*sin(temp_q(0, 0))*sin(temp_q(4, 0)) - l2*cos(temp_q(1, 0))*sin(temp_q(0, 0)) - l4*sin(temp_q(1, 0) + temp_q(2, 0))*cos(temp_q(4, 0))*sin(temp_q(0, 0)) - l4*cos(temp_q(0, 0))*sin(temp_q(3, 0))*sin(temp_q(4, 0)) - l3*sin(temp_q(1, 0) + temp_q(2, 0))*sin(temp_q(0, 0)) - l4*cos(temp_q(3, 0))*sin(temp_q(0, 0))*sin(temp_q(1, 0))*sin(temp_q(2, 0))*sin(temp_q(4, 0));
	jacob(0, 1) = cos(temp_q(0, 0))*(l3*cos(temp_q(1, 0) + temp_q(2, 0)) - l2*sin(temp_q(1, 0)) + l4*cos(temp_q(1, 0) + temp_q(2, 0))*cos(temp_q(4, 0)) + l4*sin(temp_q(1, 0) + temp_q(2, 0))*cos(temp_q(3, 0))*sin(temp_q(4, 0)));
	jacob(0, 2) = l3*cos(temp_q(1, 0) + temp_q(2, 0))*cos(temp_q(0, 0)) + l4*cos(temp_q(0, 0))*(cos(temp_q(1, 0) + temp_q(2, 0))*cos(temp_q(4, 0)) + sin(temp_q(1, 0) + temp_q(2, 0))*cos(temp_q(3, 0))*sin(temp_q(4, 0)));
	jacob(0, 3) = -l4*sin(temp_q(4, 0))*(cos(temp_q(3, 0))*sin(temp_q(0, 0)) - sin(temp_q(3, 0))*(cos(temp_q(0, 0))*cos(temp_q(1, 0))*cos(temp_q(2, 0)) - cos(temp_q(0, 0))*sin(temp_q(1, 0))*sin(temp_q(2, 0))));
	jacob(0, 4) = -l4*(cos(temp_q(4, 0))*(sin(temp_q(0, 0))*sin(temp_q(3, 0)) + cos(temp_q(3, 0))*(cos(temp_q(0, 0))*cos(temp_q(1, 0))*cos(temp_q(2, 0)) - cos(temp_q(0, 0))*sin(temp_q(1, 0))*sin(temp_q(2, 0)))) + sin(temp_q(4, 0))*(cos(temp_q(0, 0))*cos(temp_q(1, 0))*sin(temp_q(2, 0)) + cos(temp_q(0, 0))*cos(temp_q(2, 0))*sin(temp_q(1, 0))));
	jacob(0, 5) = 0;

	jacob(1, 0) = l3*sin(temp_q(1, 0) + temp_q(2, 0))*cos(temp_q(0, 0)) + l2*cos(temp_q(0, 0))*cos(temp_q(1, 0)) + l4*sin(temp_q(1, 0) + temp_q(2, 0))*cos(temp_q(0, 0))*cos(temp_q(4, 0)) - l4*sin(temp_q(0, 0))*sin(temp_q(3, 0))*sin(temp_q(4, 0)) - l4*cos(temp_q(0, 0))*cos(temp_q(1, 0))*cos(temp_q(2, 0))*cos(temp_q(3, 0))*sin(temp_q(4, 0)) + l4*cos(temp_q(0, 0))*cos(temp_q(3, 0))*sin(temp_q(1, 0))*sin(temp_q(2, 0))*sin(temp_q(4, 0));
	jacob(1, 1) = -l3*(sin(temp_q(0, 0))*sin(temp_q(1, 0))*sin(temp_q(2, 0)) - cos(temp_q(1, 0))*cos(temp_q(2, 0))*sin(temp_q(0, 0))) - l4*(cos(temp_q(4, 0))*(sin(temp_q(0, 0))*sin(temp_q(1, 0))*sin(temp_q(2, 0)) - cos(temp_q(1, 0))*cos(temp_q(2, 0))*sin(temp_q(0, 0))) - cos(temp_q(3, 0))*sin(temp_q(4, 0))*(cos(temp_q(1, 0))*sin(temp_q(0, 0))*sin(temp_q(2, 0)) + cos(temp_q(2, 0))*sin(temp_q(0, 0))*sin(temp_q(1, 0)))) - l2*sin(temp_q(0, 0))*sin(temp_q(1, 0));
	jacob(1, 2) = -l3*(sin(temp_q(0, 0))*sin(temp_q(1, 0))*sin(temp_q(2, 0)) - cos(temp_q(1, 0))*cos(temp_q(2, 0))*sin(temp_q(0, 0))) - l4*(cos(temp_q(4, 0))*(sin(temp_q(0, 0))*sin(temp_q(1, 0))*sin(temp_q(2, 0)) - cos(temp_q(1, 0))*cos(temp_q(2, 0))*sin(temp_q(0, 0))) - cos(temp_q(3, 0))*sin(temp_q(4, 0))*(cos(temp_q(1, 0))*sin(temp_q(0, 0))*sin(temp_q(2, 0)) + cos(temp_q(2, 0))*sin(temp_q(0, 0))*sin(temp_q(1, 0))));
	jacob(1, 3) = l4*sin(temp_q(4, 0))*(cos(temp_q(0, 0))*cos(temp_q(3, 0)) - sin(temp_q(3, 0))*(sin(temp_q(0, 0))*sin(temp_q(1, 0))*sin(temp_q(2, 0)) - cos(temp_q(1, 0))*cos(temp_q(2, 0))*sin(temp_q(0, 0))));
	jacob(1, 4) = l4*(cos(temp_q(4, 0))*(cos(temp_q(0, 0))*sin(temp_q(3, 0)) + cos(temp_q(3, 0))*(sin(temp_q(0, 0))*sin(temp_q(1, 0))*sin(temp_q(2, 0)) - cos(temp_q(1, 0))*cos(temp_q(2, 0))*sin(temp_q(0, 0)))) - sin(temp_q(4, 0))*(cos(temp_q(1, 0))*sin(temp_q(0, 0))*sin(temp_q(2, 0)) + cos(temp_q(2, 0))*sin(temp_q(0, 0))*sin(temp_q(1, 0))));
	jacob(1, 5) = 0;

	jacob(2, 0) = 0;
	jacob(2, 1) = l3*sin(temp_q(1, 0) + temp_q(2, 0)) + l2*cos(temp_q(1, 0)) - (l4*cos(temp_q(1, 0) + temp_q(2, 0))*sin(temp_q(3, 0) + temp_q(4, 0)))/2 + l4*sin(temp_q(1, 0) + temp_q(2, 0))*cos(temp_q(4, 0)) + (l4*sin(temp_q(3, 0) - temp_q(4, 0))*cos(temp_q(1, 0) + temp_q(2, 0)))/2;
	jacob(2, 2) = l3*sin(temp_q(1, 0) + temp_q(2, 0)) + l4*sin(temp_q(1, 0) + temp_q(2, 0))*cos(temp_q(4, 0)) - l4*cos(temp_q(1, 0) + temp_q(2, 0))*cos(temp_q(3, 0))*sin(temp_q(4, 0));
	jacob(2, 3) = l4*sin(temp_q(1, 0) + temp_q(2, 0))*sin(temp_q(3, 0))*sin(temp_q(4, 0));
	jacob(2, 4) = l4*(cos(temp_q(1, 0) + temp_q(2, 0))*sin(temp_q(4, 0)) - sin(temp_q(1, 0) + temp_q(2, 0))*cos(temp_q(3, 0))*cos(temp_q(4, 0)));
	jacob(2, 5) = 0;

	jacob(3, 0) = -sin(temp_q(4, 0))*(cos(temp_q(0, 0))*sin(temp_q(3, 0)) + cos(temp_q(3, 0))*(sin(temp_q(0, 0))*sin(temp_q(1, 0))*sin(temp_q(2, 0)) - cos(temp_q(1, 0))*cos(temp_q(2, 0))*sin(temp_q(0, 0)))) - cos(temp_q(4, 0))*(cos(temp_q(1, 0))*sin(temp_q(0, 0))*sin(temp_q(2, 0)) + cos(temp_q(2, 0))*sin(temp_q(0, 0))*sin(temp_q(1, 0)));
	jacob(3, 1) = cos(temp_q(0, 0))*(cos(temp_q(1, 0) + temp_q(2, 0))*cos(temp_q(4, 0)) + sin(temp_q(1, 0) + temp_q(2, 0))*cos(temp_q(3, 0))*sin(temp_q(4, 0)));
	jacob(3, 2) = cos(temp_q(0, 0))*(cos(temp_q(1, 0) + temp_q(2, 0))*cos(temp_q(4, 0)) + sin(temp_q(1, 0) + temp_q(2, 0))*cos(temp_q(3, 0))*sin(temp_q(4, 0)));
	jacob(3, 3) = -sin(temp_q(4, 0))*(cos(temp_q(3, 0))*sin(temp_q(0, 0)) - sin(temp_q(3, 0))*(cos(temp_q(0, 0))*cos(temp_q(1, 0))*cos(temp_q(2, 0)) - cos(temp_q(0, 0))*sin(temp_q(1, 0))*sin(temp_q(2, 0))));
	jacob(3, 4) = -cos(temp_q(4, 0))*(sin(temp_q(0, 0))*sin(temp_q(3, 0)) + cos(temp_q(3, 0))*(cos(temp_q(0, 0))*cos(temp_q(1, 0))*cos(temp_q(2, 0)) - cos(temp_q(0, 0))*sin(temp_q(1, 0))*sin(temp_q(2, 0)))) - sin(temp_q(4, 0))*(cos(temp_q(0, 0))*cos(temp_q(1, 0))*sin(temp_q(2, 0)) + cos(temp_q(0, 0))*cos(temp_q(2, 0))*sin(temp_q(1, 0)));
	jacob(3, 5) = 0;

	jacob(4, 0) = cos(temp_q(4, 0))*(cos(temp_q(0, 0))*cos(temp_q(1, 0))*sin(temp_q(2, 0)) + cos(temp_q(0, 0))*cos(temp_q(2, 0))*sin(temp_q(1, 0))) - sin(temp_q(4, 0))*(sin(temp_q(0, 0))*sin(temp_q(3, 0)) + cos(temp_q(3, 0))*(cos(temp_q(0, 0))*cos(temp_q(1, 0))*cos(temp_q(2, 0)) - cos(temp_q(0, 0))*sin(temp_q(1, 0))*sin(temp_q(2, 0))));
	jacob(4, 1) = sin(temp_q(0, 0))*(cos(temp_q(1, 0) + temp_q(2, 0))*cos(temp_q(4, 0)) + sin(temp_q(1, 0) + temp_q(2, 0))*cos(temp_q(3, 0))*sin(temp_q(4, 0)));
	jacob(4, 2) = sin(temp_q(0, 0))*(cos(temp_q(1, 0) + temp_q(2, 0))*cos(temp_q(4, 0)) + sin(temp_q(1, 0) + temp_q(2, 0))*cos(temp_q(3, 0))*sin(temp_q(4, 0)));
	jacob(4, 3) = sin(temp_q(4, 0))*(cos(temp_q(0, 0))*cos(temp_q(3, 0)) - sin(temp_q(3, 0))*(sin(temp_q(0, 0))*sin(temp_q(1, 0))*sin(temp_q(2, 0)) - cos(temp_q(1, 0))*cos(temp_q(2, 0))*sin(temp_q(0, 0))));
	jacob(4, 4) = cos(temp_q(4, 0))*(cos(temp_q(0, 0))*sin(temp_q(3, 0)) + cos(temp_q(3, 0))*(sin(temp_q(0, 0))*sin(temp_q(1, 0))*sin(temp_q(2, 0)) - cos(temp_q(1, 0))*cos(temp_q(2, 0))*sin(temp_q(0, 0)))) - sin(temp_q(4, 0))*(cos(temp_q(1, 0))*sin(temp_q(0, 0))*sin(temp_q(2, 0)) + cos(temp_q(2, 0))*sin(temp_q(0, 0))*sin(temp_q(1, 0)));
	jacob(4, 5) = 0;

	jacob(5, 0) = 0;
	jacob(5, 1) = sin(temp_q(1, 0) + temp_q(2, 0))*cos(temp_q(4, 0)) - cos(temp_q(1, 0) + temp_q(2, 0))*cos(temp_q(3, 0))*sin(temp_q(4, 0));
	jacob(5, 2) = sin(temp_q(1, 0) + temp_q(2, 0))*cos(temp_q(4, 0)) - cos(temp_q(1, 0) + temp_q(2, 0))*cos(temp_q(3, 0))*sin(temp_q(4, 0));
	jacob(5, 3) = sin(temp_q(1, 0) + temp_q(2, 0))*sin(temp_q(3, 0))*sin(temp_q(4, 0));
	jacob(5, 4) = cos(temp_q(1, 0) + temp_q(2, 0))*sin(temp_q(4, 0)) - sin(temp_q(1, 0) + temp_q(2, 0))*cos(temp_q(3, 0))*cos(temp_q(4, 0));
	jacob(5, 5) = 0;



	return jacob;

}




void lwa4pBlueControl::lwa4pOperationModeCallBack(const std_msgs::Int64 &msg){


	operation_mode = msg.data;

}



void lwa4pBlueControl::lwa4pBlueForceSensorCallBack(const geometry_msgs::WrenchStamped &msg){


	force_x_mv = msg.wrench.force.x;
	force_y_mv = msg.wrench.force.y;
	//force_z_mv = 0.1*(msg.wrench.force.z-51.1) + 0.9*(force_z_mv);
	force_z_mv = 0.1*(-msg.wrench.force.z) + 0.9*(force_z_mv);

	//std::cout << "Force mv: " << force_x_mv << ", " << force_y_mv << ", " << force_z_mv << endl;

}



void lwa4pBlueControl::lwa4pBlueJointStatesCallBack(const sensor_msgs::JointState &msg){

	lwa4p_blue_temp_q = Eigen::MatrixXd::Zero(6, 1);

	lwa4p_blue_temp_q(0, 0) = msg.position[0];
	lwa4p_blue_temp_q(1, 0) = msg.position[1];
	lwa4p_blue_temp_q(2, 0) = msg.position[2];
	lwa4p_blue_temp_q(3, 0) = msg.position[3];
	lwa4p_blue_temp_q(4, 0) = msg.position[4];
	lwa4p_blue_temp_q(5, 0) = msg.position[5];
}

double lwa4pBlueControl::wrapToPi(double angle)
{

	return angle - floor(angle/(2*M_PI) + 0.5)*2*M_PI;


}

Eigen::MatrixXd lwa4pBlueControl::lineMovement(Eigen::MatrixXd delta_x, Eigen::MatrixXd temp_q)
{


	Eigen::MatrixXd return_val;
	Eigen::MatrixXd jacob, jacob_inv, dq;

	jacob = calcJacobian(lwa4p_blue_temp_q);
	//jacob_inv = jacob*(jacob*jacob.transpose()).inverse();
	jacob_inv = pseudoInverse(jacob);



	dq = jacob_inv*delta_x;

//	std::cout << jacob << endl;
//	std::cout << jacob_inv << endl;
//	std::cout << dq << endl;
	std::cout << "New angles: " << endl;
	std::cout << wrapToPi(dq(0, 0)) << endl;
	std::cout << wrapToPi(dq(1, 0)) << endl;
	std::cout << wrapToPi(dq(2, 0)) << endl;
	std::cout << wrapToPi(dq(3, 0)) << endl;
	std::cout << wrapToPi(dq(4, 0)) << endl;
	std::cout << wrapToPi(dq(5, 0)) << endl;



	return_val = Eigen::MatrixXd::Zero(6, 1);
	return_val(0, 0) = wrapToPi(temp_q(0, 0) + dq(0, 0));
	return_val(1, 0) = wrapToPi(temp_q(1, 0) + dq(1, 0));
	return_val(2, 0) = wrapToPi(temp_q(2, 0) - dq(2, 0));
	return_val(3, 0) = wrapToPi(temp_q(3, 0) + dq(3, 0));
	return_val(4, 0) = wrapToPi(temp_q(4, 0) + dq(4, 0));
	return_val(5, 0) = wrapToPi(temp_q(5, 0) + dq(5, 0));
    /*
	return_val(0, 0) = wrapToPi(  dq(0, 0));
	return_val(1, 0) = wrapToPi(  dq(1, 0));
	return_val(2, 0) = wrapToPi( -dq(2, 0)); //INVERTIRATI?
	return_val(3, 0) = wrapToPi(  dq(3, 0));
	return_val(4, 0) = wrapToPi(  dq(4, 0));
	return_val(5, 0) = wrapToPi(  dq(5, 0));
    */



	return return_val;


}

//trajectory_msgs::JointTrajectory lwa4pBlueControl::createRobotTrajectoryMsg( Eigen::MatrixXd new_Q )
control_msgs::FollowJointTrajectoryActionGoal lwa4pBlueControl::createRobotTrajectoryMsg( Eigen::MatrixXd new_Q, int time_to_reach )
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
	  //traj_vector_prep.goal_id.stamp = h.stamp;
	std::ostringstream convert;
	convert << h.stamp.nsec;
	return_value.goal_id.id = "HoCook blue trajectory " + convert.str();
	std_msgs::Header h2, h_temp;
	ros::Time Tprep = ros::Time::now();
	h2.stamp.sec =  Tprep.sec ;
	h2.stamp.nsec =  Tprep.nsec ;
	return_value.goal.trajectory.header.stamp = h2.stamp; //kada da krene
	return_value.header.stamp = h2.stamp;

	//std::cout << sampledTrajectory.pose_joint_1.size() << endl;

	double temp_time;
	double time_step = 1; //0.02; ///50.0;

	double new_time = 2;

	//new_time = h2.stamp.sec + h2.stamp.nsec*pow(10, -9);
	//new_time = time_step;


	trajectory_msgs::JointTrajectoryPoint point_current;

	point_current.positions.push_back((double) lwa4p_blue_temp_q(0, 0) + (double) new_Q(0, 0));
	point_current.positions.push_back((double) lwa4p_blue_temp_q(1, 0) + (double) new_Q(1, 0));
	point_current.positions.push_back((double) lwa4p_blue_temp_q(2, 0) + (double) new_Q(2, 0));
	point_current.positions.push_back((double) lwa4p_blue_temp_q(3, 0) + (double) new_Q(3, 0));
	point_current.positions.push_back((double) lwa4p_blue_temp_q(4, 0) + (double) new_Q(4, 0));
	point_current.positions.push_back((double) lwa4p_blue_temp_q(5, 0) + (double) new_Q(5, 0));
/*
	point_current.velocities.push_back(new_Q(0, 0));
	point_current.velocities.push_back(new_Q(1, 0));
	point_current.velocities.push_back(new_Q(2, 0));
	point_current.velocities.push_back(new_Q(3, 0));
	point_current.velocities.push_back(new_Q(4, 0));
	point_current.velocities.push_back(new_Q(5, 0));

	point_current.accelerations.push_back(0);
	point_current.accelerations.push_back(0);
	point_current.accelerations.push_back(0);
	point_current.accelerations.push_back(0);
	point_current.accelerations.push_back(0);
	point_current.accelerations.push_back(0);
*/
	new_time = new_time + time_step;

	//point_current.time_from_start.nsec = h2.stamp.nsec + ((i+1) * (1/50))*pow(10, 9);
	//point_current.time_from_start.nsec = (int) (((double) 1/((double) 30))*pow(10, 9)); //(new_time - floor(new_time))*pow(10, 9); //(new_time % ceil(new_time))*pow(10, 9);
	point_current.time_from_start.nsec = (int) (0.15*pow(10, 9)); //(new_time - floor(new_time))*pow(10, 9); //(new_time % ceil(new_time))*pow(10, 9);

	point_current.time_from_start.sec = time_to_reach;

	//std::cout << point_current.time_from_start.sec<<", "<< (new_time - floor(new_time))*pow(10, 9) <<", " <<h2.stamp<<", "<<h2.stamp.sec<<", "<<h2.stamp.nsec<<", " << endl;

	return_value.goal.trajectory.points.push_back(point_current);


	//std::cout << "Pocetno "<< h_temp.stamp.sec<<", "<<h_temp.stamp.nsec << endl;
	//std::cout << h2.stamp.sec<<", "<<h2.stamp.nsec << endl;
	//std::cout << return_value << endl;


	return return_value;

	/*

	trajectory_msgs::JointTrajectory return_value2;

	return_value2.joint_names.push_back("arm_1_joint");
	return_value2.joint_names.push_back("arm_2_joint");
	return_value2.joint_names.push_back("arm_3_joint");
	return_value2.joint_names.push_back("arm_4_joint");
	return_value2.joint_names.push_back("arm_5_joint");
	return_value2.joint_names.push_back("arm_6_joint");

	std_msgs::Header h;
	ros::Time Tprep = ros::Time::now();
	h.stamp.sec = 0; //(int) Tprep.sec;
	h.stamp.nsec = 0; //(int) Tprep.nsec ;
	return_value2.header.stamp = h.stamp;

	double temp_time;
	double time_step = 1; //0.02; ///50.0;

	double new_time = 2;


	trajectory_msgs::JointTrajectoryPoint point_current;

	point_current.positions.push_back((double) lwa4p_blue_temp_q(0, 0) + (double) new_Q(0, 0)*(0.01));
	point_current.positions.push_back((double) lwa4p_blue_temp_q(1, 0) + (double) new_Q(1, 0)*(0.01));
	point_current.positions.push_back((double) lwa4p_blue_temp_q(2, 0) + (double) new_Q(2, 0)*(0.01));
	point_current.positions.push_back((double) lwa4p_blue_temp_q(3, 0) + (double) new_Q(3, 0)*(0.01));
	point_current.positions.push_back((double) lwa4p_blue_temp_q(4, 0) + (double) new_Q(4, 0)*(0.01));
	point_current.positions.push_back((double) lwa4p_blue_temp_q(5, 0) + (double) new_Q(5, 0)*(0.01));

	point_current.velocities.push_back((double) new_Q(0, 0));
	point_current.velocities.push_back((double) new_Q(1, 0));
	point_current.velocities.push_back((double) new_Q(2, 0));
	point_current.velocities.push_back((double) new_Q(3, 0));
	point_current.velocities.push_back((double) new_Q(4, 0));
	point_current.velocities.push_back((double) new_Q(5, 0));

	point_current.accelerations.push_back(0);
	point_current.accelerations.push_back(0);
	point_current.accelerations.push_back(0);
	point_current.accelerations.push_back(0);
	point_current.accelerations.push_back(0);
	point_current.accelerations.push_back(0);

	new_time = new_time + time_step;

	//point_current.time_from_start.nsec = h2.stamp.nsec + ((i+1) * (1/50))*pow(10, 9);
	//point_current.time_from_start.nsec = (int) (((double) 1/((double) 30))*pow(10, 9)); //(new_time - floor(new_time))*pow(10, 9); //(new_time % ceil(new_time))*pow(10, 9);
	point_current.time_from_start.nsec = (int) (((double) 1/((double) 100))*pow(10, 9)); //(new_time - floor(new_time))*pow(10, 9); //(new_time % ceil(new_time))*pow(10, 9);

	point_current.time_from_start.sec = (int) 0;

	//std::cout << point_current.time_from_start.sec<<", "<< (new_time - floor(new_time))*pow(10, 9) <<", " <<h2.stamp<<", "<<h2.stamp.sec<<", "<<h2.stamp.nsec<<", " << endl;

	return_value2.points.push_back(point_current);

	std::cout << return_value2 << endl;

	return return_value2;

	*/

};





void lwa4pBlueControl::run(){

	ros::Rate r(100), r2(1);

	//test direct kinematics
	double temp_q[6];
	temp_q[0] = 0;
	temp_q[1] = 0;
	temp_q[2] = 0;
	temp_q[3] = 0;
	temp_q[4] = 0;
	temp_q[5] = 0;

	Eigen::MatrixXd tool_vel, new_Q;
	tool_vel = Eigen::MatrixXd::Zero(6, 1);
	new_Q = Eigen::MatrixXd::Zero(6, 1);

	// Set PID
	forceCtrlPID.setKp(1.1);
	forceCtrlPID.setKd(0.0);
	forceCtrlPID.setKi(0);

	//forceCtrlPID.setUMax(1);
	//forceCtrlPID.setUMin(-1);


	double PID_result;
	int go_back = 0;


	Eigen::MatrixXd pointSpace, w1;

	pointSpace = Eigen::MatrixXd::Zero(4, 4);
	pointSpace(0, 3) = -400;
	pointSpace(1, 3) = 0; //0.029269;
	pointSpace(2, 3) = 1000;

	pointSpace(0, 0) = 0;
	pointSpace(1, 0) = 1;
	pointSpace(2, 0) = 0;

	pointSpace(0, 1) = 0;
	pointSpace(1, 1) = 0;
	pointSpace(2, 1) = -1;

	pointSpace(0, 2) = -1;
	pointSpace(1, 2) = 0;
	pointSpace(2, 2) = 0;

	w1 = Eigen::MatrixXd::Zero(9, 1);
	w1(0, 0) = pointSpace(0, 3);
	w1(1, 0) = pointSpace(1, 3);
	w1(2, 0) = pointSpace(2, 3);
	w1(3, 0) = pointSpace(0, 0);
	w1(4, 0) = pointSpace(1, 0);
	w1(5, 0) = pointSpace(2, 0);
	w1(6, 0) = pointSpace(0, 2);
	w1(7, 0) = pointSpace(1, 2);
	w1(8, 0) = pointSpace(2, 2);


	// Init impedance control
	int k = 0;
	double M = 0.003218; //0.01327;
	double b = 0.02964; //0.159;
	double K = 1;
	double Ts = 1.0/100.0;

	double Xr = 0;
	double Xc = 0;
	double Xc_1 = 0;
	double Xc_2 = 0;
	double E = 0;

	double a1 = (2*M + b*Ts)/(M + b*Ts + K*pow(Ts, 2));
	double a2 = -M/(M + b*Ts + K*pow(Ts, 2));
	double b1 = (K*pow(Ts, 2))/(M + b*Ts + K*pow(Ts, 2));
	double c1 = pow(Ts, 2)/(M + b*Ts + K*pow(Ts, 2));


	double Fr = 5;


	while(ros::ok()){

		//after calling this function ROS will processes our callbacks
        ros::spinOnce();

		//cout << "Running!" << "\n";

		Eigen::MatrixXd deltaQ;

		std_msgs::Float64 joint1, joint2, joint3, joint4, joint5, joint6;



		control_msgs::FollowJointTrajectoryActionGoal new_trajectory;

		if (operation_mode != 0 )
		{
			if (abs(operation_mode) < 10)
				Fr = operation_mode;
			else
				Fr = 0;

			// control error
			E = Fr - force_z_mv;

			// PI control
			Xr = forceCtrlPID.compute(Fr, force_z_mv);

			//impedance filter
			Xc = (a1*Xc_1 + a2*Xc_2 + b1*Xr + c1*E)/1.5;

			// for next step
			Xc_2 = Xc_1;
			Xc_1 = Xc;

			std::cout << "E = " << E << " Fr = " << Fr << endl;
			std::cout << "Xr = " << Xr << ", Xc = " << Xc << endl;
			std::cout << "Xc = " << Xc << ", Xc_1 = " << Xc_1 << ", Xc_2 = " << Xc_2 << endl;

            /*
            Xr = adaptiveCtrl.compute(Fr, force_reading);
            for (i = 0; i < 3; i++) {
                E(i) = Fr(i) - force_reading(i);
            }
            //impedance filter
            Xc = (a1*Xc_1 + a2*Xc_2 + b1*Xr + c1*E)/1.5;
            // for next step
            Xc_2 = Xc_1;
            Xc_1 = Xc;


            for (i = 0; i < 3; i++) {
                tool_vel(i, 0) = Xc;
            }

            */

			tool_vel(0, 0) = Xc;
			new_Q = lineMovement(tool_vel, lwa4p_blue_temp_q);

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

			new_trajectory = createRobotTrajectoryMsg(new_Q, 0);
			//std::cout << new_trajectory << endl;
			//robotRedProfilePub.publish(new_trajectory);
			robotBlueProfilePub.publish(new_trajectory);

		}
		else
		{

				/*
				joint1.data = 0;
				joint2.data = -1.47148;
				joint3.data = -1.32021;
				joint4.data = 0;
				joint5.data = 1.41953;
				joint6.data = -1.5708;

				lwa4pBlueJoint1Pub.publish(joint1);
				lwa4pBlueJoint2Pub.publish(joint2);
				lwa4pBlueJoint3Pub.publish(joint3);
				lwa4pBlueJoint4Pub.publish(joint4);
				lwa4pBlueJoint5Pub.publish(joint5);
				lwa4pBlueJoint6Pub.publish(joint6);
				*/

		}

		r.sleep();
	}




}
