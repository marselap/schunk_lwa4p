/******************************************************************************
File name: lwa4p_kinematics_Test.cpp
Description: Node for testing Schunk LWA4P kinematics! 
Author: Bruno Maric
******************************************************************************/

#include <schunk_lwa4p_kinematics/lwa4p_kinematics_test.h>

using namespace std;

kinematicsTest::kinematicsTest(){

	std::string configFile;
	std::string path = ros::package::getPath("schunk_lwa4p_kinematics");

	ros::NodeHandle n;
	ros::NodeHandle private_node_handle_("~");

	//getting ros params
    private_node_handle_.param("config_file", configFile, path + std::string("/config/lwa4p_configuration.yaml"));



	//init robots kinematics
	lwa4p_blue.loadParameters(0, configFile);
	lwa4p_red.loadParameters(1, configFile);


	// init subscribers - ROBOT
	lwa4pBlueJointStatesSub = n.subscribe("/blue_robot/joint_states", 10, &kinematicsTest::lwa4pBlueJointStatesCallBack, this);
	lwa4pRedJointStatesSub = n.subscribe("/red_robot/joint_states", 10, &kinematicsTest::lwa4pRedJointStatesCallBack, this);
	// init subscribers - SIMULATOR
	//lwa4pBlueJointStatesSub = n.subscribe("/lwa4p_blue/joint_states", 10, &kinematicsTest::lwa4pBlueJointStatesCallBack, this);
	//lwa4pRedJointStatesSub = n.subscribe("/lwa4p_red/joint_states", 10, &kinematicsTest::lwa4pRedJointStatesCallBack, this);
	alphaSub = n.subscribe("/alpha", 10, &kinematicsTest::alphaCallBack, this);
	betaSub = n.subscribe("/beta", 10, &kinematicsTest::betaCallBack, this);

	// init publishers
	lwa4pBlueDirectPositionResultPub = n.advertise<geometry_msgs::Vector3>("/lwa4p_blue/dk_pose_result", 1);
	lwa4pBlueDirectOrientationResultPub = n.advertise<geometry_msgs::Vector3>("/lwa4p_blue/dk_orientation_result", 1);
	lwa4pRedDirectPositionResultPub = n.advertise<geometry_msgs::Vector3>("/lwa4p_red/dk_pose_result", 1);
	lwa4pRedDirectOrientationResultPub = n.advertise<geometry_msgs::Vector3>("/lwa4p_red/dk_orientation_result", 1);

	lwa4pRedJoint1Pub = n.advertise<std_msgs::Float64>("/lwa4p_red/arm_1_joint_pos_controller/command", 1);
	lwa4pRedJoint2Pub = n.advertise<std_msgs::Float64>("/lwa4p_red/arm_2_joint_pos_controller/command", 1);
	lwa4pRedJoint3Pub = n.advertise<std_msgs::Float64>("/lwa4p_red/arm_3_joint_pos_controller/command", 1);
	lwa4pRedJoint4Pub = n.advertise<std_msgs::Float64>("/lwa4p_red/arm_4_joint_pos_controller/command", 1);
	lwa4pRedJoint5Pub = n.advertise<std_msgs::Float64>("/lwa4p_red/arm_5_joint_pos_controller/command", 1);
	lwa4pRedJoint6Pub = n.advertise<std_msgs::Float64>("/lwa4p_red/arm_6_joint_pos_controller/command", 1);

	lwa4pBlueJoint1Pub = n.advertise<std_msgs::Float64>("/lwa4p_blue/arm_1_joint_pos_controller/command", 1);
	lwa4pBlueJoint2Pub = n.advertise<std_msgs::Float64>("/lwa4p_blue/arm_2_joint_pos_controller/command", 1);
	lwa4pBlueJoint3Pub = n.advertise<std_msgs::Float64>("/lwa4p_blue/arm_3_joint_pos_controller/command", 1);
	lwa4pBlueJoint4Pub = n.advertise<std_msgs::Float64>("/lwa4p_blue/arm_4_joint_pos_controller/command", 1);
	lwa4pBlueJoint5Pub = n.advertise<std_msgs::Float64>("/lwa4p_blue/arm_5_joint_pos_controller/command", 1);
	lwa4pBlueJoint6Pub = n.advertise<std_msgs::Float64>("/lwa4p_blue/arm_6_joint_pos_controller/command", 1);

	lwa4pBlueWaypointsPub = n.advertise<schunk_lwa4p_trajectory::WaypointArray>("/lwa4p_blue/waypoints", 1);
	lwa4pRedWaypointsPub = n.advertise<schunk_lwa4p_trajectory::WaypointArray>("/lwa4p_red/waypoints", 1);


	lwa4p_blue_temp_q = Eigen::MatrixXd::Zero(6, 1);
	lwa4p_red_temp_q = Eigen::MatrixXd::Zero(6, 1);


}

kinematicsTest::~kinematicsTest(){

}

void kinematicsTest::lwa4pBlueJointStatesCallBack(const sensor_msgs::JointState &msg){

	lwa4p_blue_temp_q = Eigen::MatrixXd::Zero(6, 1);

	lwa4p_blue_temp_q(0, 0) = msg.position[0];
	lwa4p_blue_temp_q(1, 0) = msg.position[1];
	lwa4p_blue_temp_q(2, 0) = msg.position[2];
	lwa4p_blue_temp_q(3, 0) = msg.position[3];
	lwa4p_blue_temp_q(4, 0) = msg.position[4];
	lwa4p_blue_temp_q(5, 0) = msg.position[5];

}

void kinematicsTest::lwa4pRedJointStatesCallBack(const sensor_msgs::JointState &msg){

	lwa4p_red_temp_q = Eigen::MatrixXd::Zero(6, 1);

	lwa4p_red_temp_q(0, 0) = msg.position[0];
	lwa4p_red_temp_q(1, 0) = msg.position[1];
	lwa4p_red_temp_q(2, 0) = msg.position[2];
	lwa4p_red_temp_q(3, 0) = msg.position[3];
	lwa4p_red_temp_q(4, 0) = msg.position[4];
	lwa4p_red_temp_q(5, 0) = msg.position[5];
}

void kinematicsTest::alphaCallBack(const std_msgs::Float64 &msg){

	alpha = msg.data;
}

void kinematicsTest::betaCallBack(const std_msgs::Float64 &msg){

	beta = msg.data;
}

schunk_lwa4p_trajectory::WaypointArray kinematicsTest::makeWaypointsMsg(Eigen::MatrixXd waypoints)
{

	schunk_lwa4p_trajectory::WaypointArray returnValue;



	for (int i = 0; i < waypoints.cols(); i++)
	{
		returnValue.waypoint_Q1.push_back(waypoints(0, i));
		returnValue.waypoint_Q2.push_back(waypoints(1, i));
		returnValue.waypoint_Q3.push_back(waypoints(2, i));
		returnValue.waypoint_Q4.push_back(waypoints(3, i));
		returnValue.waypoint_Q5.push_back(waypoints(4, i));
		returnValue.waypoint_Q6.push_back(waypoints(5, i));
	}

	return returnValue;


}

void kinematicsTest::run(){
	
	ros::Rate r(20), r2(1);

	//test direct kinematics
	double temp_q[6];
	temp_q[0] = 0;
	temp_q[1] = 0;
	temp_q[2] = 0;
	temp_q[3] = 0;
	temp_q[4] = 0;
	temp_q[5] = 0;

	
	while(ros::ok()){

		//after calling this function ROS will processes our callbacks
        ros::spinOnce();

		//cout << "Running!" << "\n";
    
		double temp_w[9], temp2[6], temp_q[6];
		Eigen::MatrixXd DK_result, IK_result;
		Eigen::MatrixXd ik_input;
		
	/*
		DK_result = lwa4p_blue.directKinematics(lwa4p_blue_temp_q, 6);
	
		vector_temp.x = (float) DK_result(0, 3)/1000;
		vector_temp.y = (float) DK_result(1, 3)/1000;
		vector_temp.z = (float) DK_result(2, 3)/1000;
		lwa4pBlueDirectPositionResultPub.publish(vector_temp);

		vector_temp.x = (float) DK_result(0, 2);
		vector_temp.y = (float) DK_result(1, 2);
		vector_temp.z = (float) DK_result(2, 2);
		lwa4pBlueDirectOrientationResultPub.publish(vector_temp);

		std::cout << "DK result : "<< endl; //DK_result(0, 3) << ", "<< DK_result(1, 3) << ", "<< DK_result(2, 3) << endl;
		std::cout << DK_result << endl;

		ik_input = Eigen::MatrixXd::Zero(9, 1);

		ik_input(0, 0) = DK_result(0, 3);
		ik_input(1, 0) = DK_result(1, 3);
		ik_input(2, 0) = DK_result(2, 3);
		ik_input(3, 0) = DK_result(0, 0);
		ik_input(4, 0) = DK_result(1, 0);
		ik_input(5, 0) = DK_result(2, 0);
		ik_input(6, 0) = DK_result(0, 2);
		ik_input(7, 0) = DK_result(1, 2);
		ik_input(8, 0) = DK_result(2, 2);

		std::cout << "Temp q = "  << endl;
		std::cout << lwa4p_blue_temp_q << endl;
		IK_result = lwa4p_blue.inverseKinematics(ik_input);
	*/
		// Test Taylor
    
		std::cout << "Taylor process started!" << endl;

		Eigen::MatrixXd Taylor_w1, Taylor_w2, Taylor_w3;
		Eigen::MatrixXd Taylor_result1, Taylor_result2, Taylor_result3, Taylor_result4, Taylor_result;

		Taylor_w1 = Eigen::MatrixXd::Zero(9, 1);
		Taylor_w2 = Eigen::MatrixXd::Zero(9, 1);
		Taylor_w3 = Eigen::MatrixXd::Zero(9, 1);

		// SEGMENT 1
		Taylor_w1(0, 0) = -500;
		Taylor_w1(1, 0) = -300;
		Taylor_w1(2, 0) = 800;
		Taylor_w1(3, 0) = 0;
		Taylor_w1(4, 0) = -1;
		Taylor_w1(5, 0) = 0;
		Taylor_w1(6, 0) = -1;
		Taylor_w1(7, 0) = 0;
		Taylor_w1(8, 0) = 0;

		Taylor_w2(0, 0) = -500;
		Taylor_w2(1, 0) = 300;
		Taylor_w2(2, 0) = 800;
		Taylor_w2(3, 0) = 0;
		Taylor_w2(4, 0) = -1;
		Taylor_w2(5, 0) = 0;
		Taylor_w2(6, 0) = -1;
		Taylor_w2(7, 0) = 0;
		Taylor_w2(8, 0) = 0;

		Taylor_w3(0, 0) = -500;
		Taylor_w3(1, 0) = 0;
		Taylor_w3(2, 0) = 1000;
		Taylor_w3(3, 0) = 0;
		Taylor_w3(4, 0) = -1;
		Taylor_w3(5, 0) = 0;
		Taylor_w3(6, 0) = -1;
		Taylor_w3(7, 0) = 0;
		Taylor_w3(8, 0) = 0;

		//Taylor_result1 = lwa4p_blue.inverseKinematics_closestQ(Taylor_w1, lwa4p_blue_temp_q);
		//Taylor_result2 = lwa4p_blue.inverseKinematics_closestQ(Taylor_w2, lwa4p_blue_temp_q);
		//Taylor_result3 = lwa4p_blue.inverseKinematics_closestQ(Taylor_w3, lwa4p_blue_temp_q);



		Taylor_result1 = lwa4p_blue.taylorPathCheck(Taylor_w3, Taylor_w1, lwa4p_blue_temp_q, 10);
		Taylor_result2 = lwa4p_blue.taylorPathCheck(Taylor_w1, Taylor_w2, Taylor_result1.col(Taylor_result1.cols()-1), 10);
		Taylor_result3 = lwa4p_blue.taylorPathCheck(Taylor_w2, Taylor_w3, Taylor_result2.col(Taylor_result2.cols()-1), 20);
		//Taylor_result3 = lwa4p_blue.taylorPathCheck(Taylor_w2, Taylor_w3, lwa4p_blue_temp_q, 10);

		Taylor_result = Eigen::MatrixXd::Zero(Taylor_result1.rows(), Taylor_result1.cols() + Taylor_result2.cols() + Taylor_result3.cols()- 2 );
	
		for (int i = 0; i < Taylor_result1.rows(); i++)
			for (int j = 0; j < Taylor_result1.cols()-1; j++)
					Taylor_result(i, j) = Taylor_result1(i, j);	
		for (int i = 0; i < Taylor_result2.rows(); i++)
			for (int j = 0; j < Taylor_result2.cols(); j++)
					Taylor_result(i, j+Taylor_result1.cols()-1) = Taylor_result2(i, j);
		
		for (int i = 0; i < Taylor_result3.rows(); i++)
			for (int j = 0; j < Taylor_result3.cols(); j++)
					Taylor_result(i, j+Taylor_result1.cols()+Taylor_result2.cols()-2) = Taylor_result3(i, j);	
	

		std::cout << "Taylor result: " << endl;
		std::cout << "("<<Taylor_result.rows() <<", "<<Taylor_result.cols()<<")" << endl;
		std::cout << "First dot: " << Taylor_result(0, 0) << ", " << Taylor_result(1, 0) << ", " << Taylor_result(2, 0) << ", " << Taylor_result(3, 0) << ", " << Taylor_result(4, 0) << ", " << Taylor_result(5, 0) << endl;

		//std::cout << Taylor_result1 << endl;
		//std::cout << Taylor_result2 << endl;
		//std::cout << Taylor_result3 << endl;
		std::cout << Taylor_result << endl;

		//std::cout << Taylor_result3 << endl;
 		std::cout << "Cols = " << Taylor_result.cols() << endl;
		

		schunk_lwa4p_trajectory::WaypointArray msg2publish;

		msg2publish = makeWaypointsMsg(Taylor_result);

		lwa4pBlueWaypointsPub.publish(msg2publish);
		std::cout << "Msg published!" << endl;
	

		Eigen::MatrixXd blue_working, blue_q;
/*
		blue_working = Eigen::MatrixXd::Zero(9, 1);


		// SEGMENT 1
		blue_working(0, 0) = 640;
		blue_working(1, 0) = 0;
		blue_working(2, 0) = 1020;
		blue_working(3, 0) = 0;
		blue_working(4, 0) = 1;
		blue_working(5, 0) = 0;
		blue_working(6, 0) = 1;
		blue_working(7, 0) = 0;
		blue_working(8, 0) = 0;

		blue_q = lwa4p_blue.inverseKinematics_closestQ(blue_working, lwa4p_blue_temp_q);

		std::cout << blue_q << endl;

		std_msgs::Float64  temp;

		temp.data = blue_q(0, 0);
		lwa4pBlueJoint1Pub.publish(temp);
		temp.data = blue_q(1, 0);
		lwa4pBlueJoint2Pub.publish(temp);
		temp.data = blue_q(2, 0);
		lwa4pBlueJoint3Pub.publish(temp);
		temp.data = blue_q(3, 0);
		lwa4pBlueJoint4Pub.publish(temp);
		temp.data = blue_q(4, 0);
		lwa4pBlueJoint5Pub.publish(temp);
		temp.data = blue_q(5, 0);
		lwa4pBlueJoint6Pub.publish(temp);
*/		
	
		int i;
		std::cin >> i;
		if (i == 0 )
			break;

		//printf("BLUE %f %f %f %f %f %f\n", lwa4p_blue_temp_q[0], lwa4p_blue_temp_q[1], lwa4p_blue_temp_q[2], lwa4p_blue_temp_q[3], lwa4p_blue_temp_q[4], lwa4p_blue_temp_q[5]);
		//printf("RED %f %f %f %f %f %f\n", lwa4p_blue_temp_q[0], lwa4p_blue_temp_q[1], lwa4p_blue_temp_q[2], lwa4p_blue_temp_q[3], lwa4p_blue_temp_q[4], lwa4p_blue_temp_q[5]);


		r.sleep();
	}
	
	

	
}