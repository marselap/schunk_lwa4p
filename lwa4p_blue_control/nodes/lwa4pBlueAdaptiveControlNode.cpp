/******************************************************************************
File name: kinematicsTestNode.cpp
Description: Node for testing Schunk LWA4P kinematics!
Author: Bruno Maric
******************************************************************************/

#include <lwa4p_blue_control/adaptiveImpedanceControl.h>

int main(int argc, char** argv){

	std::string configFile;

	// Call subscrivers init

	ros::init(argc, argv, "lwa4pBlueAdaptiveControlNode");

	ros::NodeHandle n;
	ros::NodeHandle private_node_handle_("~");

	std::string path = ros::package::getPath("lwa4p_blue_control");

    //getting ros params
    //private_node_handle_.param("config_file", configFile, path + std::string("/config/lwa4p_configuration.yaml"));


    adaptiveImpedanceControl blueControl;
    blueControl.run();


	return 0;

}
