/******************************************************************************
File name: trajectoryPlanningNode.cpp
Description: Trajectory planning node for Schunk LWA4P! 
Author: Bruno Maric
******************************************************************************/

#include <schunk_lwa4p_trajectory/hoCookTrajectory.h>

int main(int argc, char** argv){

	std::string configFile;

	// Call subscrivers init

	ros::init(argc, argv, "TrajectoryPlanningNode");

	ros::NodeHandle n;
	ros::NodeHandle private_node_handle_("~");

	std::string path = ros::package::getPath("schunk_lwa4p_trajectory");

    //getting ros params
    //private_node_handle_.param("config_file", configFile, path + std::string("/config/lwa4p_configuration.yaml"));


    trajectoryPlanning planner;
    planner.run();


	return 0;

}