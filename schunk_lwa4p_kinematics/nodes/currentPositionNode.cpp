/******************************************************************************
File name: currentPositionNode.cpp
Description: Node that publishes current robot position in global coordinate system
Author: MP
******************************************************************************/

#include <schunk_lwa4p_kinematics/current_position.h>

int main(int argc, char** argv){

    std::string configFile;

    // Call subscribers init

    ros::init(argc, argv, "currentPositionNode");

    ros::NodeHandle n;
    ros::NodeHandle private_node_handle_("~");

    std::string path = ros::package::getPath("schunk_lwa4p_kinematics");

    //getting ros params
    private_node_handle_.param("config_file", configFile, path + std::string("/config/lwa4p_configuration.yaml"));

    currentPosition dk_position;
    dk_position.run();

    return 0;

}
