/******************************************************************************
File name: guiControlNode.cpp
Description: Node for testing Schunk LWA4P kinematics!
Author: Bruno Maric
******************************************************************************/

#include <schunk_lwa4p_kinematics/gui_control.h>

int main(int argc, char** argv){

    std::string configFile;

    // Call subscribers init

    ros::init(argc, argv, "GuiControlNode");

    ros::NodeHandle n;
    ros::NodeHandle private_node_handle_("~");

    std::string path = ros::package::getPath("schunk_lwa4p_kinematics");

    //getting ros params
    private_node_handle_.param("config_file", configFile, path + std::string("/config/lwa4p_configuration.yaml"));

    guiControl test;
    test.run();

    return 0;

}
