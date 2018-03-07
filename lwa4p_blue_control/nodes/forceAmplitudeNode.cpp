/******************************************************************************
File name: kinematicsTestNode.cpp
Description: Node for force sensor data filtering and analysis
Author: MP
******************************************************************************/

#include <lwa4p_blue_control/forceAmplitude.h>

int main(int argc, char** argv){

    std::string configFile;


    ros::init(argc, argv, "forceAmplitudeNode");

    ros::NodeHandle n;
    ros::NodeHandle private_node_handle_("~");

    std::string path = ros::package::getPath("lwa4p_blue_control");

    forceAmplitude forceAmplitude;
    forceAmplitude.run();

    return 0;

}
