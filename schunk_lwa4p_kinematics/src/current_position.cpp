    /******************************************************************************
    File name: current_position.cpp
    Description: Node that publishes current robot position in global coordinate system
    Author: MP
    ******************************************************************************/

    #include <schunk_lwa4p_kinematics/current_position.h>

    using namespace std;

    currentPosition::currentPosition(){

        std::string configFile;
        std::string path = ros::package::getPath("schunk_lwa4p_kinematics");

        ros::NodeHandle n;
        ros::NodeHandle private_node_handle_("~");

        private_node_handle_.param("config_file", configFile, path + std::string("/config/lwa4p_configuration.yaml"));

        lwa4p_blue.loadParameters(0, configFile);

        // init subscribers - ROBOT
        //lwa4pBlueJointStatesSub = n.subscribe("/blue_robot/joint_states", 10, &currentPosition::lwa4pBlueJointStatesCallBack, this);
        // init subscribers - SIMULATOR
        lwa4pBlueJointStatesSub = n.subscribe("/lwa4p_blue/joint_states", 10, &currentPosition::lwa4pBlueJointStatesCallBack, this);

        // init publishers
        lwa4pBlueCurrentPositionPub = n.advertise<std_msgs::Float64MultiArray>("/lwa4p_blue/current_position", 1);

        lwa4p_blue_temp_q = Eigen::MatrixXd::Zero(6, 1);
    }

    currentPosition::~currentPosition(){

    }

    void currentPosition::lwa4pBlueJointStatesCallBack(const sensor_msgs::JointState &msg){

        Eigen::MatrixXd dk_result;

        lwa4p_blue_temp_q = Eigen::MatrixXd::Zero(6, 1);

        lwa4p_blue_temp_q(0, 0) = msg.position[0];
        lwa4p_blue_temp_q(1, 0) = msg.position[1];
        lwa4p_blue_temp_q(2, 0) = msg.position[2];
        lwa4p_blue_temp_q(3, 0) = msg.position[3];
        lwa4p_blue_temp_q(4, 0) = msg.position[4];
        lwa4p_blue_temp_q(5, 0) = msg.position[5];
    }

    std_msgs::Float64MultiArray currentPosition::makePositionMsg(float* dk_w_result) {

        std_msgs::Float64MultiArray msg;

        msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
        msg.layout.dim[0].size = 9;

        for (int i = 0; i < 9; i++){
            msg.data.push_back(dk_w_result[i]);
        }

        return msg;

    }

    void currentPosition::run(){

        ros::Rate r(100);

        while(ros::ok()){

            ros::spinOnce();

            Eigen::MatrixXd dk_result;
            float dk_w_result[9];

            dk_result = lwa4p_blue.directKinematics(lwa4p_blue_temp_q, 6);
            int i = 0;
            dk_w_result[i++] = dk_result(0, 3);
            dk_w_result[i++] = dk_result(1, 3);
            dk_w_result[i++] = dk_result(2, 3);
            dk_w_result[i++] = dk_result(0, 0);
            dk_w_result[i++] = dk_result(1, 0);
            dk_w_result[i++] = dk_result(2, 0);
            dk_w_result[i++] = dk_result(0, 2);
            dk_w_result[i++] = dk_result(1, 2);
            dk_w_result[i++] = dk_result(2, 2);

            std_msgs::Float64MultiArray msg2publish;
            msg2publish = makePositionMsg(dk_w_result);
            lwa4pBlueCurrentPositionPub.publish(msg2publish);

            r.sleep();
        }

    }
