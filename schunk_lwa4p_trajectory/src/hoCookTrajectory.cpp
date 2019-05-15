#include "schunk_lwa4p_trajectory/hoCookTrajectory.h"

trajectoryPlanning::trajectoryPlanning()
{
    // Initialize ros node handle for move group, it is a private node handle
    nhParams = ros::NodeHandle("~");

    nhParams.param("trajectory_sampling_frequency", trajectorySamplingFrequency,
        int(50));

    // First open .yaml file
    //YAML::Node config = YAML::LoadFile(file);
    //printf("FIle path = %s\n", file);
/*
    // Go through yaml file and find all data in it
    for (YAML::const_iterator it = config.begin(); it != config.end(); ++it){
        // take first key and save it to variable key
        std::string key = it->first.as<std::string>();
        //printf("%s\n", key);

        if (key.compare("lwa4p_dual_configurator") == 0){
            robot_pose_x = config[key]["lwa4p_blue_global_pose"]["x"].as<double>();
            robot_pose_y = config[key]["lwa4p_blue_global_pose"]["y"].as<double>();
            robot_pose_theta = config[key]["lwa4p_blue_global_pose"]["theta"].as<double>();
        }
    }

*/
    // Publishing sampled trajectory to trajectory_unprocessed topic. Later the
    // mission planner decides whether to use acceleration or not. Acceleration
    // is provided to mission planner anyways

    //trajectorySampledRedPub = nhTopics.advertise<schunk_lwa4p_trajectory::TrajectorySampled>("/lwa4p_red/trajectory_sampled", 1);
    trajectorySampledBluePub = nhTopics.advertise<schunk_lwa4p_trajectory::TrajectorySampled>(
        "/lwa4p_blue/trajectory_sampled", 1);


    //robotTrajectoryRedPub = nhTopics.advertise<control_msgs::FollowJointTrajectoryActionGoal>("/red_robot/pos_based_pos_traj_controller_arm/follow_joint_trajectory/goal", 1);


    // Waypoints are recieved through callback
    //waypoints_redSub = nhTopics.subscribe("/lwa4p_red/waypoints", 1, &trajectoryPlanning::waypointRedCallback, this);
    waypoints_blueSub = nhTopics.subscribe("/lwa4p_blue/waypoints", 1,
        &trajectoryPlanning::waypointBlueCallback, this);
    waypoints_moveit = nhTopics.subscribe("/execute_trajectory/goal", 1,
        &trajectoryPlanning::waypointMoveitCallback, this);
    //cout << "Trajectory acceleration scale z = " << trajectoryAccScalerZ << endl;
    //cout << "Trajectory speed scale z = " << trajectorySpeedScalerZ << endl;

    // joint velocity and acceleration limits
    double temp_vect[6];
    temp_vect[0] = 0.5;
    temp_vect[1] = 0.5;
    temp_vect[2] = 0.5;
    temp_vect[3] = 0.5;
    temp_vect[4] = 0.5;
    temp_vect[5] = 0.5;
    setJointMaxSpeed(temp_vect);
    temp_vect[0] = 1.1;
    temp_vect[1] = 1.1;
    temp_vect[2] = 1.1;
    temp_vect[3] = 1.1;
    temp_vect[4] = 1.1;
    temp_vect[5] = 1.1;
    setJointMaxAcc(temp_vect);
}

trajectoryPlanning::~trajectoryPlanning()
{
}

void trajectoryPlanning::run()
{
    ros::spin();
}

void trajectoryPlanning::waypointMoveitCallback(const moveit_msgs::ExecuteTrajectoryActionGoal &msg)
{
    int len;
    len = msg.goal.trajectory.joint_trajectory.points.size();
    std::cout << "AAAAAAAAAA GOT WAYPOINTS" << std::endl;
    std::cout << len << std::endl;

    // std::cout << "=++++++++++++++++++++++++=" << std::endl;
    TrajectoryDerivativeConditions conditions;
    DerivativeConditions tempCond;
    tempCond.speed = 0.0;
    tempCond.acceleration = 0.0;
    conditions.Q1.start = tempCond;
    conditions.Q1.end = tempCond;
    conditions.Q2.start = tempCond;
    conditions.Q2.end = tempCond;
    conditions.Q3.start = tempCond;
    conditions.Q3.end = tempCond;
    conditions.Q4.start = tempCond;
    conditions.Q4.end = tempCond;
    conditions.Q5.start = tempCond;
    conditions.Q5.end = tempCond;
    conditions.Q6.start = tempCond;
    conditions.Q6.end = tempCond;

    Eigen::MatrixXd WP_Q;
    WP_Q = Eigen::MatrixXd::Zero(len, 6);

    n = 6;
    m = len;


    for (int i = 0; i < len; i++ )
    {
        for (int joint = 0; joint < 6; joint++){
            WP_Q(i, joint) = msg.goal.trajectory.joint_trajectory.points[i].positions[joint];
        }
    }
    std::cout << "Algorithm BLUE started!" << endl;
    schunk_lwa4p_trajectory::TrajectorySampled tempTrajectory;

    if (WP_Q.rows() > 3)
    {
        // std::cout << "aaaa planning!!" << std::endl;
        tempTrajectory = this->optimizedPlan(WP_Q);
    }
    else
    {
        std::cout << "Single point procedure" << endl;
        tempTrajectory = this->point2point(WP_Q.row(WP_Q.rows()-1));
    }
    //
    trajectorySampledBluePub.publish(tempTrajectory);
    std::cout << "Trajectory BLUE published!" << endl;
    std::cout << "==========================" << endl;
}


void trajectoryPlanning::waypointBlueCallback(const schunk_lwa4p_trajectory::WaypointArray &msg)
{
    std::cout << "=++++++++++++++++++++++++=" << endl;
    TrajectoryDerivativeConditions conditions;
    DerivativeConditions tempCond;
    tempCond.speed = 0.0;
    tempCond.acceleration = 0.0;
    conditions.Q1.start = tempCond;
    conditions.Q1.end = tempCond;
    conditions.Q2.start = tempCond;
    conditions.Q2.end = tempCond;
    conditions.Q3.start = tempCond;
    conditions.Q3.end = tempCond;
    conditions.Q4.start = tempCond;
    conditions.Q4.end = tempCond;
    conditions.Q5.start = tempCond;
    conditions.Q5.end = tempCond;
    conditions.Q6.start = tempCond;
    conditions.Q6.end = tempCond;

    schunk_lwa4p_trajectory::WaypointArray msg2;

    Eigen::MatrixXd WP_Q;
    WP_Q = Eigen::MatrixXd::Zero(msg.waypoint_Q1.size(), 6);

    n = 6;
    m = msg.waypoint_Q1.size();

    for (int i = 0; i < msg.waypoint_Q1.size(); i++ )
    {
        WP_Q(i, 0) = msg.waypoint_Q1[i];
        WP_Q(i, 1) = msg.waypoint_Q2[i];
        WP_Q(i, 2) = msg.waypoint_Q3[i];
        WP_Q(i, 3) = msg.waypoint_Q4[i];
        WP_Q(i, 4) = msg.waypoint_Q5[i];
        WP_Q(i, 5) = msg.waypoint_Q6[i];
    }

    std::cout << "Algorithm BLUE started!" << endl;
    schunk_lwa4p_trajectory::TrajectorySampled tempTrajectory;

    if (WP_Q.rows() > 3)
    {
        tempTrajectory = this->optimizedPlan(WP_Q);
    }
    else
    {
        std::cout << "Single point procedure" << endl;
        tempTrajectory = this->point2point(WP_Q.row(WP_Q.rows()-1));
    }

    trajectorySampledBluePub.publish(tempTrajectory);
    std::cout << "Trajectory BLUE published!" << endl;
    std::cout << "==========================" << endl;

}


void trajectoryPlanning::waypointRedCallback(const schunk_lwa4p_trajectory::WaypointArray &msg)
{
    TrajectoryDerivativeConditions conditions;
    DerivativeConditions tempCond;
    tempCond.speed = 0.0;
    tempCond.acceleration = 0.0;
    conditions.Q1.start = tempCond;
    conditions.Q1.end = tempCond;
    conditions.Q2.start = tempCond;
    conditions.Q2.end = tempCond;
    conditions.Q3.start = tempCond;
    conditions.Q3.end = tempCond;
    conditions.Q4.start = tempCond;
    conditions.Q4.end = tempCond;
    conditions.Q5.start = tempCond;
    conditions.Q5.end = tempCond;
    conditions.Q6.start = tempCond;
    conditions.Q6.end = tempCond;

    schunk_lwa4p_trajectory::WaypointArray msg2;

    Eigen::MatrixXd WP_Q;
    WP_Q = Eigen::MatrixXd::Zero(msg.waypoint_Q1.size(), 6);

    n = 6;
    m = msg.waypoint_Q1.size();

    for (int i = 0; i < msg.waypoint_Q1.size(); i++ )
    {
        WP_Q(i, 0) = msg.waypoint_Q1[i];
        WP_Q(i, 1) = msg.waypoint_Q2[i];
        WP_Q(i, 2) = msg.waypoint_Q3[i];
        WP_Q(i, 3) = msg.waypoint_Q4[i];
        WP_Q(i, 4) = msg.waypoint_Q5[i];
        WP_Q(i, 5) = msg.waypoint_Q6[i];
    }

    std::cout << "Algortihm RED started!" << endl;

    schunk_lwa4p_trajectory::TrajectorySampled tempTrajectory;
    if (WP_Q.rows() > 1)
    {

        tempTrajectory = this->optimizedPlan(WP_Q);
    }
    else
    {
        std::cout << "Single point procedure" << endl;
        tempTrajectory = this->point2point(WP_Q);
    }

    trajectorySampledRedPub.publish(tempTrajectory);
    std::cout << "Trajectory RED published!" << endl;
}

void trajectoryPlanning::computeParametricTime(
    Eigen::MatrixXd WP_Q,
    std::vector<double> &paramTimeArray)
{
    double temp_t;

    m = WP_Q.rows();
    for (int i = 0; i < m-1; i++)
    {
        temp_t = 0;
        for (int j = 0; j < n; j++ )
        {
            temp_t = temp_t + pow(WP_Q(i, j) - WP_Q(i + 1, j), 2.0);
        }
        temp_t = sqrt(temp_t);
        paramTimeArray.push_back(temp_t);
    }
}

double trajectoryPlanning::hoCook34(
        std::vector<Eigen::MatrixXd> &B,
        Eigen::MatrixXd WP_Q,
        TrajectoryDerivativeConditions initialConditions,
        std::vector<double> paramTime)
{

    // 1. Compute matrices M, A Dq
    Eigen::MatrixXd M, A, Dq;
    this->createMatrixM(paramTime, M);
    this->createMatrixA(paramTime, WP_Q, A);
    Dq = A*M.inverse();

    // 2. Compute matrices B
    computeCoeffsB(B, WP_Q, Dq, paramTime);

    std::cout << "got here coeffsB" << std::endl;


    // 3. Compute max speed and acceleration values
    Eigen::MatrixXd max_joints_speed, max_joints_acc;
    Eigen::MatrixXd max_joint_speed, max_joint_acc;
    double max_speed, max_acc;
    max_joints_speed = Eigen::MatrixXd::Zero(n, m-1);
    max_joints_acc = Eigen::MatrixXd::Zero(n, m-1);

    // Find max for each joint on each segment
    double temp_speed, temp_acc;
    for (int i = 0; i < m-1; i++)
    {
        for (int j = 0; j < n; j++)
        {
            this->findMaxValues((Eigen::MatrixXd) B[i].row(j), temp_speed, temp_acc, 0, paramTime[i]);
            max_joints_speed(j, i) = temp_speed;
            max_joints_acc(j, i) = temp_acc;
        }
    }

    // Find max for each joint
    max_joint_speed = Eigen::MatrixXd::Zero(n, 1);
    max_joint_acc = Eigen::MatrixXd::Zero(n, 1);

    for (int i = 0; i < n; i++)
    {
        temp_speed = 0;
        temp_acc = 0;
        for (int j = 0; j < m-1; j++)
        {
            if (max_joints_speed(i, j) > temp_speed)
                temp_speed = max_joints_speed(i, j);
            if (max_joints_acc(i, j) > temp_acc)
                temp_acc = max_joints_acc(i, j);
        }
        max_joint_speed(i, 0) = temp_speed;
        max_joint_acc(i, 0) = temp_acc;
    }

    // 4. Compute scale factors
    // Find max scale
    double Sv, Sa, S;
    Sv = 0;
    Sa = 0;
    S = 0;

    for (int i = 0; i < n; i++)
    {
        if (max_joint_speed(i, 0) /maxSpeed[i] > Sv)
            Sv = max_joint_speed(i, 0) /maxSpeed[i];
        if (sqrt(max_joint_acc(i, 0) /maxAcc[i]) > Sa)
            Sa = sqrt(max_joint_acc(i, 0) /maxAcc[i]);
    }

    if (Sv > Sa)
        S = Sv;
    else
        S = Sa;

    return S;
}

void trajectoryPlanning::findMaxValues(Eigen::MatrixXd B, double &max_speed, double &max_acc, double t0, double tf)
{

    double temp_pose = 0;
    double temp_speed = 0;
    double temp_acc = 0;
    double temp_speed_max = 0;
    double temp_acc_max = 0;
    std::vector<double> temp_B;

    for (int i = 0; i < B.cols(); i++)
    {
        temp_B.push_back(B(0, i));
    }

    for (int t = 0; t <= tf*trajectorySamplingFrequency; t++)
    {
        if (B.cols() == 5)
        {
            calculatePolynomialValueOrder4(temp_B, (double) t/trajectorySamplingFrequency, temp_pose, temp_speed, temp_acc);
        }
        else if (B.cols() == 4)
        {
            calculatePolynomialValueOrder3(temp_B, (double) t/trajectorySamplingFrequency, temp_pose, temp_speed, temp_acc);
        }
        else
        {
            std::cout << "Something went wrong!" << endl;
        }

        if (abs(temp_speed) > temp_speed_max )
            temp_speed_max = abs(temp_speed);
        if (abs(temp_acc) > temp_acc_max)
            temp_acc_max = abs(temp_acc);
    }

    max_speed = temp_speed_max;
    max_acc = temp_acc_max;
}

schunk_lwa4p_trajectory::TrajectorySampled trajectoryPlanning::optimizedPlan(
        Eigen::MatrixXd WP_Q)
{
    schunk_lwa4p_trajectory::TrajectorySampled dummyReturn;
    int niter = 0;

    // init HoCook
    std::vector<Eigen::MatrixXd> B;
    double S;

    B.push_back(Eigen::MatrixXd::Zero(n, 5));
    for (int i = 0; i < m-3; i++)
        B.push_back(Eigen::MatrixXd::Zero(n, 4));
    B.push_back(Eigen::MatrixXd::Zero(n, 5));

    // HoCook Algorithm - STEP 1
    // Compute parametric time
    std::vector<double> paramTimeArray;
    this->computeParametricTime(WP_Q, paramTimeArray);



    // HoCook Algorithm - STEP 2
    // Following steps in subroutine in hoCook34, that can be called in each step of optimization
    // 1. Compute matrices M, A Dq
    // 2. Compute matrices B
    // 3. Compute max speed and acceleration values
    // 4. Compute scale factors
    TrajectoryDerivativeConditions initialConditions;
    S = hoCook34(B, WP_Q, initialConditions, paramTimeArray);

    std::cout << "S = " << S << endl;


    int tempParamTimeInt;
    double tempParamTimeFloat;
    while ( S > 1 and niter < 10)
    {
        // Modify parametric time
        for (int i = 0; i < m-1; i++)
        {
            tempParamTimeInt = paramTimeArray[i]*S * trajectorySamplingFrequency;
            tempParamTimeFloat = (tempParamTimeInt + 1.0) / trajectorySamplingFrequency;
            //paramTimeArray[i] = paramTimeArray[i]*S;
            paramTimeArray[i] = tempParamTimeFloat;
        }
        S = hoCook34(B, WP_Q, initialConditions, paramTimeArray);
        // Loopand check if scale factor is 1 then algorithm is DONE
        niter ++;
    }

    std::cout << "niter = " << niter << endl;

    //HoCook Algoritm - STEP 5
    // return sampled Trajectory
    dummyReturn = sampleTrajectory(B, paramTimeArray, trajectorySamplingFrequency);

    //control_msgs::FollowJointTrajectoryActionGoal robotTrajectoryMsg;
    //robotTrajectoryMsg = createRobotTrajectoryMsg(dummyReturn);
    //robotTrajectoryRedPub.publish(robotTrajectoryMsg);

    return dummyReturn;
}

void trajectoryPlanning::createMatrixM(std::vector<double> t, Eigen::MatrixXd &M)
{

    M = Eigen::MatrixXd::Zero(m - 2, m - 2);
    M(0, 0) = 3/t[0] + 2/t[1];
    M(1, 0) = 1/t[1];

    for (int i = 1; i < m-3; i ++ )
    {
        M(i - 1, i) = t[i + 1];
        M(i, i) = 2*(t[i] + t[i + 1]);
        M(i + 1, i) = t[i];
    }

    M(m - 4, m - 3) = 1/t[m - 3];
    M(m - 3, m - 3) = 2/t[m - 3] + 3/t[m - 2];
}

void trajectoryPlanning::createMatrixA(std::vector<double> t, Eigen::MatrixXd &Q, Eigen::MatrixXd &A)
{
    A = Eigen::MatrixXd::Zero(n, m - 2);

    for (int j = 0; j < n; j++)
    {
        A(j, 0) = (6/(pow(t[0], 2)))*(Q(1, j) - Q(0, j)) + (3/(pow(t[1], 2)))*(Q(2, j) - Q(1, j));
    }

    for (int i = 1; i < m - 3; i++)
    {
        for (int j = 0; j < n; j++)
        {
            A(j, i) = (3/(t[i]*t[i+1]))*(pow(t[i], 2)*(Q(i+2, j) - Q(i+1, j)) + pow(t[i+1], 2)*(Q(i+1, j) - Q(i, j)));
        }
    }

    for (int j = 0; j < n; j++)
    {
        A(j, m - 3) = (3/(pow(t[m-3], 2)))*(Q(m-2, j) - Q(m-3, j)) + (6/(pow(t[m-2], 2)))*(Q(m-1, j) - Q(m-2, j));
    }

}

void trajectoryPlanning::computeCoeffsB(std::vector<Eigen::MatrixXd> &B,
        Eigen::MatrixXd WP_Q, Eigen::MatrixXd Dq,
        std::vector<double> t)
{

    Eigen::MatrixXd temp_1, temp_2;

    // First segment
    temp_1 = Eigen::MatrixXd::Zero(n, 4);
    for (int i = 0; i < n; i++)
    {
        temp_1(i, 0) = WP_Q(0, i);
        temp_1(i, 1) = WP_Q(1, i);
        temp_1(i, 3) = Dq(i, 0);
    }
    temp_2 = Eigen::MatrixXd::Zero(4, 5);
    temp_2(0, 0) = 1;
    temp_2(0, 3) = -4/pow(t[0], 3);
    temp_2(0, 4) = 3/pow(t[0], 4);
    temp_2(1, 3) = 4/pow(t[0], 3);
    temp_2(1, 4) = -3/pow(t[0], 4);
    temp_2(3, 3) = -1/pow(t[0], 2);
    temp_2(3, 4) = 1/pow(t[0], 3);
    B[0] = temp_1*temp_2;

    // Middle segments
    for (int j = 1; j < m-2; j++)
    {
        temp_1 = Eigen::MatrixXd::Zero(n, 4);
        for (int i = 0; i < n; i++)
        {
            temp_1(i, 0) = WP_Q(j, i);
            temp_1(i, 1) = WP_Q(j+1, i);
            temp_1(i, 2) = Dq(i, j-1);
            temp_1(i, 3) = Dq(i, j);
        }
        temp_2 = Eigen::MatrixXd::Zero(4, 4);
        temp_2(0, 0) = 1;
        temp_2(2, 1) = 1;
        temp_2(0, 2) = -3/pow(t[j], 2);
        temp_2(0, 3) = 2/pow(t[j], 3);
        temp_2(1, 2) = 3/pow(t[j], 2);
        temp_2(1, 3) = -2/pow(t[j], 3);
        temp_2(2, 2) = -2/pow(t[j], 1);
        temp_2(2, 3) = 1/pow(t[j], 2);
        temp_2(3, 2) = -1/pow(t[j], 1);
        temp_2(3, 3) = 1/pow(t[j], 2);
        B[j] = temp_1*temp_2;
    }

    // Last segment
    temp_1 = Eigen::MatrixXd::Zero(n, 4);
    for (int i = 0; i < n; i++)
    {
        temp_1(i, 0) = WP_Q(m-2, i);
        temp_1(i, 1) = WP_Q(m-1, i);
        temp_1(i, 2) = Dq(i, m-3);
    }
    temp_2 = Eigen::MatrixXd::Zero(4, 5);
    temp_2(0, 0) = 1;
    temp_2(2, 1) = 1;
    temp_2(0, 2) = -6/pow(t[m-2], 2);
    temp_2(0, 3) = 8/pow(t[m-2], 3);
    temp_2(0, 4) = -3/pow(t[m-2], 4);
    temp_2(1, 2) = 6/pow(t[m-2], 2);
    temp_2(1, 3) = -8/pow(t[m-2], 3);
    temp_2(1, 4) = 3/pow(t[m-2], 4);
    temp_2(2, 2) = -3/pow(t[m-2], 1);
    temp_2(2, 3) = 3/pow(t[m-2], 2);
    temp_2(2, 4) = -1/pow(t[m-2], 3);
    B[m-2] = temp_1*temp_2;
}

schunk_lwa4p_trajectory::TrajectorySampled trajectoryPlanning::point2point(
        Eigen::MatrixXd WP_Q)
{

    schunk_lwa4p_trajectory::TrajectorySampled return_value;

    for (int i = 0; i < WP_Q.rows(); i++)
    {
        return_value.pose_joint_1.push_back(WP_Q(i, 0));
        return_value.speed_joint_1.push_back(0);
        return_value.acc_joint_1.push_back(0);

        return_value.pose_joint_2.push_back(WP_Q(i, 1));
        return_value.speed_joint_2.push_back(0);
        return_value.acc_joint_2.push_back(0);

        return_value.pose_joint_3.push_back(WP_Q(i, 2));
        return_value.speed_joint_3.push_back(0);
        return_value.acc_joint_3.push_back(0);

        return_value.pose_joint_4.push_back(WP_Q(i, 3));
        return_value.speed_joint_4.push_back(0);
        return_value.acc_joint_4.push_back(0);

        return_value.pose_joint_5.push_back(WP_Q(i, 4));
        return_value.speed_joint_5.push_back(0);
        return_value.acc_joint_5.push_back(0);

        return_value.pose_joint_6.push_back(WP_Q(i, 5));
        return_value.speed_joint_6.push_back(0);
        return_value.acc_joint_6.push_back(0);

    }
    return return_value;
}

schunk_lwa4p_trajectory::TrajectorySampled trajectoryPlanning::sampleTrajectory(
        std::vector<Eigen::MatrixXd> &B,
        std::vector<double> &tf,
        int sampleFrequency)
{

    std::vector<double> temp_B;
    double temp_pose = 0;
    double temp_speed = 0;
    double temp_acc = 0;

    schunk_lwa4p_trajectory::TrajectorySampled return_value;

    int lastSampleMomentSegment;

    for (int seg = 0; seg < m-1; seg++)
    {
        if (seg < m-2) {
            lastSampleMomentSegment = tf[seg]*sampleFrequency - 1;
        }
        else {
            lastSampleMomentSegment = tf[seg]*sampleFrequency;
        }
        // loop joints
        for (int i = 0; i < n; i++)
        {
            temp_B.clear();
            for (int j = 0; j < B[seg].cols(); j++)
                temp_B.push_back(B[seg](i, j));

//            for (int t = 0; t <= tf[seg]*sampleFrequency; t++)
            for (int t = 0; t <= lastSampleMomentSegment; t++)
            {
                if (B[seg].cols() == 5)
                {
                    calculatePolynomialValueOrder4(temp_B, (double) t/sampleFrequency, temp_pose, temp_speed, temp_acc);
                }
                else if (B[seg].cols() == 4)
                {
                    calculatePolynomialValueOrder3(temp_B, (double) t/sampleFrequency, temp_pose, temp_speed, temp_acc);
                }
                else
                {
                    std::cout << "Something went wrong!" << endl;
                }

                // Pack result to msg
                if (i == 0)
                {
                    return_value.pose_joint_1.push_back(temp_pose);
                    return_value.speed_joint_1.push_back(temp_speed);
                    return_value.acc_joint_1.push_back(temp_acc);
                }
                else if (i == 1)
                {
                    return_value.pose_joint_2.push_back(temp_pose);
                    return_value.speed_joint_2.push_back(temp_speed);
                    return_value.acc_joint_2.push_back(temp_acc);
                }
                else if (i == 2)
                {
                    return_value.pose_joint_3.push_back(temp_pose);
                    return_value.speed_joint_3.push_back(temp_speed);
                    return_value.acc_joint_3.push_back(temp_acc);
                }
                else if (i == 3)
                {
                    return_value.pose_joint_4.push_back(temp_pose);
                    return_value.speed_joint_4.push_back(temp_speed);
                    return_value.acc_joint_4.push_back(temp_acc);
                }
                else if (i == 4)
                {
                    return_value.pose_joint_5.push_back(temp_pose);
                    return_value.speed_joint_5.push_back(temp_speed);
                    return_value.acc_joint_5.push_back(temp_acc);
                }
                else if (i == 5)
                {
                    return_value.pose_joint_6.push_back(temp_pose);
                    return_value.speed_joint_6.push_back(temp_speed);
                    return_value.acc_joint_6.push_back(temp_acc);
                }
                else
                    std::cout << "Something went wrong!" << endl;
            }
        }
    }
    return return_value;
}

control_msgs::FollowJointTrajectoryActionGoal trajectoryPlanning::createRobotTrajectoryMsg(
        schunk_lwa4p_trajectory::TrajectorySampled sampledTrajectory)
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
    return_value.goal_id.id = "HoCook trajectory " + convert.str();
    std_msgs::Header h2, h_temp;
    ros::Time Tprep = ros::Time::now();
    h2.stamp.sec = (int) Tprep.sec;
    h2.stamp.nsec = (int) Tprep.nsec;
    return_value.goal.trajectory.header.stamp = h2.stamp; //kada da krene
    return_value.header.stamp = h2.stamp;

    std::cout << sampledTrajectory.pose_joint_1.size() << endl;

    double temp_time;
    double time_step = 1; //0.02; ///50.0;

    double new_time = 2;
    //new_time = h2.stamp.sec + h2.stamp.nsec*pow(10, -9);
    //new_time = time_step;

    //for (int i = 0; i < sampledTrajectory.pose_joint_1.size(); i ++)
    for (int i = 0; i < 4; i ++)
    {
        trajectory_msgs::JointTrajectoryPoint point_current;

        point_current.positions.push_back((double) sampledTrajectory.pose_joint_1[i] + 1.57);
        point_current.positions.push_back((double) sampledTrajectory.pose_joint_2[i]);
        point_current.positions.push_back((double) sampledTrajectory.pose_joint_3[i]);
        point_current.positions.push_back((double) sampledTrajectory.pose_joint_4[i]);
        point_current.positions.push_back((double) sampledTrajectory.pose_joint_5[i]);
        point_current.positions.push_back((double) sampledTrajectory.pose_joint_6[i]);

        point_current.velocities.push_back((double) sampledTrajectory.speed_joint_1[i]);
        point_current.velocities.push_back((double) sampledTrajectory.speed_joint_2[i]);
        point_current.velocities.push_back((double) sampledTrajectory.speed_joint_3[i]);
        point_current.velocities.push_back((double) sampledTrajectory.speed_joint_4[i]);
        point_current.velocities.push_back((double) sampledTrajectory.speed_joint_5[i]);
        point_current.velocities.push_back((double) sampledTrajectory.speed_joint_6[i]);
        /*
        point_current.velocities.push_back(0);
        point_current.velocities.push_back(0);
        point_current.velocities.push_back(0);
        point_current.velocities.push_back(0);
        point_current.velocities.push_back(0);
        point_current.velocities.push_back(0);
        */
        point_current.accelerations.push_back(0);
        point_current.accelerations.push_back(0);
        point_current.accelerations.push_back(0);
        point_current.accelerations.push_back(0);
        point_current.accelerations.push_back(0);
        point_current.accelerations.push_back(0);

        new_time = new_time + time_step;

        //point_current.time_from_start.nsec = h2.stamp.nsec + ((i+1) * (1/50))*pow(10, 9);
        point_current.time_from_start.nsec = 0; //(new_time - floor(new_time))*pow(10, 9); //(new_time % ceil(new_time))*pow(10, 9);
        point_current.time_from_start.sec = floor(new_time);

        std::cout << point_current.time_from_start.sec<<", "<< (new_time - floor(new_time))*pow(10, 9) <<", " <<h2.stamp<<", "<<h2.stamp.sec<<", "<<h2.stamp.nsec<<", " << endl;

        return_value.goal.trajectory.points.push_back(point_current);
    }
    return return_value;
}

void trajectoryPlanning::setTrajectorySamplingFrequency(int freq)
{
    trajectorySamplingFrequency = freq;
}

void trajectoryPlanning::setJointMaxSpeed(double speed[6])
{
    for (int i = 0; i < 6; i++)
        maxSpeed[i] = speed[i];
}

void trajectoryPlanning::setJointMaxAcc(double acc[6])
{
    for (int i = 0; i < 6; i++)
        maxAcc[i] = acc[i];
}

// Following set of functions calculates polynomial values at specific point
// in time. These are used for sampling position, speed and acceleration.
void trajectoryPlanning::calculatePolynomialValueOrder3(
    std::vector<double> B, double t, double &position, double &speed,
    double &acceleration)
{
    // Initialize return value and powers of time to make computation easier
    // to read
    double t1 = t;
    double t2 = t1*t;
    double t3 = t2*t;

    // Calculate position based on time and coefficients given as arguments
    position = B[3]*t3 + B[2]*t2 + B[1]*t1 + B[0];

    // Speed is derivative of position so B[0] isn't here
    speed = 3*B[3]*t2 + 2*B[2]*t1 + B[1];

    // Acceleration is second derivative of position so B[0] and B[1] are
    // not here
    acceleration = 6*B[3]*t1 + 2*B[2];
}

void trajectoryPlanning::calculatePolynomialValueOrder4(
    std::vector<double> B, double t, double &position, double &speed,
    double &acceleration)
{
    // Initialize return value and powers of time to make computation easier
    // to read
    double t1 = t;
    double t2 = t1*t;
    double t3 = t2*t;
    double t4 = t3*t;

    // Calculate position based on time and coefficients given as arguments
    position = B[4]*t4 + B[3]*t3 + B[2]*t2 + B[1]*t1 + B[0];

    // Speed is derivative of position so B[0] isn't here
    speed = 4*B[4]*t3 + 3*B[3]*t2 + 2*B[2]*t1 + B[1];

    // Acceleration is second derivative of position so B[0] and B[1] are
    // not here
    acceleration = 12*B[4]*t2 + 6*B[3]*t1 + 2*B[2];
}
