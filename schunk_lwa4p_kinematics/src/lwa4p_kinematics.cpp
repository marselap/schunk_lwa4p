#include <schunk_lwa4p_kinematics/lwa4p_kinematics.h>
#include <iostream>


lwa4p_kinematics::lwa4p_kinematics()
{
    eps = 0.001;
    eps_pose = 0.00001;
    eps_orientation = 0.00001;

}

void lwa4p_kinematics::loadParameters(int robot_id, std::string file)
{

    // robot_id == 0 -->'schunk_lwa4p_blue'
    // robot_id == 1 --> 'schunk_lwa4p_red'

    // First open .yaml file
    YAML::Node config = YAML::LoadFile(file);
    //printf("FIle path = %s\n", file);

    // Go through yaml file and find all data in it
    for (YAML::const_iterator it = config.begin(); it != config.end(); ++it)
    {
        // take first key and save it to variable key
        std::string key = it->first.as<std::string>();
        //printf("%s\n", key);

        if (robot_id == 0 && key.compare("lwa4p_dual_configurator") == 0)
        {
            robot_pose_x = config[key]["lwa4p_blue_global_pose"]["x"].as<double>();
            robot_pose_y = config[key]["lwa4p_blue_global_pose"]["y"].as<double>();
            robot_pose_theta = config[key]["lwa4p_blue_global_pose"]["theta"].as<double>();
        }
        else if (robot_id == 1 && key.compare("lwa4p_dual_configuration") == 0)
        {
            robot_pose_x = config[key]["lwa4p_red_global_pose"]["x"].as<double>();
            robot_pose_y = config[key]["lwa4p_red_global_pose"]["y"].as<double>();
            robot_pose_theta = config[key]["lwa4p_red_global_pose"]["theta"].as<double>();
        }
        else if (robot_id == 0 && key.compare("lwa4p_blue") == 0)
        {
            dh_param_d[0] = config[key]["DH_d"]["d0"].as<double>();
            dh_param_d[1] = config[key]["DH_d"]["d1"].as<double>();
            dh_param_d[2] = config[key]["DH_d"]["d2"].as<double>();
            dh_param_d[3] = config[key]["DH_d"]["d3"].as<double>();
            dh_param_d[4] = config[key]["DH_d"]["d4"].as<double>();
            dh_param_d[5] = config[key]["DH_d"]["d5"].as<double>();

            dh_param_a[0] = config[key]["DH_a"]["a0"].as<double>();
            dh_param_a[1] = config[key]["DH_a"]["a1"].as<double>();
            dh_param_a[2] = config[key]["DH_a"]["a2"].as<double>();
            dh_param_a[3] = config[key]["DH_a"]["a3"].as<double>();
            dh_param_a[4] = config[key]["DH_a"]["a4"].as<double>();
            dh_param_a[5] = config[key]["DH_a"]["a5"].as<double>();

            dh_param_alpha[0] = config[key]["DH_alpha"]["alpha1"].as<double>();
            dh_param_alpha[1] = config[key]["DH_alpha"]["alpha2"].as<double>();
            dh_param_alpha[2] = config[key]["DH_alpha"]["alpha3"].as<double>();
            dh_param_alpha[3] = config[key]["DH_alpha"]["alpha4"].as<double>();
            dh_param_alpha[4] = config[key]["DH_alpha"]["alpha5"].as<double>();
            dh_param_alpha[5] = config[key]["DH_alpha"]["alpha6"].as<double>();

            dh_param_Q0[0] = config[key]["Q0"]["q0_0"].as<double>();
            dh_param_Q0[1] = config[key]["Q0"]["q0_1"].as<double>();
            dh_param_Q0[2] = config[key]["Q0"]["q0_2"].as<double>();
            dh_param_Q0[3] = config[key]["Q0"]["q0_3"].as<double>();
            dh_param_Q0[4] = config[key]["Q0"]["q0_4"].as<double>();
            dh_param_Q0[5] = config[key]["Q0"]["q0_5"].as<double>();

            joint_limits_min[0] = config[key]["joint_limits"]["joint1_min"].as<double>();
            joint_limits_min[1] = config[key]["joint_limits"]["joint2_min"].as<double>();
            joint_limits_min[2] = config[key]["joint_limits"]["joint3_min"].as<double>();
            joint_limits_min[3] = config[key]["joint_limits"]["joint4_min"].as<double>();
            joint_limits_min[4] = config[key]["joint_limits"]["joint5_min"].as<double>();
            joint_limits_min[5] = config[key]["joint_limits"]["joint6_min"].as<double>();

            joint_limits_max[0] = config[key]["joint_limits"]["joint1_max"].as<double>();
            joint_limits_max[1] = config[key]["joint_limits"]["joint2_max"].as<double>();
            joint_limits_max[2] = config[key]["joint_limits"]["joint3_max"].as<double>();
            joint_limits_max[3] = config[key]["joint_limits"]["joint4_max"].as<double>();
            joint_limits_max[4] = config[key]["joint_limits"]["joint5_max"].as<double>();
            joint_limits_max[5] = config[key]["joint_limits"]["joint6_max"].as<double>();

        }
        else if (robot_id == 1 && key.compare("lwa4p_red") == 0)
        {
            dh_param_d[0] = config[key]["DH_d"]["d0"].as<double>();
            dh_param_d[1] = config[key]["DH_d"]["d1"].as<double>();
            dh_param_d[2] = config[key]["DH_d"]["d2"].as<double>();
            dh_param_d[3] = config[key]["DH_d"]["d3"].as<double>();
            dh_param_d[4] = config[key]["DH_d"]["d4"].as<double>();
            dh_param_d[5] = config[key]["DH_d"]["d5"].as<double>();

            dh_param_a[0] = config[key]["DH_a"]["a0"].as<double>();
            dh_param_a[1] = config[key]["DH_a"]["a1"].as<double>();
            dh_param_a[2] = config[key]["DH_a"]["a2"].as<double>();
            dh_param_a[3] = config[key]["DH_a"]["a3"].as<double>();
            dh_param_a[4] = config[key]["DH_a"]["a4"].as<double>();
            dh_param_a[5] = config[key]["DH_a"]["a5"].as<double>();

            dh_param_alpha[0] = config[key]["DH_alpha"]["alpha1"].as<double>();
            dh_param_alpha[1] = config[key]["DH_alpha"]["alpha2"].as<double>();
            dh_param_alpha[2] = config[key]["DH_alpha"]["alpha3"].as<double>();
            dh_param_alpha[3] = config[key]["DH_alpha"]["alpha4"].as<double>();
            dh_param_alpha[4] = config[key]["DH_alpha"]["alpha5"].as<double>();
            dh_param_alpha[5] = config[key]["DH_alpha"]["alpha6"].as<double>();

            dh_param_Q0[0] = config[key]["Q0"]["q0_0"].as<double>();
            dh_param_Q0[1] = config[key]["Q0"]["q0_1"].as<double>();
            dh_param_Q0[2] = config[key]["Q0"]["q0_2"].as<double>();
            dh_param_Q0[3] = config[key]["Q0"]["q0_3"].as<double>();
            dh_param_Q0[4] = config[key]["Q0"]["q0_4"].as<double>();
            dh_param_Q0[5] = config[key]["Q0"]["q0_5"].as<double>();

            joint_limits_min[0] = config[key]["joint_limits"]["joint1_min"].as<double>();
            joint_limits_min[1] = config[key]["joint_limits"]["joint2_min"].as<double>();
            joint_limits_min[2] = config[key]["joint_limits"]["joint3_min"].as<double>();
            joint_limits_min[3] = config[key]["joint_limits"]["joint4_min"].as<double>();
            joint_limits_min[4] = config[key]["joint_limits"]["joint5_min"].as<double>();
            joint_limits_min[5] = config[key]["joint_limits"]["joint6_min"].as<double>();

            joint_limits_max[0] = config[key]["joint_limits"]["joint1_max"].as<double>();
            joint_limits_max[1] = config[key]["joint_limits"]["joint2_max"].as<double>();
            joint_limits_max[2] = config[key]["joint_limits"]["joint3_max"].as<double>();
            joint_limits_max[3] = config[key]["joint_limits"]["joint4_max"].as<double>();
            joint_limits_max[4] = config[key]["joint_limits"]["joint5_max"].as<double>();
            joint_limits_max[5] = config[key]["joint_limits"]["joint6_max"].as<double>();
        }
    }

}

Eigen::MatrixXd lwa4p_kinematics::directKinematics(Eigen::MatrixXd direct_qIn, int end_joint = 6)
{

    Eigen::MatrixXd T_temp;
    Eigen::MatrixXd T_final;

    direct_q[0] = (direct_qIn(0, 0) + dh_param_Q0[0]);
    direct_q[1] = (direct_qIn(1, 0) + dh_param_Q0[1]);
    direct_q[2] = (-direct_qIn(2, 0) + dh_param_Q0[2]);
    direct_q[3] = (direct_qIn(3, 0) + dh_param_Q0[3]);
    direct_q[4] = (-direct_qIn(4, 0) + dh_param_Q0[4]);
    direct_q[5] = (direct_qIn(5, 0) + dh_param_Q0[5]);

    /*
        Transformation matrix layout
        [cos(theta_k), -cos(alpha_k)*sin(theta_k), sin(alpha_k)*sin(theta_k), a_k*cos(theta_k),
         sin(theta_k), cos(alpha_k)*cos(theta_k), -sin(alpha_k)*cos(theta_k), a_k*sin(theta_k),
         0, sin(alpha_k), cos(alpha_k), d_k,
         0, 0, 0, 1    ]
    */

    T_temp = Eigen::MatrixXd::Zero(4, 4);
    T_final = Eigen::MatrixXd::Identity(4, 4);

    for (int i = 0; i < end_joint; i++)
    {
        // Form i-th transformation matrix
        T_temp = Eigen::MatrixXd::Zero(4, 4);

        T_temp(0, 0) = cos(direct_q[i]);
        T_temp(1, 0) = sin(direct_q[i]);

        T_temp(0, 1) = -cos(dh_param_alpha[i])*sin(direct_q[i]);
        T_temp(1, 1) = cos(dh_param_alpha[i])*cos(direct_q[i]);
        T_temp(2, 1) = sin(dh_param_alpha[i]);

        T_temp(0, 2) = sin(dh_param_alpha[i])*sin(direct_q[i]);
        T_temp(1, 2) = -sin(dh_param_alpha[i])*cos(direct_q[i]);
        T_temp(2, 2) = cos(dh_param_alpha[i]);

        T_temp(0, 3) = dh_param_a[i]*cos(direct_q[i]);
        T_temp(1, 3) = dh_param_a[i]*sin(direct_q[i]);
        T_temp(2, 3) = dh_param_d[i];
        T_temp(3, 3) = 1;

        //Calculate transformation matrix to the i-th joint
        T_final = T_final*T_temp;

    }

    // Create result vector: direct_w = [position, orientation]
/*
    direct_w[0] = T_final(0, 3);
    direct_w[1] = T_final(1, 3);
    direct_w[2] = T_final(2, 3);
    direct_w[3] = T_final(0, 0);
    direct_w[4] = T_final(1, 0);
    direct_w[5] = T_final(2, 0);
    direct_w[6] = T_final(0, 2);
    direct_w[7] = T_final(1, 2);
    direct_w[8] = T_final(2, 2);
*/

    //std::cout << "DK result:" << direct_w[0]<<", "<< direct_w[1]<<", "<< direct_w[2]<<", "<< direct_w[3]<<", "<< direct_w[4]<<", "<< direct_w[5]<<", "<< direct_w[6]<<", "<< direct_w[7]<<", "<<  direct_w[8]<<"\n";
    //std::cout << T_final<< "\n";
    //std::cout << "\n";

    return T_final;
}


Eigen::MatrixXd lwa4p_kinematics::inverseKinematics(Eigen::MatrixXd ik_goal_w)
{

    //std::cout << "ID input, goal_w: " << ik_goal_w(0, 0) <<", "<< ik_goal_w(1, 0) <<", "<< ik_goal_w(2, 0) <<", "<< ik_goal_w(3, 0) <<", "<< ik_goal_w(4, 0) <<", "<< ik_goal_w(5, 0) <<", "<< ik_goal_w(6, 0) <<", "<< ik_goal_w(7, 0) <<", "<< ik_goal_w(8, 0)<<"\n";

    // Solution layout
    // [q1-1, q2-1-1, q3-1-1, q4-1-1-1, q5-1-1-1, q6;
    //  q1-1, q2-1-1, q3-1-1, q4-1-1-2, q5-1-1-2, q6;
    //  q1-1, q2-1-2, q3-1-2, q4-1-2-1, q5-1-2-1, q6;
    //  q1-1, q2-1-2, q3-1-2, q4-1-2-2, q5-1-2-2, q6;
    //  q1-2, q2-2-1, q3-2-1, q4-2-1-1, q5-2-1-1, q6;
    //  q1-2, q2-2-1, q3-2-1, q4-2-1-2, q5-2-1-2, q6;
    //  q1-2, q2-2-2, q3-2-2, q4-2-2-1, q5-2-2-1, q6;
    //  q1-2, q2-2-2, q3-2-2, q4-2-2-2, q5-2-2-2, q6]
    inverse_Q = Eigen::MatrixXcd::Zero(8, 6);

    goal_vect = Eigen::MatrixXcd::Zero(9, 1);
    temp_w = Eigen::MatrixXcd::Zero(9, 1);

    p1 = Eigen::MatrixXcd::Zero(1, 1);
    p2 = Eigen::MatrixXcd::Zero(8, 1);

    // Waist
    for (int i = 0; i < 4; i++)
    {
        inverse_Q(i, 0) = atan2(ik_goal_w(1, 0) - dh_param_d[5]*ik_goal_w(7, 0), ik_goal_w(0, 0) - dh_param_d[5]*ik_goal_w(6, 0));

    }
    for (int i = 4; i < 8; i++)
    {
        inverse_Q(i, 0)  = wrapToPi(inverse_Q.real()(0, 0) + M_PI);     // WRAP_TO_PI
    }

    // Auxiliary variables
    p1(0, 0) = ik_goal_w(2, 0) - dh_param_d[0] - dh_param_d[5]*ik_goal_w(8, 0);
    for (int i = 0; i < 8; i++)
    {
        p2(i, 0) = ik_goal_w(0, 0)*cos(inverse_Q.real()(i, 0)) + ik_goal_w(1, 0)*sin(inverse_Q.real()(i, 0)) - dh_param_d[5]*(ik_goal_w(6, 0)*cos(inverse_Q.real()(i, 0)) + ik_goal_w(7, 0)*sin(inverse_Q.real()(i, 0)));

    }


    // Elbow
    for (int i = 0; i < 8; i++)
    {
        inverse_Q(i, 2) = acos((pow(p1.real()(0, 0), 2) + pow(p2.real()(i, 0), 2) - pow(dh_param_a[1], 2) - pow(dh_param_d[3], 2))/(2*dh_param_a[1]*dh_param_d[3]));
    }

    // For q1-1
    // q([3:4,7:8],3) = sign(q([3:4,7:8],3))*pi - q([3:4,7:8],3);
    for (int i = 2; i < 4; i++)
    {
        // inverse_Q(i, 2) = this->sign(inverse_Q.real()(i, 2))*M_PI - inverse_Q.real()(i, 2);
        inverse_Q(i, 2) = - inverse_Q.real()(i, 2);
    }
    for (int i = 6; i < 8; i++)
    {
        // inverse_Q(i, 2) = this->sign(inverse_Q.real()(i, 2))*M_PI - inverse_Q.real()(i, 2);
        inverse_Q(i, 2) = - inverse_Q.real()(i, 2);
    }

    // Shoulder
    // We are discarding the imaginary part to supress warnings. Infeasible
    // solutions will show up with imaginary q5.
    //q(:,2) = atan2(real(p1*(l2+l3*sin(q(:,3)))+p2.*l3.*cos(q(:,3))),...
    //           real(p2.*(l2+l3*sin(q(:,3)))-p1*l3*cos(q(:,3))));


    for (int i = 0; i < 8; i++)
    {
        temp_i_1 = Eigen::MatrixXd::Zero(1, 1);
        temp_i_2 = Eigen::MatrixXd::Zero(1, 1);



        // temp_i_1(0, 0) = p1.real()(0, 0)*(dh_param_a[1] + dh_param_d[3]*sin(inverse_Q.real()(i, 2))) + p2.real()(i, 0)*dh_param_d[3]*cos(inverse_Q.real()(i, 2));
        // temp_i_2(0, 0) = p2.real()(i, 0)*(dh_param_a[1] + dh_param_d[3]*sin(inverse_Q.real()(i, 2))) - p1.real()(0, 0)*dh_param_d[3]*cos(inverse_Q.real()(i, 2));
        temp_i_1(0, 0) = -p2.real()(i, 0)*(dh_param_a[1] + dh_param_d[3]*cos(inverse_Q.real()(i, 2))) - p1.real()(0, 0)*dh_param_d[3]*sin(inverse_Q.real()(i, 2));
        temp_i_2(0, 0) =  p1.real()(0, 0)*(dh_param_a[1] + dh_param_d[3]*cos(inverse_Q.real()(i, 2))) - p2.real()(i, 0)*dh_param_d[3]*sin(inverse_Q.real()(i, 2));


        inverse_Q(i, 1) = (double) atan2((double) temp_i_1.real()(0, 0), (double) temp_i_2.real()(0, 0));
    }


    // Wrist pitch
    for (int i = 0; i < 7; i += 2)
    {
        //std::cout << "acos("<<-(ik_goal_w(6, 0)*cos(inverse_Q.real()(i, 0)) + ik_goal_w(7, 0)*sin(inverse_Q.real()(i, 0)))*sin(inverse_Q.real()(i, 1) + inverse_Q.real()(i, 2)) + ik_goal_w(8, 0)*cos(inverse_Q.real()(i, 1) + inverse_Q.real()(i, 2))<<")=";
        inverse_Q(i, 4) = acos(-(ik_goal_w(6, 0)*cos(inverse_Q.real()(i, 0)) + ik_goal_w(7, 0)*sin(inverse_Q.real()(i, 0)))*sin(inverse_Q.real()(i, 1) + inverse_Q.real()(i, 2)) + ik_goal_w(8, 0)*cos(inverse_Q.real()(i, 1) + inverse_Q.real()(i, 2)));
        inverse_Q(i+1, 4) = -inverse_Q.real()(i, 4);
        //std::cout << inverse_Q(i, 4) << endl;
    }


    // Elbow roll
    // !!! Undefined when q5 = 0
    // We are discarding the imaginary part to supress warnings. Infeasible
    // solutions will show up with imaginary q5.
    for (int i = 0; i < 8; i++)
    {
        temp_i_1 = Eigen::MatrixXd::Zero(1, 1);
        temp_i_2 = Eigen::MatrixXd::Zero(1, 1);

        // temp_i_1(0, 0) = sin(inverse_Q.real()(i, 4))*(-r1.real()(0, 0)*sin(inverse_Q.real()(i, 0)) + r2.real()(0, 0)*cos(inverse_Q.real()(i, 0)));
        // temp_i_2(0, 0) = sin(inverse_Q.real()(i, 4))*(-(r1.real()(0, 0)*cos(inverse_Q.real()(i, 0)) + r2.real()(0, 0)*sin(inverse_Q.real()(i, 0)))*cos(inverse_Q.real()(i, 1) + inverse_Q.real()(i, 2)) - r3.real()(0, 0)*sin(inverse_Q.real()(i, 1) + inverse_Q.real()(i, 2)));

        temp_i_1(0, 0) = sin(inverse_Q.real()(i, 4))*(ik_goal_w(6, 0)*sin(inverse_Q.real()(i, 0)) - ik_goal_w(7, 0)*cos(inverse_Q.real()(i, 0)));
        temp_i_2(0, 0) = -sin(inverse_Q.real()(i, 4))*((ik_goal_w(6, 0)*cos(inverse_Q.real()(i, 0)) + ik_goal_w(7, 0)*sin(inverse_Q.real()(i, 0)))*cos(inverse_Q.real()(i, 1) + inverse_Q.real()(i, 2)) + ik_goal_w(8, 0)*sin(inverse_Q.real()(i, 1) + inverse_Q.real()(i, 2)));

        inverse_Q(i, 3) = (double) atan2((double) temp_i_1.real()(0, 0), (double) temp_i_2.real()(0, 0)) ;
        //inverse_Q(i, 3) = inverse_Q.real()(i, 3) + M_PI;

      }

      // Wrist roll
      Eigen::MatrixXd vector_y;
      vector_y = Eigen::MatrixXd::Zero(3, 1);

      vector_y(0, 0) = ik_goal_w(5, 0)*ik_goal_w(7, 0) - ik_goal_w(4, 0)*ik_goal_w(8, 0);
      vector_y(1, 0) = ik_goal_w(3, 0)*ik_goal_w(8, 0) - ik_goal_w(5, 0)*ik_goal_w(6, 0);
      vector_y(2, 0) = ik_goal_w(4, 0)*ik_goal_w(6, 0) - ik_goal_w(3, 0)*ik_goal_w(7, 0);



      for (int i = 0 ; i < 8; i++)
      {

          temp_i_1 = Eigen::MatrixXd::Zero(1, 1);
        temp_i_2 = Eigen::MatrixXd::Zero(1, 1);

        temp_i_1(0, 0) = sin(inverse_Q.real()(i, 4))*(  vector_y(2, 0)*(cos(inverse_Q.real()(i, 1)+inverse_Q.real()(i, 2))) - ( vector_y(0, 0)*cos(inverse_Q.real()(i, 0))+ vector_y(1, 0)*sin(inverse_Q.real()(i, 0)))*sin(inverse_Q.real()(i, 1)+inverse_Q.real()(i, 2)));
        temp_i_2(0, 0) = sin(inverse_Q.real()(i, 4))*(-ik_goal_w(5, 0)*(cos(inverse_Q.real()(i, 1)+inverse_Q.real()(i, 2))) + (ik_goal_w(3, 0)*cos(inverse_Q.real()(i, 0))+ik_goal_w(4, 0)*sin(inverse_Q.real()(i, 0)))*sin(inverse_Q.real()(i, 1)+inverse_Q.real()(i, 2)));



        inverse_Q(i, 5) = (double) atan2((double) temp_i_1.real()(0, 0), (double) temp_i_2.real()(0, 0)) ;

      }




    // When q5=0 set q4 to "default" value, which is pi in my kinematic model.
/*    for (int i = 0; i < 8; i++)
    {
        if (inverse_Q.real()(i, 4) < 2*eps)
            inverse_Q(i, 3) = M_PI;
    }
*/
    // Filter infeasible points.
    // Infeasible points contain imaginary joint rotations.
    // TODO: Are there any other indicators of infeasibility?
    // TODO: Currently, the imaginary cutoff value is arbitrary,
    //       is there a better way to pick a cutoff
    for (int i = 0; i < 8; i++)
        valid_results[i] = true;

    for (int i = 7; i >= 0; i--)
    {


        // Check for outliers in row i
        temp_outlier = false;
        for (int j = 0; j < 6; j++)
        {

            if ( ((inverse_Q.imag()(i, j))  > eps) )
            {
                temp_outlier = true;
                break;
            }
        }

        if (temp_outlier)
        {
            valid_results[i] = false;
        }



    }



    for (int i = 0; i < 8; i++)
    {

        //inverse_Q(i, 3) = wrapToPi(inverse_Q.real()(i, 3) + M_PI);
        inverse_Q(i, 2) *= -1;
        inverse_Q(i, 4) *= -1;

      }





      Eigen::MatrixXd DK_result, direct_check_q;

      direct_check_q = Eigen::MatrixXd::Zero(6, 1);




    // Chect inverse result using direct kinematics


      // Check IK results, using DK
    for (int i = 0; i < 8; i++)
    {
        for (int j = 0; j < 6; j++)
            direct_check_q(j, 0) = inverse_Q.real()(i, j);

        DK_result =  directKinematics(direct_check_q);
        temp_w(0, 0) = DK_result(0, 3);
          temp_w(1, 0) = DK_result(1, 3);
          temp_w(2, 0) = DK_result(2, 3);
          temp_w(3, 0) = DK_result(0, 0);
          temp_w(4, 0) = DK_result(1, 0);
          temp_w(5, 0) = DK_result(2, 0);
          temp_w(6, 0) = DK_result(0, 2);
          temp_w(7, 0) = DK_result(1, 2);
          temp_w(8, 0) = DK_result(2, 2);



        norm_vect = temp_w - ik_goal_w;


        if (norm_vect.lpNorm<Eigen::Infinity>() > pow(10, -6))
        {
            valid_results[i] = false;
        }
        // Check joint limits
        for (int j = 0; j < 6; j++)
            if ((inverse_Q.real()(i, j) < joint_limits_min[j]) or (inverse_Q.real()(i, j) > joint_limits_max[j]) )
            {
                    valid_results[i] = false;
                    break;
            }

    }



    // Make final matrix with only valid solutions
    // init var to count how many valid solutions are in inverse_Q
    inverse_solutions = 0;

    for (int i = 0; i < 8; i++)
        if (valid_results[i])
            inverse_solutions ++;

    // init matrix and index for final IK matrix
    inverse_Q_final = Eigen::MatrixXd::Zero(inverse_solutions, 6);
    temp_solution = 0;

    for (int i = 0; i < 8; i++)
    {
        if (valid_results[i])
        {
            for (int j = 0; j < 6; j++)
            {
                inverse_Q_final(temp_solution, j) = inverse_Q.real()(i, j);

            }
        temp_solution++;

        }
    }




    /*
    temp_inverse_result[0] = inverse_Q_final.real()(0, 0);
    temp_inverse_result[1] = inverse_Q_final.real()(0, 1);
    temp_inverse_result[2] = inverse_Q_final.real()(0, 2);
    temp_inverse_result[3] = inverse_Q_final.real()(0, 3);
    temp_inverse_result[4] = inverse_Q_final.real()(0, 4);
    temp_inverse_result[5] = inverse_Q_final.real()(0, 5);
     */

     //std::cout << "Final result, solution number="<<inverse_solutions << endl;
    //std::cout << inverse_Q << endl;
    //std::cout << inverse_Q_final << endl;

    return inverse_Q_final;

}

Eigen::MatrixXd lwa4p_kinematics::inverseKinematics_closestQ(Eigen::MatrixXd goal_w, Eigen::MatrixXd temp_q)
{

    Eigen::MatrixXd IK_result, returnValue;


    IK_result = inverseKinematics(goal_w);



    // Handle singularity in Schunk configuration
    // When q(5) = 0 the q(4) rotation is arbitrary; We will set it to q0(4)
    // When doing trajectory planning, this will prevent sudden jumps in q(4)
    if (IK_result.rows() > 0)
        for (int i = (IK_result.rows()-1); i >= 0; i--)
            if (IK_result(i, 4) < 2*eps)
                IK_result(i, 3)    = temp_q(3, 0);



    ikc_temp = Eigen::MatrixXcd::Zero(6, 1);
    // Compute the closest q1
    k_min = IK_result.rows()-1;
    if (k_min > 0)
    {
        for (int i = 0; i < 6; i++)
        {
            ikc_temp(i, 0) = wrapToPi(IK_result(IK_result.rows()-1, i) - temp_q(i, 0));
        }
        d_min = ikc_temp.lpNorm<Eigen::Infinity>();

        for (int i = k_min - 1; i >= 0; i--)
        {
            for (int j = 0; j < 6; j++)
            {
                ikc_temp(j, 0) = wrapToPi(IK_result(i, j) - temp_q(j, 0));
            }
            d = ikc_temp.lpNorm<Eigen::Infinity>();
            if (d < d_min)
            {
                k_min = i;
                d_min = d;
            }
        }

    }
    //std::cout<<"k_min="<<k_min<<"\n";
    //make return vector

    returnValue = Eigen::MatrixXd::Zero(6, 1);

    if (k_min >= 0)
        for(int i = 0; i < 6; i++)
        {
            returnValue(i, 0) = inverse_Q_final(k_min, i);  //(k_min, i);
        }

//    std::cout << "IK Closest result: " << returnValue(0, 0) << ", " << returnValue(1, 0) << ", " << returnValue(2, 0) << ", " << returnValue(3, 0) << ", " << returnValue(4, 0) << ", " << returnValue(5, 0) << endl;

    return returnValue;

}

Eigen::MatrixXd lwa4p_kinematics::taylorPathCheck(Eigen::MatrixXd w1, Eigen::MatrixXd w2, Eigen::MatrixXd q0, double tolerance)
{

    Eigen::MatrixXd taylorResult, finalResult;
    Eigen::MatrixXd dk_result, dk_w_result, ik_result;
    Eigen::MatrixXd q_prev;

    dk_w_result = Eigen::MatrixXd::Zero(9, 1);
    taylorResult = taylorPath(w1, w2, tolerance, 0);
    finalResult = Eigen::MatrixXd::Zero(taylorResult.rows(), taylorResult.cols());

    //std::cout << taylorResult << endl;

    dk_result = directKinematics(taylorResult.col(0));
    dk_w_result(0, 0) = dk_result(0, 3);
    dk_w_result(1, 0) = dk_result(1, 3);
    dk_w_result(2, 0) = dk_result(2, 3);
    dk_w_result(3, 0) = dk_result(0, 0);
    dk_w_result(4, 0) = dk_result(1, 0);
    dk_w_result(5, 0) = dk_result(2, 0);
    dk_w_result(6, 0) = dk_result(0, 2);
    dk_w_result(7, 0) = dk_result(1, 2);
    dk_w_result(8, 0) = dk_result(2, 2);
    q_prev = inverseKinematics_closestQ(dk_w_result, q0);

    for (int i = 0; i < 6; i++)
        finalResult(i, 0) = q_prev(i, 0);


    for (int i = 1; i < taylorResult.cols(); i++)
    {
        dk_result = directKinematics(taylorResult.col(i));
        dk_w_result(0, 0) = dk_result(0, 3);
        dk_w_result(1, 0) = dk_result(1, 3);
        dk_w_result(2, 0) = dk_result(2, 3);
        dk_w_result(3, 0) = dk_result(0, 0);
        dk_w_result(4, 0) = dk_result(1, 0);
        dk_w_result(5, 0) = dk_result(2, 0);
        dk_w_result(6, 0) = dk_result(0, 2);
        dk_w_result(7, 0) = dk_result(1, 2);
        dk_w_result(8, 0) = dk_result(2, 2);
        ik_result = inverseKinematics_closestQ(dk_w_result, q_prev);

        for (int j = 0; j < 6; j++)
            finalResult(j, i) = ik_result(j, 0);

        q_prev = ik_result;
    }

    return finalResult;

}

Eigen::MatrixXd lwa4p_kinematics::taylorPath(Eigen::MatrixXd w1, Eigen::MatrixXd w2, double tolerance, int nrec)
{

    Eigen::MatrixXd q0, q1, q2;
    Eigen::MatrixXd qm, wm, wM, w_norm;
    Eigen::MatrixXd q_final_1, q_final_1_1, q_final_2, q_final;

    q_final_1 = Eigen::MatrixXd::Zero(6, 2);
    q0 = Eigen::MatrixXd::Zero(6, 1);

    q1 = inverseKinematics_closestQ(w1, q0);
    q2 = inverseKinematics_closestQ(w2, q0);



    if ((q1.rows() > 0) and (q2.rows() > 0))
    {
        //std::cout << q1 << endl;
        //std::cout << q2 << endl;

        q_final = Eigen::MatrixXd::Zero(6, 2);
        q_final << q1, q2;

        //std::cout << q << endl;

        qm = Eigen::MatrixXd::Zero(6, 1);
        for (int i = 0; i < 6; i++)
            qm(i, 0) = wrapToPi((q1(i, 0) + q2(i, 0))/2);

        wm = directKinematics(qm);

        wM = Eigen::MatrixXd::Zero(9, 1);
        for (int i = 0; i < 9; i++)
            wM(i, 0) = (w1(i, 0) + w2(i, 0))/2;


        // make norm vector
        w_norm = Eigen::MatrixXd::Zero(3, 1);
        for (int i = 0; i < 3; i++)
            w_norm(i, 0) = wm(i, 3) - wM(i, 0);

        //std::cout << "wm " << wm << endl;
        //std::cout << "wM " << wM << endl;
        //std::cout << "w_norm result = " << w_norm.lpNorm<2>()  << endl;


        /*
        std::cout << "++++++++++++++++++++++++++++++++++++++++ " << endl;
        std::cout << "w1 = " << w1(0,0) << " " << w1(1,0) << " " << w1(2,0) << " " << endl;
        std::cout << "w2 = " << w2(0,0) << " " << w2(1,0) << " " << w2(2,0) << " " << endl;
        std::cout << "wM = " << wM(0,0) << " " << wM(1,0) << " " << wM(2,0) << " " << endl;
        std::cout << "++++++++++++++++++++++++++++++++++++++++ " << endl;
        */

        if ((w_norm.lpNorm<2>() > tolerance) and (nrec < 10))
        {
        /*
            std::cout << "nrec= " << nrec << endl;
            std::cout << "q1 = " << q1.transpose() << endl;
            std::cout << "q2 = " << q2.transpose() << endl;
            std::cout << "norm = " << w_norm.lpNorm<2>() << endl;
        */

            q_final_1_1 = taylorPath(w1, wM, tolerance, nrec+1);

            // REMOVE LAST COLUMN FROM q
            q_final_1 = Eigen::MatrixXd(q_final_1_1.rows(), q_final_1_1.cols()-1);
            for (int i = 0; i < q_final_1.rows(); i++)
                for (int j = 0; j < q_final_1_1.cols()-1; j++)
                    q_final_1(i, j) = q_final_1_1(i, j);

            q_final_2 = taylorPath(wM, w2, tolerance, nrec+1);

            //std::cout << q_final_1 << endl;
            //std::cout << q_final_2 << endl;

            q_final = Eigen::MatrixXd::Zero(6, q_final_1.cols() + q_final_2.cols());

            q_final << q_final_1, q_final_2;
            /*
            std::cout << q_final_1.size() <<endl;
            std::cout << q_final_2.size() << endl;
            std::cout << q_final.size() << endl;
            */
        }
        else
        {
            return q_final;
        }

    }

    //Eigen::MatrixXd dummyReturn;
    //dummyReturn = Eigen::MatrixXd::Zero(6, 1);
    //std::cout << "Taylor, problem happend. DUMMY RETURN." << endl;

    //return q_final_1;
}

double lwa4p_kinematics::wrapToPi(double angle)
{

    return angle - floor(angle/(2*M_PI) + 0.5)*2*M_PI;


}

double lwa4p_kinematics::sign(double a)
{

    if (a > 0)
        return 1;
    else if (a < 0)
        return -1;
    else
        return 0;

}
