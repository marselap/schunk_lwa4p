#include "ros/ros.h"
//#include "ros/console.h"
#include "yaml-cpp/yaml.h"
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <ros/package.h>
#include <string>
#include <math.h>
#include <eigen3/Eigen/Eigen>

using namespace std;


class lwa4p_kinematics{

	public:
		lwa4p_kinematics();
		void loadParameters(int robot_id, std::string configFile);
		Eigen::MatrixXd directKinematics(Eigen::MatrixXd joint_state, int end_joint);
		Eigen::MatrixXd inverseKinematics(Eigen::MatrixXd goal_w);
		Eigen::MatrixXd inverseKinematics_closestQ(Eigen::MatrixXd goal_w, Eigen::MatrixXd temp_q);
		Eigen::MatrixXd taylorPath(Eigen::MatrixXd w1, Eigen::MatrixXd w2, double tolerance, int nrec);
		Eigen::MatrixXd taylorPathCheck(Eigen::MatrixXd w1, Eigen::MatrixXd w2, Eigen::MatrixXd q0, double tolerance);
		
		double wrapToPi(double angle);
		double sign(double a);

		// robot position
		double robot_pose_x;
		double robot_pose_y;
		double robot_pose_theta;

		double temp_inverse_result[6];

		

	private:
		double temp;	
		double eps, eps_pose, eps_orientation;	

		


		// DH parameters
		double dh_param_d[6];
		double dh_param_a[6];
		double dh_param_alpha[6];
		double dh_param_Q0[6];
		double joint_limits_min[6], joint_limits_max[6];

		// Direct kinematics
		double direct_qIn[6];
		double direct_q[6];
		double direct_w[9];


		// Inverse kinematics
		
		bool valid_results[8], temp_outlier; 
		
		int inverse_solutions, temp_solution;
		Eigen::MatrixXcd r1, r2, r3;
		Eigen::MatrixXcd p1, p2;
		Eigen::MatrixXcd temp_i_1, temp_i_2;
		Eigen::MatrixXcd inverse_Q; 
		Eigen::MatrixXd temp_mat, inverse_Q_final;

		Eigen::MatrixXcd norm_vect, goal_vect, temp_w;
	  	double *direct_check_w, direct_check_q[6];
	  	

	  	

		// Inverse kinematics closest Q
		Eigen::MatrixXcd eigen_tempQ, ikc_temp;
		double d_min, d;
		int k_min, k, IK;
		double closest_Q[6];



};