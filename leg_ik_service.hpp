#ifndef LEG_IK_SERVICE_HPP_
#define LEG_IK_SERVICE_HPP_
#include <ros/ros.h>
#include <kdl_parser/kdl_parser.hpp>
#include "chainiksolvervel_pinv.hpp"
#include <kdl/chainfksolverpos_recursive.hpp>
#include "hp_chainiksolverpos_nr_jl.hpp"
#include <hexapod_msgs/GetLegIKSolver.h>

#define NUM_LEGS 6
#define NUM_JOINTS 3

classLegKinematics {
	public:
		LegKinematics();
		boolinit();
	private:
		ros::NodeHandle node, node_private;
		std::string root_name, tip_name;
		doublejoint_lower_limit, joint_upper_limit;
		const static unsigned intnum_joints = NUM_JOINTS;
		const static unsigned intnum_legs = NUM_LEGS;

		KDL::Chain* chains_ptr[6];
		KDL::JntArrayjoint_min, joint_max;
		KDL::ChainFkSolverPos_recursive* fk_solver[6];
		KDL::HP_ChainIkSolverPos_NR_JL* ik_solver_pos[6];
		KDL::ChainIkSolverVel_pinv* ik_solver_vel[6];

		ros::ServiceServerik_service;

		boolloadModel(conststd::string xml);
		boolgetLegIKSolver (	hexapod_msgs::GetLegIKSolver::Request &request,
								hexapod_msgs::GetLegIKSolver::Response &response);
};

#endif /* LEG_IK_SERVICE_HPP_ */
