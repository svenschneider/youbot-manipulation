#include <youbot_arm_kinematics/solver_info_processor.h>


SolverInfoProcessor::SolverInfoProcessor(const urdf::Model &robot_model,
	const std::string &tip_name,
	const std::string &root_name)
{
	// setup the IK solver information which contains joint names, joint limits and so on
	boost::shared_ptr<const urdf::Link> link = robot_model.getLink(tip_name);
	while (link) {
		// check if we have already reached the final link
		if (link->name == root_name) break;

		// if we have reached the last joint the root frame is not in the chain
		// then we cannot build the chain
		if (!link->parent_joint) {
			ROS_ERROR("The provided root link is not in the chain");
			ROS_ASSERT(false);
		}

		// process the joint
		boost::shared_ptr<const urdf::Joint> joint = robot_model.getJoint(link->parent_joint->name);
		if (!joint) {
			ROS_ERROR("Could not find joint: %s", link->parent_joint->name.c_str());
			ROS_ASSERT(false);
		}

		// add the joint to the chain
		if (joint->type != urdf::Joint::UNKNOWN && joint->type != urdf::Joint::FIXED) {
			ROS_DEBUG("Joint axis: %f, %f, %f", joint->axis.x, joint->axis.y, joint->axis.z);

			addJointToChainInfo(link->parent_joint, _solver_info);
		}

		link = robot_model.getLink(link->getParent()->name);
        }

	_solver_info.link_names.push_back(tip_name);

	// We expect order from root to tip, so reverse the order
	std::reverse(_solver_info.limits.begin(), _solver_info.limits.end());
	std::reverse(_solver_info.joint_names.begin(), _solver_info.joint_names.end());
	std::reverse(_solver_info.link_names.begin(), _solver_info.link_names.end());
}


SolverInfoProcessor::~SolverInfoProcessor()
{
}


kinematics_msgs::KinematicSolverInfo SolverInfoProcessor::getSolverInfo() const
{
	return _solver_info;
}


void SolverInfoProcessor::addJointToChainInfo(boost::shared_ptr<const urdf::Joint> joint, kinematics_msgs::KinematicSolverInfo &info)
{
	motion_planning_msgs::JointLimits limit;
	info.joint_names.push_back(joint->name);//Joints are coming in reverse order

	if (joint->type != urdf::Joint::CONTINUOUS) {
		// use safety limits if available else use joint limits
		if (joint->safety) {
			limit.min_position = joint->safety->soft_lower_limit;
			limit.max_position = joint->safety->soft_upper_limit;
		} else {
			limit.min_position = joint->limits->lower;
			limit.max_position = joint->limits->upper;
		}
		limit.has_position_limits = true;
	} else {
		limit.min_position = -M_PI;
		limit.max_position = M_PI;
		limit.has_position_limits = false;
	}

	limit.max_velocity = joint->limits->velocity;
	limit.has_velocity_limits = 1;
	limit.joint_name = joint->name;
	info.limits.push_back(limit);
}

