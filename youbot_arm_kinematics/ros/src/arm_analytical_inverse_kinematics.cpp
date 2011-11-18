#include <youbot_arm_kinematics/arm_analytical_inverse_kinematics.h>

// ROS includes
#include <pr2_arm_kinematics/pr2_arm_kinematics_utils.h>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>

// Package includes
#include <youbot_arm_kinematics/solver_info_processor.h>


#define DEG_TO_RAD(x) ((x) * M_PI / 180.0)

const double ArmAnalyticalInverseKinematics::ALMOST_PLUS_ONE = 0.9999999;
const double ArmAnalyticalInverseKinematics::ALMOST_MINUS_ONE = -0.9999999;


ArmAnalyticalInverseKinematics::ArmAnalyticalInverseKinematics(const urdf::Model &robot_model,
		const std::string &robot_description,
		const std::string &root_name,
		const std::string &tip_name)
{
	SolverInfoProcessor solver_info_processor(robot_model, tip_name, root_name);
	_solver_info = solver_info_processor.getSolverInfo();

	for (unsigned int i = 0; i < _solver_info.joint_names.size(); i++) {
		_min_angles.push_back(_solver_info.limits[i].min_position);
		_max_angles.push_back(_solver_info.limits[i].max_position);
	}
}


ArmAnalyticalInverseKinematics::~ArmAnalyticalInverseKinematics()
{
}


int ArmAnalyticalInverseKinematics::CartToJnt(const KDL::JntArray& q_init,
		const KDL::Frame& p_in,
		std::vector<KDL::JntArray>& q_out)
{
	KDL::JntArray solution;
	bool bools[] = { true, false };

	// there are no solutions available yet
	q_out.clear();


	// iterate over all redundant solutions
	for (int i = 0; i < 2; i++) {
		for (int j = 0; j < 2; j++) {
			for (int k = 0; k < 2; k++) {
				solution = ik(p_in, bools[i], bools[j], bools[k]);
				if (isSolutionValid(solution)) q_out.push_back(solution);
			}
		}
	}

	if (q_out.size() > 0) {
		ROS_DEBUG("Inverse Kinematic found a solution");

		return 1;
	} else {
		ROS_DEBUG("Inverse Kinematic found no solution.");

		return -1;
	}
}


void ArmAnalyticalInverseKinematics::getSolverInfo(kinematics_msgs::KinematicSolverInfo &info)
{
	info = _solver_info;
}


KDL::JntArray ArmAnalyticalInverseKinematics::ik(const KDL::Frame& g0,
		bool offset_joint_1, bool offset_joint_3, bool offset_joint_5)
{
	// Parameters from youBot URDF file
	double l0x = 0.024;
	double l0z = 0.096;
	double l1x = 0.033;
	double l1z = 0.019;
	double l2 = 0.155;
	double l3 = 0.135;

	// Distance from arm_link_3 to arm_link_5 (can also be replaced by e.g.
	// distance from arm_link_3 to tool center point)
	double d = 0.13;

	double j1;
	double j2;
	double j3;
	double j4;
	double j5;



	// Transform position to the first joint
	KDL::Vector p1;
	p1.x(g0.p.x() - l0x);	// offset to front
	p1.y(g0.p.y());		// no offset to side
	p1.z(g0.p.z() - l0z);	// offset to above

	// First joint
	j1 = atan2(p1.y(), p1.x());
	if (offset_joint_1) {
		if (j1 < 0.0) j1 += M_PI;
		else j1 -= M_PI;
	}



	// Transform position to the second joint
	KDL::Vector p2;
	p2.x(p1.x() * cos(j1) + p1.y() * sin(j1));
	// offset the x value depending on orientation of first link
	p2.x(p2.x() - l1x);
	// no offset in the y-value
	p2.y(p1.x() * sin(j1) + p1.y() * cos(j1));
	p2.z(p1.z() - l1z);


	// Combine the position of the goal in the second joint with the orginal
	// orientation of the goal
	KDL::Frame g2_tmp(g0.M, p2);

	// Project the frame into the plane of the arm
	KDL::Frame g2 = ProjectGoalOrientationIntoArmSubspace(g2_tmp);

	// Set all values in the frame that are close to zero to exactly zero
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			if (abs(g2.M(i, j)) < 0.000001) g2.M(i, j) = 0;
		}
	}

	// Fifth joint, determines the roll of the gripper (= wrist angle)
	// The joint can either be oriented according to the calculated angle or
	// offset from the angle by +Pi or -Pi.
	// To choose between +Pi and -Pi we consider the angle in order to stay
	// within the joint limits:
	// * if the angle is greater than zero use -Pi
	// * if the angle is less than or equal to zero use +Pi
	double s1 = sin(j1);
	double c1 = cos(j1);
	double r11 = g2.M(0, 0);
	double r12 = g2.M(0, 1);
	double r21 = g2.M(1, 0);
	double r22 = g2.M(1, 1);
	j5 = atan2(r21 * c1 - r11 * s1, r22 * c1 - r12 * s1);
	if (offset_joint_5) {
		if (j5 > 0.0) j5 -= M_PI;
		else j5 += M_PI;
	}

	// The sum of joint angles two to four determines the overall "pitch" of the
	// end effector
	double r13 = g2.M(0, 2);
	double r23 = g2.M(1, 2);
	double r33 = g2.M(2, 2);
	double j234 = atan2(r13 * c1 + r23 * s1, r33);

	if (!offset_joint_1) {
		p2.x(p2.x() - d * cos(j234));
		p2.z(p2.z() + d * sin(j234));
	} else {
		p2.x(p2.x() + d * cos(j234));
		p2.z(p2.z() - d * sin(j234));
	}


	// Check if the goal position can be reached at all
	if ((l2 + l3) < sqrt((p2.x() * p2.x()) + (p2.z() * p2.z()))) {
		return KDL::JntArray();
	}

	// Third joint
	double j3_cos = ((p2.x() * p2.x()) + (p2.z() * p2.z()) - (l2 * l2) - (l3 * l3)) / (2 * l2 * l3);
	if (j3_cos > ALMOST_PLUS_ONE) j3 = 0.0;
	else if (j3_cos < ALMOST_MINUS_ONE) j3 = M_PI;
	else j3 = atan2(sqrt(1 - (j3_cos * j3_cos)), j3_cos);
	if (offset_joint_3) j3 = -j3;

	// Second joint
	if (j3 >= 0) j2 = -atan2(p2.z(), p2.x()) - atan2(l3 * sin(j3), l2 + l3 * cos(j3));
	else j2 = -atan2(p2.z(), p2.x()) + atan2(l3 * sin(j3), l2 + l3 * cos(j3));
	j2 += M_PI_2;


	// Fourth joint, determines the pitch of the gripper
	j4 = j234 - (j2 - M_PI_2) - j3;
	if (j4 > M_PI) j4 -= 2 * M_PI;



	// This IK assumes that the arm points upwards, so we need to consider
	// the offsets to the real home position
	double offset1 = DEG_TO_RAD( 169.0);
	double offset2 = DEG_TO_RAD(  65.0);
	double offset3 = DEG_TO_RAD(-146.0);
	double offset4 = DEG_TO_RAD( 102.5);
	double offset5 = DEG_TO_RAD( 167.5);

	KDL::JntArray solution(5);
	solution(0) = offset1 - j1;
	solution(1) = j2 + offset2;
	solution(2) = j3 + offset3;
	solution(3) = j4 + offset4;
	solution(4) = offset5 - j5;

	ROS_INFO("Configuration without offsets: %f, %f, %f, %f, %f", j1, j2, j3, j4, j5);
	ROS_INFO("Configuration with offsets: %f, %f, %f, %f, %f", solution(0), solution(1), solution(2), solution(3), solution(4));

	return solution;
}


KDL::Frame ArmAnalyticalInverseKinematics::ProjectGoalOrientationIntoArmSubspace(const KDL::Frame &goal) const
{
	double x = goal.p.x();
	double y = goal.p.y();
	KDL::Vector y_t_hat = goal.M.UnitY();							// y vector of the rotation matrix
	KDL::Vector z_t_hat = goal.M.UnitZ();							// z vector of the rotation matrix

	// m_hat is the normal of the "arm plane"
	KDL::Vector m_hat(0, -1, 0);

	// k_hat is the vector about which rotation of the goal frame is performed
	KDL::Vector k_hat = m_hat * z_t_hat;							// cross product

	// z_t_hat_tick is the new pointing direction of the arm
	KDL::Vector z_t_hat_tick = k_hat * m_hat;						// cross product

	// the amount of rotation
	double cos_theta = KDL::dot(z_t_hat, z_t_hat_tick);
	double sin_theta = KDL::dot(z_t_hat * z_t_hat_tick, k_hat);		// first cross product then dot product

	// use Rodrigues' rotation formula to perform the rotation
	KDL::Vector y_t_hat_tick = (cos_theta * y_t_hat) + (sin_theta * (k_hat * y_t_hat)) + (1 - cos_theta) * (KDL::dot(k_hat, y_t_hat)) * k_hat;	// k_hat * y_t_hat is cross product
	KDL::Vector x_t_hat_tick = y_t_hat_tick * z_t_hat_tick;			// cross product

	KDL::Rotation rot(x_t_hat_tick, y_t_hat_tick, z_t_hat_tick);

	// the frame uses the old position but has the new, projected orientation
	return KDL::Frame(rot, goal.p);
}


bool ArmAnalyticalInverseKinematics::isSolutionValid(const KDL::JntArray &solution) const
{
	bool valid = true;

	if (solution.rows() != 5) return false;

	for (unsigned int i = 0; i < solution.rows(); i++) {
		if ((solution(i) < _min_angles[i]) || (solution(i) > _max_angles[i])) {
			valid = false;
		}
	}

	return valid;
}
