#include <youbot_arm_kinematics/inverse_kinematics.h>

#include <iostream>
#include <sstream>


using namespace youbot_arm_kinematics;


#define DEG_TO_RAD(x) ((x) * M_PI / 180.0)

const double InverseKinematics::ALMOST_PLUS_ONE = 0.9999999;
const double InverseKinematics::ALMOST_MINUS_ONE = -0.9999999;


InverseKinematics::InverseKinematics(
        const std::vector<double> &min_angles,
        const std::vector<double> &max_angles,
        Logger &logger) : logger_(logger)
{
    min_angles_ = min_angles;
    max_angles_ = max_angles;
}


InverseKinematics::~InverseKinematics()
{
}


int InverseKinematics::CartToJnt(const KDL::JntArray &q_init,
        const KDL::Frame &p_in,
        std::vector<KDL::JntArray> &q_out)
{
    KDL::JntArray solution;
    bool bools[] = { true, false };

    // there are no solutions available yet
    q_out.clear();


    // iterate over all redundant solutions
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 2; j++) {
            solution = ik(p_in, bools[i], bools[j]);
            if (isSolutionValid(solution)) q_out.push_back(solution);
        }
    }

    if (q_out.size() > 0) {
        logger_.write("Inverse kinematics found a solution",
                __FILE__, __LINE__);

        return 1;
    } else {
        logger_.write("Inverse kinematics found no solution",
                __FILE__, __LINE__);

        return -1;
    }
}


KDL::JntArray InverseKinematics::ik(const KDL::Frame& g0,
        bool offset_joint_1, bool offset_joint_3)
{
    // Parameters from youBot URDF file
    double l0x = 0.024;
    double l0z = 0.096;
    double l1x = 0.033;
    double l1z = 0.019;
    double l2 = 0.155;
    double l3 = 0.135;

    // Distance from arm_link_4 to arm_link_5
    double d = 0.13;

    double j1 = 0.0;
    double j2 = 0.0;
    double j3 = 0.0;
    double j4 = 0.0;
    double j5 = 0.0;


    // Transform from frame 0 to frame 1
    KDL::Frame frame0_to_frame1(
            KDL::Rotation::Identity(),
            KDL::Vector(-l0x, 0, -l0z));
    KDL::Frame g1 = frame0_to_frame1 * g0;

    // First joint
    j1 = atan2(g1.p.y(), g1.p.x());
    if (offset_joint_1) {
        if (j1 < 0.0) j1 += M_PI;
        else j1 -= M_PI;
    }


    // Transform from frame 1 to frame 2
    KDL::Frame frame1_to_frame2(
            KDL::Rotation::RPY(0, 0, -j1),
            KDL::Vector(-l1x, 0, -l1z));
    KDL::Frame g2 = frame1_to_frame2 * g1;

    // Project the frame into the plane of the arm
    KDL::Frame g2_proj = projectGoalOrientationIntoArmSubspace(g2);

    // Set all values in the frame that are close to zero to exactly zero
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            if (fabs(g2_proj.M(i, j)) < 0.000001) g2_proj.M(i, j) = 0;
        }
    }

    // Fifth joint, determines the roll of the gripper (= wrist angle)
    double s1 = sin(j1);
    double c1 = cos(j1);
    double r11 = g1.M(0, 0);
    double r12 = g1.M(0, 1);
    double r21 = g1.M(1, 0);
    double r22 = g1.M(1, 1);
    j5 = atan2(r21 * c1 - r11 * s1, r22 * c1 - r12 * s1);

    // The sum of joint angles two to four determines the overall "pitch" of the
    // end effector
    double r13 = g2_proj.M(0, 2);
    double r33 = g2_proj.M(2, 2);
    double j234 = atan2(r13, r33);


    KDL::Vector p2 = g2_proj.p;

    // In the arm's subplane, offset from the end-effector to the fourth joint
    p2.x(p2.x() - d * sin(j234));
    p2.z(p2.z() - d * cos(j234));


    // Check if the goal position can be reached at all
    if ((l2 + l3) < sqrt((p2.x() * p2.x()) + (p2.z() * p2.z()))) {
        return KDL::JntArray();
    }

    // Third joint
    double l_sqr = (p2.x() * p2.x()) + (p2.z() * p2.z());
    double l2_sqr = l2 * l2;
    double l3_sqr = l3 * l3;
    double j3_cos = (l_sqr - l2_sqr - l3_sqr) / (2.0 * l2 * l3);

    if (j3_cos > ALMOST_PLUS_ONE) j3 = 0.0;
    else if (j3_cos < ALMOST_MINUS_ONE) j3 = M_PI;
    else j3 = atan2(sqrt(1.0 - (j3_cos * j3_cos)), j3_cos);

    if (offset_joint_3) j3 = -j3;


    // Second joint
    double t1 = atan2(p2.z(), p2.x());
    double t2 = atan2(l3 * sin(j3), l2 + l3 * cos(j3));
    j2 = M_PI_2 - t1 - t2;


    // Fourth joint, determines the pitch of the gripper
    j4 = j234 - j2 - j3;


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

    std::stringstream sstr;
    sstr << "Configuration without offsets: "
            << j1 << ", "
            << j2 << ", "
            << j3 << ", "
            << j4 << ", "
            << j5 << std::endl;
    sstr << "Configuration with offsets: "
            << solution(0) << ", "
            << solution(1) << ", "
            << solution(2) << ", "
            << solution(3) << ", "
            << solution(4);
    logger_.write(sstr.str(), __FILE__, __LINE__);

    return solution;
}


KDL::Frame InverseKinematics::projectGoalOrientationIntoArmSubspace(
        const KDL::Frame &goal) const
{
    KDL::Vector y_t_hat = goal.M.UnitY();   // y vector of the rotation matrix
    KDL::Vector z_t_hat = goal.M.UnitZ();   // z vector of the rotation matrix

    // m_hat is the normal of the "arm plane"
    KDL::Vector m_hat(0, -1, 0);

    // k_hat is the vector about which rotation of the goal frame is performed
    KDL::Vector k_hat = m_hat * z_t_hat;        // cross product

    // z_t_hat_tick is the new pointing direction of the arm
    KDL::Vector z_t_hat_tick = k_hat * m_hat;   // cross product

    // the amount of rotation
    double cos_theta = KDL::dot(z_t_hat, z_t_hat_tick);
    // first cross product then dot product
    double sin_theta = KDL::dot(z_t_hat * z_t_hat_tick, k_hat);

    // use Rodrigues' rotation formula to perform the rotation
    KDL::Vector y_t_hat_tick = (cos_theta * y_t_hat)
            // k_hat * y_t_hat is cross product
            + (sin_theta * (k_hat * y_t_hat)) + (1 - cos_theta)
            * (KDL::dot(k_hat, y_t_hat)) * k_hat;
    KDL::Vector x_t_hat_tick = y_t_hat_tick * z_t_hat_tick; // cross product

    KDL::Rotation rot(x_t_hat_tick, y_t_hat_tick, z_t_hat_tick);

    // the frame uses the old position but has the new, projected orientation
    return KDL::Frame(rot, goal.p);
}


bool InverseKinematics::isSolutionValid(const KDL::JntArray &solution) const
{
    bool valid = true;

    if (solution.rows() != 5) return false;

    for (unsigned int i = 0; i < solution.rows(); i++) {
        if ((solution(i) < min_angles_[i]) || (solution(i) > max_angles_[i])) {
            valid = false;
        }
    }

    return valid;
}
