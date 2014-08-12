#include <youbot_arm_kinematics_moveit/youbot_arm_kinematics_moveit.h>
#include <youbot_arm_kinematics_moveit/configuration_comparator.h>

#include <pluginlib/class_list_macros.h>
#include <tf_conversions/tf_kdl.h>

#include <algorithm>


PLUGINLIB_EXPORT_CLASS(youbot_arm_kinematics_moveit::KinematicsPlugin,
        kinematics::KinematicsBase);


using namespace youbot_arm_kinematics_moveit;


KinematicsPlugin::KinematicsPlugin() : NUM_JOINTS(5)
{
}


KinematicsPlugin::~KinematicsPlugin()
{
}


bool KinematicsPlugin::initialize(const std::string &robot_description,
        const std::string &group_name,
        const std::string &base_frame,
        const std::string &tip_frame,
        double search_discretization)
{
    setValues(robot_description, group_name, base_frame, tip_frame,
            search_discretization);

    urdf::Model robot_model;
    if (!robot_model.initParam(robot_description)) return false;

    std::vector<double> lower_limits;
    std::vector<double> upper_limits;
    if (!extractKinematicData(robot_model, base_frame, tip_frame,
            joint_names_, link_names_, lower_limits, upper_limits)) {
        return false;
    }

    // Validate that the correct number of joints has been extracted
    if (joint_names_.size() != NUM_JOINTS) return false;

    ik_.reset(new youbot_arm_kinematics::InverseKinematics(lower_limits,
            upper_limits, kinematics_logger_));

    return true;
}


bool KinematicsPlugin::getPositionIK(const geometry_msgs::Pose &ik_pose,
        const std::vector<double> &ik_seed_state,
        std::vector<double> &solution,
        moveit_msgs::MoveItErrorCodes &error_code,
        const kinematics::KinematicsQueryOptions &options) const
{
    // Check if the initialize function has already been called successfully
    if (!ik_) return false;

    // Validate that there is one seed value per joint
    if (ik_seed_state.size() != NUM_JOINTS) return false;


    // Convert the ROS pose to a KDL frame
    KDL::Frame frame;
    tf::poseMsgToKDL(ik_pose, frame);

    // Convert the seed array to a KDL joint array
    KDL::JntArray seed = configurationStdToKdl(ik_seed_state);

    // Calculate the inverse position kinematics
    std::vector<KDL::JntArray> kdl_solutions;
    int res = ik_->CartToJnt(seed, frame, kdl_solutions);

    if (res <= 0) {
        error_code.val = moveit_msgs::MoveItErrorCodes::NO_IK_SOLUTION;
        return false;
    }

    // Convert the found solution from a KDL joint array to a vector of doubles
    std::vector<std::vector<double> > solutions(kdl_solutions.size());
    for (std::size_t i = 0; i < kdl_solutions.size(); i++) {
        solutions[i] = configurationKdlToStd(kdl_solutions[i]);
    }

    // Sort the solutions based on the distance to the seed state and return the
    // closest solution (i.e. the first one after sorting).
    ConfigurationComparator<double> comp(ik_seed_state);
    std::sort(solutions.begin(), solutions.end(), comp);
    solution = solutions[0];

    error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
    return true;
}


bool KinematicsPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
        const std::vector<double> &ik_seed_state,
        double timeout,
        std::vector<double> &solution,
        moveit_msgs::MoveItErrorCodes &error_code,
        const kinematics::KinematicsQueryOptions &options) const
{
    IKCallbackFn solution_callback = 0;
    std::vector<double> consistency_limits;

    return searchPositionIK(ik_pose, ik_seed_state, timeout, consistency_limits,
            solution, solution_callback, error_code);
}


bool KinematicsPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
        const std::vector<double> &ik_seed_state,
        double timeout,
        const std::vector<double> &consistency_limits,
        std::vector<double> &solution,
        moveit_msgs::MoveItErrorCodes &error_code,
        const kinematics::KinematicsQueryOptions &options) const
{
    IKCallbackFn solution_callback = 0;

    return searchPositionIK(ik_pose, ik_seed_state, timeout, consistency_limits,
            solution, solution_callback, error_code);
}


bool KinematicsPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
        const std::vector<double> &ik_seed_state,
        double timeout,
        std::vector<double> &solution,
        const IKCallbackFn &solution_callback,
        moveit_msgs::MoveItErrorCodes &error_code,
        const kinematics::KinematicsQueryOptions &options) const
{
    std::vector<double> consistency_limits;

    return searchPositionIK(ik_pose, ik_seed_state, timeout, consistency_limits,
            solution, solution_callback, error_code);
}


bool KinematicsPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
        const std::vector<double> &ik_seed_state,
        double timeout,
        const std::vector<double> &consistency_limits,
        std::vector<double> &solution,
        const IKCallbackFn &solution_callback,
        moveit_msgs::MoveItErrorCodes &error_code,
        const kinematics::KinematicsQueryOptions &options) const
{
    return getPositionIK(ik_pose, ik_seed_state, solution, error_code, options);
}


bool KinematicsPlugin::getPositionFK(const std::vector<std::string> &link_names,
        const std::vector<double> &joint_angles,
        std::vector<geometry_msgs::Pose> &poses) const
{
    return false;
}


const std::vector<std::string> &KinematicsPlugin::getJointNames() const
{
    return joint_names_;
}


const std::vector<std::string> &KinematicsPlugin::getLinkNames() const
{
    return link_names_;
}


bool KinematicsPlugin::extractKinematicData(const urdf::Model &robot_model,
        const std::string &base_frame,
        const std::string &tip_frame,
        std::vector<std::string> &joint_names,
        std::vector<std::string> &link_names,
        std::vector<double> &lower_limits,
        std::vector<double> &upper_limits) const
{
    boost::shared_ptr<urdf::Link> link = boost::const_pointer_cast<urdf::Link>(
            robot_model.getLink(tip_frame));

    while ((link) && (link->name != base_frame)) {
        link_names.push_back(link->name);
        boost::shared_ptr<urdf::Joint> joint = link->parent_joint;

        // Don't consider invalid, unknown or fixed joints
        if ((!joint) || (joint->type == urdf::Joint::UNKNOWN)
                || (joint->type == urdf::Joint::FIXED)) {
            // Continue with the next link in the kinematic chain
            link = link->getParent();

            continue;
        }

        joint_names.push_back(joint->name);

        // Extract the joint limits
        if (joint->type != urdf::Joint::CONTINUOUS) {
            if (joint->safety) {
                lower_limits.push_back(joint->safety->soft_lower_limit);
                upper_limits.push_back(joint->safety->soft_upper_limit);
            } else {
                lower_limits.push_back(joint->limits->lower);
                upper_limits.push_back(joint->limits->upper);
            }
        } else {
            lower_limits.push_back(-M_PI);
            upper_limits.push_back( M_PI);
        }

        // Continue with the next link in the kinematic chain
        link = link->getParent();
    }

    // The kinematic chain ended and the base frame was not found
    if (!link) return false;

    // The data has been extracted from the tip to the base, but it is required
    // the other way round
    std::reverse(link_names.begin(), link_names.end());
    std::reverse(joint_names.begin(), joint_names.end());
    std::reverse(lower_limits.begin(), lower_limits.end());
    std::reverse(upper_limits.begin(), upper_limits.end());

    return true;
}


KDL::JntArray KinematicsPlugin::configurationStdToKdl(
        const std::vector<double> &v) const
{
    KDL::JntArray jnt(v.size());

    for (std::size_t i = 0; i < v.size(); i++) {
        jnt(i) = v[i];
    }

    return jnt;
}


std::vector<double> KinematicsPlugin::configurationKdlToStd(
        const KDL::JntArray &jnt) const
{
    std::vector<double> v(jnt.rows());

    for (int i = 0; i < jnt.rows(); i++) {
        v[i] = jnt(i);
    }

    return v;
}
