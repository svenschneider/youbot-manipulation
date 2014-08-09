#include <youbot_arm_kinematics_moveit/youbot_arm_kinematics_moveit.h>

#include <pluginlib/class_list_macros.h>


PLUGINLIB_EXPORT_CLASS(youbot_arm_kinematics_moveit::KinematicsPlugin,
        kinematics::KinematicsBase);


using namespace youbot_arm_kinematics_moveit;


KinematicsPlugin::KinematicsPlugin()
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
    return false;
}


bool KinematicsPlugin::getPositionIK(const geometry_msgs::Pose &ik_pose,
        const std::vector<double> &ik_seed_state,
        std::vector<double> &solution,
        moveit_msgs::MoveItErrorCodes &error_code,
        const kinematics::KinematicsQueryOptions &options) const
{
    return false;
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
    static std::vector<std::string> joint_names;

    return joint_names;
}


const std::vector<std::string> &KinematicsPlugin::getLinkNames() const
{
    static std::vector<std::string> link_names;

    return link_names;
}
