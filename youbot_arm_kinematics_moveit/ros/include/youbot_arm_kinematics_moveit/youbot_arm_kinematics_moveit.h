#ifndef YOUBOT_ARM_KINEMATICS_MOVEIT_H
#define YOUBOT_ARM_KINEMATICS_MOVEIT_H

#include <boost/shared_ptr.hpp>

#include <moveit/kinematics_base/kinematics_base.h>
#include <urdf/model.h>
#include <youbot_arm_kinematics/inverse_kinematics.h>

#include <youbot_arm_kinematics_moveit/kinematics_logger.h>


namespace youbot_arm_kinematics_moveit
{

class KinematicsPlugin : public kinematics::KinematicsBase
{
    public:
        /**
         * Ctor.
         */
        KinematicsPlugin();

        /**
         * Dtor.
         */
        virtual ~KinematicsPlugin();

        /**
         * @see kinematics::KinematicsBase::initialize
         */
        bool initialize(const std::string &robot_description,
                const std::string &group_name,
                const std::string &base_frame,
                const std::string &tip_frame,
                double search_discretization);

        /**
         * @see kinematics::KinematicsBase::getPositionIK
         */
        bool getPositionIK(const geometry_msgs::Pose &ik_pose,
                const std::vector<double> &ik_seed_state,
                std::vector<double> &solution,
                moveit_msgs::MoveItErrorCodes &error_code,
                const kinematics::KinematicsQueryOptions &options
                        = kinematics::KinematicsQueryOptions()) const;

        /**
         * @see kinematics::KinematicsBase::searchPositionIK
         */
        bool searchPositionIK(const geometry_msgs::Pose &ik_pose,
                const std::vector<double> &ik_seed_state,
                double timeout,
                std::vector<double> &solution,
                moveit_msgs::MoveItErrorCodes &error_code,
                const kinematics::KinematicsQueryOptions &options
                        = kinematics::KinematicsQueryOptions()) const;

        /**
         * @see kinematics::KinematicsBase::searchPositionIK
         */
        bool searchPositionIK(const geometry_msgs::Pose &ik_pose,
                const std::vector<double> &ik_seed_state,
                double timeout,
                const std::vector<double> &consistency_limits,
                std::vector<double> &solution,
                moveit_msgs::MoveItErrorCodes &error_code,
                const kinematics::KinematicsQueryOptions &options
                        = kinematics::KinematicsQueryOptions()) const;

        /**
         * @see kinematics::KinematicsBase::searchPositionIK
         */
        bool searchPositionIK(const geometry_msgs::Pose &ik_pose,
                const std::vector<double> &ik_seed_state,
                double timeout,
                std::vector<double> &solution,
                const IKCallbackFn &solution_callback,
                moveit_msgs::MoveItErrorCodes &error_code,
                const kinematics::KinematicsQueryOptions &options
                        = kinematics::KinematicsQueryOptions()) const;

        /**
         * @see kinematics::KinematicsBase::searchPositionIK
         */
        bool searchPositionIK(const geometry_msgs::Pose &ik_pose,
                const std::vector<double> &ik_seed_state,
                double timeout,
                const std::vector<double> &consistency_limits,
                std::vector<double> &solution,
                const IKCallbackFn &solution_callback,
                moveit_msgs::MoveItErrorCodes &error_code,
                const kinematics::KinematicsQueryOptions &options
                        = kinematics::KinematicsQueryOptions()) const;

        /**
         * @see kinematics::KinematicsBase::getPositionFK
         */
        bool getPositionFK(const std::vector<std::string> &link_names,
                const std::vector<double> &joint_angles,
                std::vector<geometry_msgs::Pose> &poses) const;

        /**
         * @see kinematics::KinematicsBase::getJointNames
         */
        const std::vector<std::string> &getJointNames() const;

        /**
         * @see kinematics::KinematicsBase::getLinkNames
         */
        const std::vector<std::string> &getLinkNames() const;


    private:
        /**
         * Copy ctor.
         */
        KinematicsPlugin(const KinematicsPlugin &other);

        /**
         * Assignment operator.
         */
        KinematicsPlugin &operator=(const KinematicsPlugin &other);

        /**
         * Given a URDF model of a robot, extract information such as joint
         * names or limits from it. The base and the tip frame must be connected
         * in the URDF model, else the function fails (returns false).
         *
         * @param robot_model The URDF representation of the robot.
         *
         * @param base_frame A link in the kinematic structure which is taken as
         * the base of the analyzed kinematic chain.
         *
         * @param tip_frame A link in the kinematic structure which is taken as
         * the tip of the analyzed kinematic chain.
         *
         * @param joint_names The extracted joint names. The variable is
         * modified in-place.
         *
         * @param link_names The extracted link names. The variable is modified
         * in-place.
         *
         * @param lower_limits The extracted lower limits of each joint. The
         * unit depends on the specification in the URDF model. The variable is
         * modified in-place.
         *
         * @param upper_limits The extracted upper limits of each joint. The
         * unit depends on the specification in the URDF model. The variable is
         * modified in-place.
         *
         * @return The function returns true on success and false otherwise.
         */
        bool extractKinematicData(const urdf::Model &robot_model,
                const std::string &base_frame,
                const std::string &tip_frame,
                std::vector<std::string> &joint_names,
                std::vector<std::string> &link_names,
                std::vector<double> &lower_limits,
                std::vector<double> &upper_limits) const;

        /**
         * Convert a joint configuration vector, in which each value is
         * represented as a double, to a KDL joint array. The opposite
         * conversion is implemented in the {@see configurationKdlToStd}
         * function.
         *
         * @param v The input configuration.
         *
         * @return The input configuration converted to a joint array.
         */
        KDL::JntArray configurationStdToKdl(const std::vector<double> &v) const;

        /**
         * Convert a KDL joint array to a joint configuration vector, in which
         * each value is represented as a double. The opposite conversion is
         * implemented in the {@see configurationStdToKdl} function.
         *
         * @param jnt The input joint configuration.
         *
         * @return The converted joint configuration vector.
         */
        std::vector<double> configurationKdlToStd(const KDL::JntArray &jnt)
                const;


    private:
        /**
         * The number of joints for which the inverse kinematics can solve.
         */
        const std::size_t NUM_JOINTS;

        /**
         * The joints on which the kinematics plugin is working.
         */
        std::vector<std::string> joint_names_;

        /**
         * The links in the kinematic chain between the base frame and the tip
         * frame (as provided to the @see KinematicsPlugin::initialize
         * function).
         */
        std::vector<std::string> link_names_;

        /**
         * The inverse kinematics solver.
         */
        boost::shared_ptr<youbot_arm_kinematics::InverseKinematics> ik_;

        /**
         * The logger adapter for the kinematics package.
         */
        KinematicsLogger kinematics_logger_;
};

}

#endif
