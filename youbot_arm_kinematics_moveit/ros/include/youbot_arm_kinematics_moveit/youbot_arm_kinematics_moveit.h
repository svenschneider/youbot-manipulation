#ifndef YOUBOT_ARM_KINEMATICS_MOVEIT_H
#define YOUBOT_ARM_KINEMATICS_MOVEIT_H

#include <moveit/kinematics_base/kinematics_base.h>


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
};

}

#endif
