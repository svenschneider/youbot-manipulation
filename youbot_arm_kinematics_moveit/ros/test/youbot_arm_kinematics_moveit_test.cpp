#include <moveit/kinematics_base/kinematics_base.h>
#include <pluginlib/class_loader.h>
#include <ros/ros.h>
#include <gtest/gtest.h>


const std::string PLUGIN = "youbot_arm_kinematics_moveit::KinematicsPlugin";
typedef boost::shared_ptr<kinematics::KinematicsBase> KinematicsBasePtr;


TEST(youbot_arm_kinematics_moveit, wrong_arguments_fail_init)
{
    pluginlib::ClassLoader<kinematics::KinematicsBase> loader(
            "moveit_core", "kinematics::KinematicsBase");
    KinematicsBasePtr kinematics = loader.createInstance(PLUGIN);

    EXPECT_FALSE(kinematics->initialize("", "", "", "", 0.0));
}


TEST(youbot_arm_kinematics_moveit, init_sets_values)
{
    pluginlib::ClassLoader<kinematics::KinematicsBase> loader(
            "moveit_core", "kinematics::KinematicsBase");
    KinematicsBasePtr kinematics = loader.createInstance(PLUGIN);

    EXPECT_TRUE(kinematics->initialize("/robot_description", "arm_1",
            "arm_link_0", "arm_link_5", 0.1));

    EXPECT_EQ("arm_1", kinematics->getGroupName());
    EXPECT_EQ("arm_link_0", kinematics->getBaseFrame());
    EXPECT_EQ("arm_link_5", kinematics->getTipFrame());
    EXPECT_NEAR(0.1, kinematics->getSearchDiscretization(), 0.001);
}


TEST(youbot_arm_kinematics_moveit, wrong_kinematics_fails_init)
{
    pluginlib::ClassLoader<kinematics::KinematicsBase> loader(
            "moveit_core", "kinematics::KinematicsBase");
    KinematicsBasePtr kinematics = loader.createInstance(PLUGIN);

    EXPECT_FALSE(kinematics->initialize("/robot_description", "", "arm_link_1",
            "arm_link_5", 0.0));
}


TEST(youbot_arm_kinematics_moveit, wrong_base_and_tip_frame_fails_init)
{
    pluginlib::ClassLoader<kinematics::KinematicsBase> loader(
            "moveit_core", "kinematics::KinematicsBase");
    KinematicsBasePtr kinematics = loader.createInstance(PLUGIN);

    EXPECT_FALSE(kinematics->initialize("/robot_description", "", "arm_link_5",
            "arm_link_1", 0.0));
}


TEST(youbot_arm_kinematics_moveit, joint_and_link_names_not_empty_after_init)
{
    pluginlib::ClassLoader<kinematics::KinematicsBase> loader(
            "moveit_core", "kinematics::KinematicsBase");
    KinematicsBasePtr kinematics = loader.createInstance(PLUGIN);

    EXPECT_TRUE(kinematics->getJointNames().empty());
    EXPECT_TRUE(kinematics->getLinkNames().empty());

    EXPECT_TRUE(kinematics->initialize("/robot_description", "arm_1",
            "arm_link_0", "arm_link_5", 0.1));

    EXPECT_EQ(5, kinematics->getJointNames().size());
    EXPECT_EQ(5, kinematics->getLinkNames().size());

    EXPECT_EQ("arm_joint_1", kinematics->getJointNames()[0]);
    EXPECT_EQ("arm_joint_2", kinematics->getJointNames()[1]);
    EXPECT_EQ("arm_joint_3", kinematics->getJointNames()[2]);
    EXPECT_EQ("arm_joint_4", kinematics->getJointNames()[3]);
    EXPECT_EQ("arm_joint_5", kinematics->getJointNames()[4]);

    EXPECT_EQ("arm_link_1", kinematics->getLinkNames()[0]);
    EXPECT_EQ("arm_link_2", kinematics->getLinkNames()[1]);
    EXPECT_EQ("arm_link_3", kinematics->getLinkNames()[2]);
    EXPECT_EQ("arm_link_4", kinematics->getLinkNames()[3]);
    EXPECT_EQ("arm_link_5", kinematics->getLinkNames()[4]);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "youbot_arm_kinematics_moveit_test");

    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
