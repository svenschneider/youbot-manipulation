#include <moveit/kinematics_base/kinematics_base.h>
#include <pluginlib/class_loader.h>
#include <gtest/gtest.h>


const std::string PLUGIN = "youbot_arm_kinematics_moveit::KinematicsPlugin";
typedef boost::shared_ptr<kinematics::KinematicsBase> KinematicsBasePtr;


TEST(load_moveit_plugin_test, load_plugin)
{
    pluginlib::ClassLoader<kinematics::KinematicsBase> loader(
            "moveit_core", "kinematics::KinematicsBase");

    EXPECT_FALSE(loader.isClassLoaded(PLUGIN));

    KinematicsBasePtr kinematics;
    EXPECT_NO_THROW(kinematics = loader.createInstance(PLUGIN));

    EXPECT_TRUE(loader.isClassLoaded(PLUGIN));
    EXPECT_TRUE(kinematics);
}


TEST(load_moveit_plugin_test, empty_joint_and_link_names)
{
    pluginlib::ClassLoader<kinematics::KinematicsBase> loader(
            "moveit_core", "kinematics::KinematicsBase");
    KinematicsBasePtr kinematics = loader.createInstance(PLUGIN);

    EXPECT_TRUE(kinematics->getJointNames().empty());
    EXPECT_TRUE(kinematics->getLinkNames().empty());
}


int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
