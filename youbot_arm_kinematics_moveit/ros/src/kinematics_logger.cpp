#include <youbot_arm_kinematics_moveit/kinematics_logger.h>

#include <ros/console.h>


KinematicsLogger::~KinematicsLogger()
{
}


void KinematicsLogger::write(const std::string &msg, const char *file, int line)
{
    ROSCONSOLE_DEFINE_LOCATION(true, ros::console::levels::Debug,
            ROSCONSOLE_ROOT_LOGGER_NAME ".youbot_arm_kinematics");

    if (__rosconsole_define_location__loc.logger_enabled_) {
        ros::console::print(0, __rosconsole_define_location__loc.logger_,
                ros::console::levels::Debug, file, line, "", "%s", msg.c_str());
    }
}
