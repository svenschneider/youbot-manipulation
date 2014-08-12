#include <youbot_arm_kinematics_moveit/kinematics_logger.h>

#include <ros/console.h>


KinematicsLogger::~KinematicsLogger()
{
}


void KinematicsLogger::write(const std::string &msg, const char *file, int line)
{
    ROSCONSOLE_DEFINE_LOCATION(true, ros::console::levels::Debug,
            ROSCONSOLE_ROOT_LOGGER_NAME ".youbot_arm_kinematics");

    if (enabled) {
        ros::console::print(0, loc.logger_, ros::console::levels::Debug,
                file, line, "", "%s", msg.c_str());
    }
}
