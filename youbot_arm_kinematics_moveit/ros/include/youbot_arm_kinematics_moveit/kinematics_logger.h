#ifndef KINEMATICS_LOGGER_H
#define KINEMATICS_LOGGER_H

#include <youbot_arm_kinematics/logger.h>

class KinematicsLogger : public youbot_arm_kinematics::Logger
{
    public:
        /**
         * Dtor.
         */
        virtual ~KinematicsLogger();

        /**
         * @see youbot_arm_kinematics::Logger::write
         */
        void write(const std::string &msg, const char *file, int line);
};

#endif
