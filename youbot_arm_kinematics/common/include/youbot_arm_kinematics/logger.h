#ifndef LOGGER_H
#define LOGGER_H

#include <string>

namespace youbot_arm_kinematics
{

class Logger
{
    public:
        /**
         * Ctor.
         */
        Logger();

        /**
         * Dtor.
         */
        virtual ~Logger();

        /**
         * Write a log message. The default implementation is a dummy which does
         * nothing. This function should be overridden in a derived class to
         * achieve different behavior.
         *
         * @param msg The message to write to the log.
         *
         * @param file The source file from where the message was logged.
         *
         * @param line The line in the file from where the message was logged.
         */
        virtual void write(const std::string &msg, const char *file, int line);


    public:
        /**
         * A null object which can be used as a default argument.
         */
        static Logger null;
};

}

#endif
