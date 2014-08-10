#ifndef CONFIGURATION_COMPARATOR_H
#define CONFIGURATION_COMPARATOR_H

#include <vector>

/**
 * A comparator for joint configurations (represented as vectors of double). The
 * configurations are compared w.r.t. a reference configuration.
 *
 * A use case of this class is sorting a collection of joint configurations so
 * that the configuration closest to the current configuration can be found.
 */
template <typename T>
class ConfigurationComparator
{
    public:
        /**
         * Ctor.
         *
         * @param reference The reference configuration.
         */
        ConfigurationComparator(const std::vector<T> &reference);

        /**
         * Dtor.
         */
        virtual ~ConfigurationComparator();

        /**
         * Compare the two input configurations based on their distance to the
         * reference configuration.
         *
         * @param a The first configuration.
         *
         * @param b The second configuration.
         *
         * @return The function returns true when the first configuration is
         * closer to the reference configuration, else it return false.
         */
        bool operator()(const std::vector<T> &a, const std::vector<T> &b);


    private:
        /**
         * The reference configuration as provided to the ctor.
         */
        std::vector<T> reference_;
};


#include "configuration_comparator.inl"


#endif
