#include <youbot_arm_kinematics_moveit/configuration_comparator.h>
#include <exception>


template <typename T>
ConfigurationComparator<T>::ConfigurationComparator(
        const std::vector<T> &reference) : reference_(reference)
{
}


template <typename T>
ConfigurationComparator<T>::~ConfigurationComparator()
{
}


template <typename T>
bool ConfigurationComparator<T>::operator()(const std::vector<T> &a,
        const std::vector<T> &b)
{
    if ((reference_.size() != a.size()) || (a.size() != b.size())) {
        throw std::exception();
    }
    
    double sqr_dist_a = 0.0;
    double sqr_dist_b = 0.0;
    
    for (std::size_t i = 0; i < reference_.size(); i++) {
        double dist_a = reference_[i] - a[i];
        double dist_b = reference_[i] - b[i];
        
        sqr_dist_a += dist_a * dist_a;
        sqr_dist_b += dist_b * dist_b;
    }
    
    return sqr_dist_a < sqr_dist_b;
}
