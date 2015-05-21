#include <youbot_arm_kinematics_moveit/configuration_comparator.h>
#include <gtest/gtest.h>


TEST(configuration_comparator, throw_when_vectors_have_different_size)
{
    std::vector<double> a;
    std::vector<double> b;
    std::vector<double> ref;

    ConfigurationComparator<double> comp(ref);
    EXPECT_NO_THROW(comp(a, b));

    a.resize(1);
    EXPECT_THROW(comp(a, b), std::exception);

    a.resize(0);
    b.resize(1);
    EXPECT_THROW(comp(a, b), std::exception);
}


TEST(configuration_comparator, compare_vectors_with_one_element)
{
    std::vector<double> a;
    std::vector<double> b;
    std::vector<double> ref;

    ref.push_back(0.0);
    ConfigurationComparator<double> comp(ref);

    a.push_back(1.0);
    b.push_back(2.0);
    EXPECT_TRUE(comp(a, b));

    a[0] = 2.0;
    EXPECT_FALSE(comp(a, b));

    b[0] = -2.0;
    EXPECT_FALSE(comp(a, b));
}


TEST(configuration_comparator, compare_vectors_with_two_elements)
{
    std::vector<double> a;
    std::vector<double> b;
    std::vector<double> ref;

    ref.push_back(0.0);
    ref.push_back(0.0);
    ConfigurationComparator<double> comp(ref);

    a.push_back(-2.0);
    a.push_back( 1.0);
    b.push_back( 2.0);
    b.push_back( 2.0);
    EXPECT_TRUE(comp(a, b));
}


TEST(configuration_comparator, compare_vectors_with_two_elements_and_reference)
{
    std::vector<double> a;
    std::vector<double> b;
    std::vector<double> ref;

    ref.push_back(1.0);
    ref.push_back(2.0);
    ConfigurationComparator<double> comp(ref);

    a.push_back(1.0);
    a.push_back(2.0);
    b.push_back(2.0);
    b.push_back(2.0);
    EXPECT_TRUE(comp(a, b));
}


int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
