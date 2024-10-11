#include <gtest/gtest.h>
#include "claw_api.hpp"

TEST(roboclaw, InitialConnection)
{
    ASSERT_EQ(5, 2+2+1);

}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}