#include <gtest/gtest.h>

#include "control/coil_controller.h"

namespace Testing
{
class CoilControllerTest : public ::testing::Test
{
    using Base = ::testing::Test;

  public:
    CoilControllerTest() : Base() {};

    void SetUp() override
    {
        Base::SetUp();
    }

    void TearDown() override
    {
        Base::TearDown();
    }
};

} // namespace Testing