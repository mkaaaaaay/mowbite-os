#include <gtest/gtest.h>

#include "mowbite_msgs/blade.hpp"

namespace {

mowbite_msgs::msg::HardwareStatus Status(bool blade_on, bool ok, int16_t rpm) {
  mowbite_msgs::msg::HardwareStatus status;
  status.blade_on = blade_on;
  status.blade_motor.ok = ok;
  status.blade_motor.rpm = rpm;
  return status;
}

TEST(Blade, StillOnlyWhenOffAndNotTurning) {
  EXPECT_FALSE(mowbite_msgs::BladeMayTurn(Status(false, true, 0)));
  EXPECT_FALSE(mowbite_msgs::BladeMayTurn(Status(false, false, 0)));
  EXPECT_TRUE(mowbite_msgs::BladeMayTurn(Status(true, true, 0)));
  EXPECT_TRUE(mowbite_msgs::BladeMayTurn(Status(true, false, 0)));
  // Running down after off, in either direction, also below blade_running_rpm.
  EXPECT_TRUE(mowbite_msgs::BladeMayTurn(Status(false, true, 40)));
  EXPECT_TRUE(mowbite_msgs::BladeMayTurn(Status(false, true, -3)));
  // An rpm without a report says nothing.
  EXPECT_FALSE(mowbite_msgs::BladeMayTurn(Status(false, false, 40)));
}

}  // namespace
