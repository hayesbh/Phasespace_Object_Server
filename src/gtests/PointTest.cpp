#include "gtest/gtest.h"
#include "../PhaseSpace/Point.h"

using object_server::Point;

class PointTest : public ::testing::Test {
  protected:
    Point ZeroMag_;
    Point NearZeroMag_;
    virtual void SetUp() {
      ZeroMag_.init(0,0,0);
      NearZeroMag_.init(0.00000000001, 0.00000000001, 0.00000000001);
    }
};
TEST_F(PointTest, NormZeroMag) {
  Point p = ZeroMag_;
  ASSERT_TRUE(p.equals(ZeroMag_));
}

TEST_F(PointTest, NormNearZeroMag) {
  Point p = NearZeroMag_; 
  EXPECT_FALSE(p.x == 0 && p.y == 0 && p.z == 0) << p.x << "," << p.y << "," << p.z;
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
