#include <algorithm>
#include <vector>
#include "gtest/gtest.h"
#include "../PhaseSpace/Point.h"

using object_server::Point;
using object_server::FindPointById;
using std::vector;
using std::next_permutation;

class PointTest : public ::testing::Test {
  protected:
    Point ZeroMag_;
    Point NearZeroMag_;
    vector<Point> points;
    virtual void SetUp() {
      ZeroMag_.Init(0, 0, 0);
      NearZeroMag_.Init(0.000000000000000000001,
                        0.000000000000000000001,
                        0.000000000000000000001);
      for (int i = 19; i >= 0; --i) {
        Point p;
        p.Init(i*173, i/79, i*1.735);
        p.id_ = i;
        points.push_back(p);
      }
    }
};


// Normalization Tests
// Vector normalization can handle a Zero Magnitude
TEST_F(PointTest, NormZeroMag) {
  Point p = ZeroMag_;
  p = p.Normalize();
  ASSERT_TRUE(p.Equals(ZeroMag_)) << "Normalize 0 vector !=  0 vector";
  ASSERT_EQ(p.Magnitude(), 0)  << "Normalize 0 vector: Mag != 0";
}

// Vector normalization can handle a Near-Zero Magnitude
TEST_F(PointTest, NormNearZeroMag) {
  Point p = NearZeroMag_; 
  p = p.Normalize();
  EXPECT_FALSE(p.Equals(ZeroMag_)) << "Normalize near 0 vector == 0 vector";
  ASSERT_EQ(p.Magnitude(), 1)  << "Normalize near 0 vector: Mag != 1";
}

// FindById works if item is in the list
TEST_F(PointTest, FindPointByIdSuccess) {
  for (int j = 0; j < 20; ++j) {
    for (int i = 0; i < 20; ++i) {
      ASSERT_EQ(FindPointById(i, points)->id_, i) 
                  << "FindPointById not returning right iterator";
    }
  std::random_shuffle(points.begin(), points.end());
  }
}

// FindById works if the item is not in the list
TEST_F(PointTest, FindPointByIdFail) {
  ASSERT_EQ(FindPointById(20, points),  points.end()) 
             << "FindPointById not failing gracefully";
}

// Run the tests on the point class
int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

