#include <algorithm>
#include <vector>
#include "gtest/gtest.h"
#include "../PhaseSpace/Point.h"
#include "../PhaseSpace/GloveType.h"

using std::vector;
using object_server::Point;
using object_server::GloveType;

class GloveInitTest : public ::testing::Test {
  protected :
    vector<Point> points06;
    vector<Point> points713;
    vector<Point> too_many;
    vector<Point> saddle;
    virtual void SetUp() {
      for (int i = 0; i < 7; ++i) {
        Point p;
        p.init(i*53 % 13, i * 3.14, i / 1.123);
        p.id = i;
        points06.push_back(p);
      }
      for (int i = 7; i < 14; ++i) {
        Point p;
        p.init(i*53 % 13, i * 3.14, i / 1.123);
        p.id = i;
        points713.push_back(p);
      }
      for (int i = 3; i < 10; ++i) {
        Point p;
        p.init(i+1, i+2, i+3);
        p.id = i;
        saddle.push_back(p);
      }
      too_many = points06;
      Point p;
      p.init(1, 2, 3);
      p.id = 7;
      too_many.push_back(p);
    }
};

// Initialize with points from 0-6
TEST_F(GloveInitTest, Init06) {
  GloveType glove;
  glove.init(points06, false);
  EXPECT_EQ(glove.thumb_->id, 1) << "Thumb Wrong";
  EXPECT_EQ(glove.fore_->id, 3) << "Point Wrong";
  EXPECT_EQ(glove.middle_->id, 4) << "Middle wrong";
  EXPECT_EQ(glove.ring_->id, 0) << "Ring wrong";
  EXPECT_EQ(glove.pinkey_->id, 5) << "Pinkey Wrong";
  EXPECT_EQ(glove.base_left_->id, 2) << "Base Left Wrong";
  EXPECT_EQ(glove.base_right_->id, 6) << "Base Right Wrong";
}

TEST_F(GloveInitTest, Init713) {
  GloveType glove;
  glove.init(points713, false);
  EXPECT_EQ(glove.thumb_->id, 8) << "Thumb Wrong";
  EXPECT_EQ(glove.fore_->id, 10) << "Point Wrong";
  EXPECT_EQ(glove.middle_->id, 11) << "Middle wrong";
  EXPECT_EQ(glove.ring_->id, 7) << "Ring wrong";
  EXPECT_EQ(glove.pinkey_->id, 12) << "Pinkey Wrong";
  EXPECT_EQ(glove.base_left_->id, 9) << "Base Left Wrong";
  EXPECT_EQ(glove.base_right_->id, 13) << "Base Right Wrong";
}

TEST_F(GloveInitTest, InitTooMany) {
  GloveType glove;
  EXPECT_FALSE(glove.init(too_many, false));
}

TEST_F(GloveInitTest, InitSaddle) {
  GloveType glove;
  EXPECT_FALSE(glove.init(saddle, false));
}


// Run the tests on the point class
int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

