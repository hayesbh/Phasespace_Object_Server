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
        p.Init(i*53 % 13, i * 3.14, i / 1.123);
        p.id_ = i;
        points06.push_back(p);
      }
      for (int i = 7; i < 14; ++i) {
        Point p;
        p.Init(i*53 % 13, i * 3.14, i / 1.123);
        p.id_ = i;
        points713.push_back(p);
      }
      for (int i = 3; i < 10; ++i) {
        Point p;
        p.Init(i+1, i+2, i+3);
        p.id_ = i;
        saddle.push_back(p);
      }
      too_many = points06;
      Point p;
      p.Init(1, 2, 3);
      p.id_ = 7;
      too_many.push_back(p);
    }
};

// Initialize with points from 0-6
TEST_F(GloveInitTest, Init06) {
  GloveType glove;
  glove.Init(points06, false);
  EXPECT_EQ(glove.thumb_->id_, 1) << "Thumb Wrong";
  EXPECT_EQ(glove.fore_->id_, 3) << "Point Wrong";
  EXPECT_EQ(glove.middle_->id_, 4) << "Mid_dle wrong";
  EXPECT_EQ(glove.ring_->id_, 0) << "Ring wrong";
  EXPECT_EQ(glove.pinkey_->id_, 5) << "Pinkey Wrong";
  EXPECT_EQ(glove.base_left_->id_, 2) << "Base Left Wrong";
  EXPECT_EQ(glove.base_right_->id_, 6) << "Base Right Wrong";
}

TEST_F(GloveInitTest, Init713) {
  GloveType glove;
  glove.Init(points713, false);
  EXPECT_EQ(glove.thumb_->id_, 8) << "Thumb Wrong";
  EXPECT_EQ(glove.fore_->id_, 10) << "Point Wrong";
  EXPECT_EQ(glove.middle_->id_, 11) << "Mid_dle wrong";
  EXPECT_EQ(glove.ring_->id_, 7) << "Ring wrong";
  EXPECT_EQ(glove.pinkey_->id_, 12) << "Pinkey Wrong";
  EXPECT_EQ(glove.base_left_->id_, 9) << "Base Left Wrong";
  EXPECT_EQ(glove.base_right_->id_, 13) << "Base Right Wrong";
}

TEST_F(GloveInitTest, InitTooMany) {
  GloveType glove;
  EXPECT_FALSE(glove.Init(too_many, false));
}

TEST_F(GloveInitTest, InitSaddle) {
  GloveType glove;
  EXPECT_FALSE(glove.Init(saddle, false));
}


// Run the tests on the point class
int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

