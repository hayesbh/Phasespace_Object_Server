#include <algorithm>
#include <vector>
#include "gtest/gtest.h"
#include "../PhaseSpace/Point.h"
#include "../PhaseSpace/ManualObject.h"
#include "../PhaseSpace/PSObject.h"
#include "../PhaseSpace/Object.h"
#include "../PhaseSpace/GloveType.h"
#include "../PhaseSpace/DefaultType.h"

using object_server::Point;
using object_server::PSObject;
using std::vector;



class InitTest : public ::testing::Test {
  protected :
  Point origin__;
  Point p1_;
  Point p2_;
  Point p3_;
  Point p4_;
  Point p5_;
  Point p6_;
  Point p7_;
  vector<Point> all_points_;
  virtual void SetUp() {
    origin__.init(0,0,0);
    p1_.init(1, 0, 0);
    p2_.init(0, 1, 0);
    p3_.init(0, 0, 1);
    p4_.init(1, 1, 1);
    p5_.init(0, 1, 1);
    p6_.init(1, 0, 1);
    p7_.init(1, 1, 0);
    all_points_.push_back(origin_);
    all_points_.push_back(p1_);
    all_points_.push_back(p2_);
    all_points_.push_back(p3_);
    all_points_.push_back(p4_);
    all_points_.push_back(p5_);
    all_points_.push_back(p6_);
    all_points_.push_back(p7_);
  }
};

TEST_F(InitTest, OnePoint) {
  vector<Point> points;
  points.push_back(origin_);
  PSObject obj;
  obj.init(0, "single_point", points, "default", false);
  EXPECT_TRUE(obj.get_points()[0].equals(origin_));
  EXPECT_TRUE(obj.get_center().equals(origin_));
  EXPECT_TRUE(fabs(obj.get_dimensions()[0]-.01) <= .0001 && fabs(obj.get_dimensions()[1]-.01) <= .0001 && fabs(obj.get_dimensions()[0]-.01) <= .0001) 
    << obj.get_dimensions()[0] << ", " << obj.get_dimensions()[1] << ", " << obj.get_dimensions()[2];
  EXPECT_TRUE(obj.get_OriginalAxis1().x == 1 && obj.get_OriginalAxis1().y == 0 && obj.get_OriginalAxis1().z == 0)
    << obj.get_OriginalAxis1();
  EXPECT_TRUE(obj.get_OriginalAxis2().x == 0 && obj.get_OriginalAxis2().y == 1 && obj.get_OriginalAxis2().z == 0)
    << obj.get_OriginalAxis2();
  EXPECT_TRUE(obj.get_type() == "default");
  EXPECT_TRUE(obj.get_axis1_ids().size() == 0);
  EXPECT_TRUE(obj.get_axis2_ids().size() == 0);
}

TEST_F(InitTest, SeveralPoints) {
  PSObject obj;
  obj.init(0, "all_points", all_points_, "default", false);
  EXPECT_TRUE(obj.get_center().equals(origin_)) << obj.get_center();
  EXPECT_TRUE(fabs(obj.get_dimensions()[0]-1) <= .0001 && fabs(obj.get_dimensions()[1]-1) <= .0001 && fabs(obj.get_dimensions()[0]-1) <= .0001)
    << obj.get_dimensions()[0] << ", " << obj.get_dimensions()[1] << ", " << obj.get_dimensions()[2];
  EXPECT_TRUE(obj.get_OriginalAxis1().x == 1 && obj.get_OriginalAxis1().y == 0 && obj.get_OriginalAxis1().z == 0)
    << obj.get_OriginalAxis1();
  EXPECT_TRUE(obj.get_OriginalAxis2().x == 0 && obj.get_OriginalAxis2().y == 1 && obj.get_OriginalAxis2().z == 0)
    << obj.get_OriginalAxis2();
  EXPECT_TRUE(obj.get_type() == "default");
  EXPECT_TRUE(obj.get_axis1_ids().size() == 2);
  EXPECT_TRUE(obj.get_axis1_ids()[0].equals(origin_)
  EXPECT_TRUE(obj.get_axis2_ids().size() == 2);
}


int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
