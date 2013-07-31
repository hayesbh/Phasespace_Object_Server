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
  Point origin_;
  Point p1_;
  Point p2_;
  Point p3_;
  Point p4_;
  Point p5_;
  Point p6_;
  Point p7_;
  vector<Point> all_points_;
  virtual void SetUp() {
    origin_.Init(0,0,0);
    origin_.id_ = 0;
    p1_.Init(1, 0, 0);
    p1_.id_ = 1;
    p2_.Init(0, 1, 0);
    p2_.id_ = 2;
    p3_.Init(0, 0, 1);
    p3_.id_ = 3;
    p4_.Init(1, 1, 1);
    p4_.id_ = 4;
    p5_.Init(0, 1, 1);
    p5_.id_ = 5;
    p6_.Init(1, 0, 1);
    p6_.id_ = 6;
    p7_.Init(1, 1, 0);
    p7_.id_ = 7;
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
  obj.Init(0, "single_point", points, "default", false);
  EXPECT_TRUE(obj.get_points()[0].Equals(origin_));
  EXPECT_TRUE(obj.get_center().Equals(origin_));
  EXPECT_TRUE(fabs(obj.get_dimensions()[0]-.022) <= .0001 && fabs(obj.get_dimensions()[1]-.022) <= .0001 && fabs(obj.get_dimensions()[0]-.022) <= .0001) 
    << obj.get_dimensions()[0] << ", " << obj.get_dimensions()[1] << ", " << obj.get_dimensions()[2];
  EXPECT_TRUE(obj.get_original_axis1().x_ == 1 && obj.get_original_axis1().y_ == 0 && obj.get_original_axis1().z_ == 0)
    << obj.get_original_axis1();
  EXPECT_TRUE(obj.get_original_axis2().x_ == 0 && obj.get_original_axis2().y_ == 1 && obj.get_original_axis2().z_ == 0)
    << obj.get_original_axis2();
  EXPECT_TRUE(obj.get_type() == "default");
  EXPECT_TRUE(obj.get_axis1_ids().size() == 0);
  EXPECT_TRUE(obj.get_axis2_ids().size() == 0);
}

TEST_F(InitTest, SeveralPoints) {
  PSObject obj;
  obj.Init(0, "all_points", all_points_, "default", false);
  for(int i = 0; i < 2; ++i) {
  vector<Point> points = obj.get_points();
  vector<Point>::iterator iter;
  for (iter = points.begin(); iter != points.end(); ++iter) {
    EXPECT_TRUE(iter->current_ == 1);
  }
  EXPECT_TRUE(obj.get_center().x_ == .5 && obj.get_center().y_ == .5 && obj.get_center().z_ == .5) << obj.get_center();
  EXPECT_TRUE(obj.get_rotation()[0] == 1 && obj.get_rotation()[1] == 0 && obj.get_rotation()[2] == 0 && obj.get_rotation()[3] == 0);
  EXPECT_TRUE(fabs(obj.get_dimensions()[0]-1.1) <= .0001 && fabs(obj.get_dimensions()[1]-1.1) <= .0001 && fabs(obj.get_dimensions()[2]-1.1) <= .0001)
    << obj.get_dimensions()[0] << ", " << obj.get_dimensions()[1] << ", " << obj.get_dimensions()[2];
  EXPECT_TRUE(obj.get_original_axis1().x_ == 1 && obj.get_original_axis1().y_ == 0 && obj.get_original_axis1().z_ == 0)
    << obj.get_original_axis1();
  EXPECT_TRUE(obj.get_original_axis2().x_ == 0 && obj.get_original_axis2().y_ == 1 && obj.get_original_axis2().z_ == 0)
    << obj.get_original_axis2();
  EXPECT_TRUE(obj.get_type() == "default");
  EXPECT_TRUE(obj.get_axis1_ids().size() == 2) << obj.get_axis1_ids().size();
  EXPECT_TRUE(obj.get_axis1_ids()[0] == 0 && obj.get_axis1_ids()[1] == 1);
  EXPECT_TRUE(obj.get_axis2_ids().size() == 2);
  EXPECT_TRUE(obj.get_axis2_ids()[0] == 0 && obj.get_axis2_ids()[1] == 2);
  obj.reset();
  }
}


int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
