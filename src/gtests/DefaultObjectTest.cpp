#include <vector>
#include "gtest/gtest.h"
#include "../PhaseSpace/Point.h"
#include "../PhaseSpace/ObjectType.h"
#include "../PhaseSpace/DefaultType.h"

using object_server::Point;
using object_server::DefaultType;
using object_server::ObjectType;
using std::vector;

class InitTest : public ::testing::Test {
  protected:
  Point p1;
  Point p2;
  Point p3;
  Point p4;
  Point p5;
  Point p6;
  Point p7;
  Point p8;
  vector<Point> points;
  virtual void SetUp() {
    p1.Init(0, 0, 0);
    p1.id_ = 1;
    p2.Init(1, 0, 0);
    p2.id_ = 2;
    p3.Init(0, 1, 0);
    p3.id_ = 3;
    p4.Init(0, 0, 1);
    p4.id_ = 4;
    p5.Init(1, 1, 0);
    p5.id_ = 5;
    p6.Init(0, 1, 1);
    p6.id_ = 6;
    p7.Init(1, 0, 1);
    p7.id_ = 7;
    p8.Init(1, 1, 1);
    points.push_back(p1);
    points.push_back(p2);
    points.push_back(p3);
    points.push_back(p4);
    points.push_back(p5);
    points.push_back(p6);
    points.push_back(p7);
    points.push_back(p8);
  }
};

TEST_F(InitTest, NormalCoordinateSystem) {
  DefaultType obj;
  obj.Init(points, true);
  EXPECT_TRUE(obj.get_rigidity());
  EXPECT_TRUE(obj.get_angle()[0] == 1 && obj.get_angle()[1] == 0 && obj.get_angle()[2] == 0 && obj.get_angle()[3] == 0) 
    << obj.get_angle()[0] << ", " << obj.get_angle()[1] << ", " << obj.get_angle()[2] << ", " << obj.get_angle()[3];
  EXPECT_TRUE(obj.get_axis1_ids().size() == 2);
  EXPECT_TRUE(obj.get_original_axis1().Equals(p2));
  EXPECT_TRUE(obj.get_original_axis2().Equals(p3));
  EXPECT_TRUE(obj.get_angle()[0] == 1 && obj.get_angle()[1] == 0 && obj.get_angle()[2] == 0 && obj.get_angle()[3] == 0);
  EXPECT_TRUE(fabs(obj.get_dimensions()[0]-1.1) < .001 && fabs(obj.get_dimensions()[1]-1.1) < .001 && fabs(obj.get_dimensions()[2]-1.1) < .001)
    << obj.get_dimensions()[0] << ", " << obj.get_dimensions()[1] << ", " << obj.get_dimensions()[2];
}


int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
