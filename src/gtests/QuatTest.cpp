#include <algorithm>
#include <vector>
#include "gtest/gtest.h"
#include "../PhaseSpace/Point.h"
#include "../PhaseSpace/quaternion.h"

using quaternions::QRotate;
using object_server::Point;
using std::vector;
class QuatTest : public ::testing::Test {
  protected:
    Point x_axis;
    Point y_axis;
    Point z_axis;
    vector<float> empty;
    vector<float> zero;
    vector<float> ident;
    vector<float> x180;
    vector<float> y180;
    vector<float> z180;
    vector<float> x90;
    vector<float> y90;
    vector<float> z90;
    virtual void SetUp() {
      x_axis.Init(1, 0, 0);
      y_axis.Init(0, 1, 0);
      z_axis.Init(0, 0, 1);

      zero.push_back(0);
      zero.push_back(0);
      zero.push_back(0);
      zero.push_back(0);

      ident.push_back(1);
      ident.push_back(0);
      ident.push_back(0);
      ident.push_back(0);
      
      x180.push_back(0);
      x180.push_back(1);
      x180.push_back(0);
      x180.push_back(0);
      
      y180.push_back(0);
      y180.push_back(0);
      y180.push_back(1);
      y180.push_back(0);
      
      z180.push_back(0);
      z180.push_back(0);
      z180.push_back(0);
      z180.push_back(1);
     
      x90.push_back(sqrt(0.5));
      x90.push_back(sqrt(0.5));
      x90.push_back(0);
      x90.push_back(0); 
    
      y90.push_back(sqrt(0.5));
      y90.push_back(0);
      y90.push_back(sqrt(0.5));
      y90.push_back(0);
     
      z90.push_back(sqrt(0.5));
      z90.push_back(0);
      z90.push_back(0);
      z90.push_back(sqrt(0.5));
    }
};
// Rotate Each of the Axes .y_ an emp.y_ vector (quaternion) (invalid)
TEST_F(QuatTest, InvalidQuatRotation) {
  Point p = QRotate(x_axis, empty);
  EXPECT_TRUE(p.Equals(x_axis));
  p = QRotate(y_axis, empty);
  EXPECT_TRUE(p.Equals(y_axis));
  p = QRotate(z_axis, empty);
  EXPECT_TRUE(p.Equals(z_axis));
}

// Rotate Each of the Axes .y_ a 0 quaternion (invalid) 
TEST_F(QuatTest, ZeroFailedRotation) {
  Point p = QRotate(x_axis, zero);
  EXPECT_TRUE(p.Equals(x_axis));
  p = QRotate(y_axis, zero);
  EXPECT_TRUE(p.Equals(y_axis));
  p = QRotate(z_axis, zero);
  EXPECT_TRUE(p.Equals(z_axis));
}

// Roate each of the axes .y_ the identi.y_ quaternion
TEST_F(QuatTest, IdentityRotations) {
  Point p = QRotate(x_axis, ident);
  EXPECT_TRUE(p.Equals(x_axis));
  p = QRotate(y_axis, ident);
  EXPECT_TRUE(p.Equals(y_axis));
  p = QRotate(z_axis, ident);
  EXPECT_TRUE(p.Equals(z_axis));
}

// Rotate each of the axes around the y-axis .y_ 180 degress
TEST_F(QuatTest, X180) {
  Point p = QRotate(x_axis, x180);
  EXPECT_TRUE(p.Equals(x_axis)) << p;

  p = QRotate(x_axis, y180);
  p.x_ *= -1;
  EXPECT_TRUE(p.Equals(x_axis)) << p;
  
  p = QRotate(x_axis, z180);
  p.x_ *= -1;
  EXPECT_TRUE(p.Equals(x_axis)) << p;
}

// Rotate each of the axes around the y-axis .y_ 180 degrees
TEST_F(QuatTest, Y180) {
  Point p = QRotate(y_axis, x180);
  p.y_ *= -1;
  EXPECT_TRUE(p.Equals(y_axis)) << p;

  p = QRotate(y_axis, y180);
  EXPECT_TRUE(p.Equals(y_axis)) << p;

  p = QRotate(y_axis, z180);
  p.y_ *= -1;
  EXPECT_TRUE(p.Equals(y_axis)) << p;
}

// Rotate each of the axes around the z-axis .y_ 180 degrees
TEST_F(QuatTest, Z180) {
  Point p = QRotate(z_axis, x180);
  p.z_ *= -1;
  EXPECT_TRUE(p.Equals(z_axis)) << p;
  
  p = QRotate(z_axis, y180);
  p.z_ *= -1;
  EXPECT_TRUE(p.Equals(z_axis)) << p;

  p = QRotate(z_axis, z180);
  EXPECT_TRUE(p.Equals(z_axis)) << p;
}

// Rotate each of the axes around the x-axis .y_ 90 degrees
TEST_F(QuatTest, X90) {
  Point p = QRotate(x_axis, x90);
  EXPECT_TRUE(p.Equals(x_axis)) << p;

  p = QRotate(x_axis, y90);
  EXPECT_TRUE(p.Equals(z_axis.Times(-1))) << p;

  p = QRotate(x_axis, z90);
  EXPECT_TRUE(p.Equals(y_axis)) << p;
}

// Rotate each of the axes around the y-axis .y_ 90 degrees
TEST_F(QuatTest, Y90) {
  Point p = QRotate(y_axis, x90);
  EXPECT_TRUE(p.Equals(z_axis)) << p;
  
  p = QRotate(y_axis, y90);
  EXPECT_TRUE(p.Equals(y_axis)) << p;

  p = QRotate(y_axis, z90);
  p.x_ *= -1;
  EXPECT_TRUE(p.Equals(x_axis)) << p;
}

// Rotate each of the axes around the z-axis .y_ 90 degrees
TEST_F(QuatTest, Z90) {
  Point p = QRotate(z_axis, x90);
  p.y_ *= -1;
  EXPECT_TRUE(p.Equals(y_axis)) << p;
  
  p = QRotate(z_axis, y90);
  EXPECT_TRUE(p.Equals(x_axis)) << p;

  p = QRotate(z_axis, z90);
  EXPECT_TRUE(p.Equals(z_axis)) << p;
}

// Test the Quaternion Class (Specifical.y_ Rotation
int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
