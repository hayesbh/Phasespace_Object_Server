// File: DefaultType.cpp
// Author: Dylan Visher
// Date: 5/18/13
// About: Logic for Default Type

#include "./DefaultType.h"

namespace object_server {

// Math And Vectors
using std::vector;

using std::pow;
// Quaternions
using quaternions::Qmult;
using quaternions::Qnormalize;
using quaternions::Qinv;
using quaternions::QRotate;

bool DefaultType::init (vector<Point> p, bool rig) {
  points = p;
  Axis1.init(1, 0, 0);
  Axis2.init(0, 1, 0);
  angle.push_back(1);
  angle.push_back(0);
  angle.push_back(0);
  angle.push_back(0);
  center.init(0, 0, 0);
  dim.push_back(.1);
  dim.push_back(.1);
  dim.push_back(.1);
  // Get the Center
  GetCenter(1);
  // Get the Angle information
  GetAngle(1);
  // Get the Dimensional information
  GetScale(1);
  rigid = rig;
  return true;
}
Point DefaultType::get_pointer() {
  return center;
}
// GetFirstAxis finds (and sets if first time of i == 1) the local x_axis
// default is two with lowest y-values
bool DefaultType::GetFirstAxis(int i = 0){
  printf("DefaultType: Getting First Axis\n");
  if (AxisPoints1.size() == 2 && i == 0) {
    if(AxisPoints1[0]->current == 0 || AxisPoints1[1]->current == 0) {
      return false;
    }
    Axis1 = AxisPoints1[1]->sub(*AxisPoints1[0]).normalize();
    return false;
  } else {
    // If there are not enough points then just return the 0 vector
    if (points.size() < 2) {
      Axis1.init();
      return false;
    }
    AxisPoints1.clear();
    // compare with max distance squared
    float low1 = 10000000000000;
    float low2 = 1000000000000;
    // P1 and P2 are the Points that define the vector
    vector<Point>::iterator P1 = points.end();
    vector<Point>::iterator P2 = points.end();
    /*Iterators for going through the points that define the object*/
    vector<Point>::iterator iter;
    for (iter = points.begin(); iter != points.end(); ++iter) {
      if (!iter->current) continue;
      if (iter->y < low1) {
        low1 = iter->y;
        low2 = low1;
        P2 = P1;
        P1 = iter;
      } else if (iter->y < low2) {
        low2 = iter->y;
        P2 = iter;
      } 
    }
    if (P1 == points.end() || P2 == points.end()) {
      return false;  
    }
    // Remember the points that make up this axis
    // for later reference when finding the angle from the start
    vector<Point>::iterator temp;
    if (P2->x < P1->x) {
      temp = P2;
      P2 = P1;
      P1 = temp;
    }
    AxisPoints1.push_back(P1);
    AxisPoints1.push_back(P2);
    OriginalAxis1 = P2->sub(*P1).normalize();
    Axis1 = OriginalAxis1;
    return true;
  }
}
// GetSecondAxis gets the y_axis in the local object's frame of reference
// For a block (default type) this would be the one that
// has the closest x value to one of the first axis points
bool DefaultType::GetBothAxes(int i = 0){
  if(!GetFirstAxis(i)) return false;
  if (AxisPoints2.size() == 2 &&  i == 0) {
    Point u;
    // Make sure that both axes are up to date
    if(AxisPoints1[0]->current == 0 || AxisPoints1[1]->current == 0 || AxisPoints2[0]->current == 0 || AxisPoints2[2]->current == 0 ) {
      return false;
    } else {
      u = AxisPoints2[1]->sub(*AxisPoints2[0]);
    }
    Point v;
    if(AxisPoints1[1] == AxisPoints2[0]) {
      v = AxisPoints1[0]->sub(*AxisPoints1[1]);
    } else {
      v = AxisPoints1[1]->sub(*AxisPoints1[0]);
    }
    // find the normal component between the two vectors
    Axis2 = u.sub(v.normalize().times(u.dot(v))).normalize();
    return true;
  } else {
    // Make sure that there are enough points to define this second axis
    if (points.size() < 3) {
      printf("Not Enough Points\n");
      Axis2.init();
      return false;
    }
    AxisPoints2.clear();
    // Make sure the first axis exists
    if(AxisPoints1[0]->current == 0 || AxisPoints1[1]->current == 0) {
      printf("Axis Points1 not current\n");
      Axis2.init();
      return false;
    }
    //  distance found so far
    float x_dist = 0;
    // P1 and P2 are the Points that define the vector for the y_axis
    // minimize the distance in regards to the x_axis
    vector<Point>::iterator P1 = points.end();
    vector<Point>::iterator P2 = points.end();
    vector<Point>::iterator iter;
    for(int i = 0; i <= 1; ++i) {
      vector<Point>::iterator temp_base = AxisPoints1[i];
      for (iter = points.begin(); iter != points.end(); ++iter) {
        if(iter == AxisPoints1[0] || iter == AxisPoints1[1]) continue;
        if(!(iter->current)) continue;
        float temp_dist;
        if ((temp_dist = fabs(iter->sub(*temp_base).x)) < x_dist) {
          P1 = temp_base;
          P2 = iter;
          x_dist = temp_dist;
        }
      }
    }
    if (P1 == points.end() || P2 == points.end()) return false;
    // Make sure that the Points are remembered for future reference
    AxisPoints2.push_back(P1);
    AxisPoints2.push_back(P2);
    Axis2 = OriginalAxis2;
    return true;
  }
}
}  // namespace object_server
