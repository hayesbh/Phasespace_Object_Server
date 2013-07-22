// File : Glove.cpp
// Author: Dylan Visher
// Date: 5/18/13
// About: Logic for the Glove Object Type

#include "./GloveType.h"

namespace object_server {

// init sets initializes the points of the glove, finds all of the finges' specific Points
// Finds the center, the angle and the scale
void GloveType::init(vector<Point> p, bool rig) {
  printf("Setting up Glove\n");
  vector<Point>::iterator iter;
  /* Push back each of the points and blank ones where they are needed */
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

  /* Set up finger specific pointers */
  for (iter = points.begin(); iter != points.end(); ++iter) {
    if ( iter->id % 7 == 1 ) thumb = iter;
    else if ( iter->id % 7 == 2 ) base_left = iter;
    else if ( iter->id % 7 == 3 ) fore = iter;
    else if ( iter->id % 7 == 4 ) middle = iter;
    else if ( iter->id % 7 == 0 ) ring = iter;
    else if ( iter->id % 7 == 5 ) pinkey = iter;
    else if ( iter->id % 7 == 6 ) base_right = iter;
  }
  GetCenter(1);
  GetAngle(1);
  GetScale(1);
  rigid = rig;
}

Point GloveType::get_pointer() {
  return *fore;
}
// GetFirstAxisAngle for a glove is defined by the two leds
// That are located on the base of the hand
bool GloveType::GetFirstAxis(int i) {
  if (AxisPoints1.size() == 2) {
    if(base_right->current == 0 || base_left->current == 0) {
      return false;
    }
    Axis1 = base_right->sub(*base_left).normalize();
    return true;
  } else {
    if(base_right->current == 0 || base_left->current == 0) return false;
    AxisPoints1.push_back(base_left);
    AxisPoints1.push_back(base_right);
    Point x_axis;
    x_axis.init(1, 0, 0);
    OriginalAxis1 = x_axis;
    Axis1 = AxisPoints1[1]->sub(*AxisPoints1[0]).normalize();
    return true;
  }
}
// GetSecondAngleAxis looks for the axis defined by the pointer finger
// And the left base
bool GloveType::GetBothAxes(int i) {
  if(!GetFirstAxis(i)) return false;
  if (AxisPoints2.size() == 2) {
  // Check the first Axis' validity
  if(base_left->current == 0 || base_right->current == 0) {
    return false;
  }
  /*check second axis*/
  Point u;
  if(fore->current == 0) {
    return false;
  } else {
    u = fore->sub(*base_left);
  }
  Point v = fore->sub(*base_left);
  Axis2 = u.sub(v.normalize().times(u.dot(v))).normalize();
  // Now find the normal component
  return true;
  }
  else {
    if(base_left->current == 0 || fore->current == 0) return false;
    AxisPoints2.push_back(fore);
    AxisPoints2.push_back(base_left);
    Point v = fore->sub(*base_left);
    Axis2 =  v.sub(Axis1.times(v.dot(Axis1))).normalize();
    Point y_axis;
    y_axis.init(0, 1, 0);
    OriginalAxis2 = y_axis;
    return true;
  }
}
} // namespace object_server
