// File: DefaultType.h
// Author: Dylan Visher
// Date: 5/18/13
// About: DefaultType contains default information
#ifndef _SHL_COREOBJECTSERVER_DEFAULTTYPE_H
#define _SHL_COREOBJECTSERVER_DEFAULTTYPE_H

#include <vector>
#include <math.h>
#include "./Point.h"
#include "./quaternion.h"

namespace object_server {

// Math And Vectors
using std::vector;

using std::pow;
// Points
using object_server::Point;
// Quaternions
using quaternions::Qmult;
using quaternions::Qnormalize;
using quaternions::Qinv;
using quaternions::QRotate;

// This class is the Default Object for holding type specific information
class DefaultType : ObjectType {
  public:
  // init initializes the Object to hold in it the points given
  // p: The points that this object will track
  void init(vector<Point> p);
  // GetFirstAngleAxis finds (and sets if first time of i == 1) the local x_axis
  // default first angle axis is defined by the furthest two points
  Point GetFirstAngleAxis(int i = 0);
  Point GetSecondAngleAxis(int i = 0);
};

}  // namespace object_server
#endif
