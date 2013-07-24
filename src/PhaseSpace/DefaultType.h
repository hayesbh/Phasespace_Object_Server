// File: DefaultType.h
// Author: Dylan Visher
// Date: 5/18/13
// About: DefaultType contains default information
#ifndef _SHL_COREOBJECTSERVER_DEFAULTTYPE_H
#define _SHL_COREOBJECTSERVER_DEFAULTTYPE_H

#include <vector>
#include <cmath>
#include "Point.h"
#include "quaternion.h"
#include "ObjectType.h"

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
class DefaultType : public ObjectType {
  public:
  // init initializes the Object to hold in it the points given
  // p: The points that this object will track
  bool init(vector<Point> p, bool rig);
  // GetFirstAngleAxis finds (and sets if first time of i == 1) the local x_axis
  // default first angle axis is defined by the furthest two points
  bool GetFirstAxis(int i);
  bool GetBothAxes(int i);
  Point get_pointer();
};

}  // namespace object_server
#endif
