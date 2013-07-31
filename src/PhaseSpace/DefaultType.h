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
using quaternions::QMult;
using quaternions::QNormalize;
using quaternions::QInv;
using quaternions::QRotate;

// This class is the Default Object for holding type specific information
class DefaultType : public ObjectType {
  public:
  // init initializes the Object to hold in it the points given
  // p: The points that this object will track
  // rig: bool value indicating whether the object is rigid (true if it is)
  // return: bool indicated whether this object was successfully initialized
  bool Init(vector<Point> p, bool rig);
  // GetFirstAngleAxis finds (and sets if first time of i == 1) the local x_axis
  // default first angle axis is defined by the two points with the y values closest to zero
  // the vector points in the +x direction
  // i: default set to 0 but if it is ever set to 1 the first axis will be reset according to the rules above
  // return: bool indicating whether this was successful at getting the first axis
  bool GetFirstAxis(int i);
  // GetBothAxes ginds both Axes (calls on GetFirstAxis) and sets both (if the first time or i == 1)
  // this is the local y axis and is defined by the point that has the closest x value to one of the base points
  // these axes that are initialized define the angle of the object
  // i: default set to 0 but if it is ever set to 1 the second axis and first axis will be reset according to the rules above
  // return: bool that indicates whether both axes were set
  bool GetBothAxes(int i);
  // get_pointer returns the pointer of the Object (specifically in the glove's case)
  Point get_pointer();
};

}  // namespace object_server
#endif
