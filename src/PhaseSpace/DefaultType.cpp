// File: DefaultType.cpp
// Author: Dylan Visher
// Date: 5/18/13
// About: Logic for Default Type

// Math And Vectors
using std::vector;

using std::pow;
// Points
using object_server::Point;
using points::PointsToPlane;
using points::FindById;
// Quaternions
using quaternions::Qmult;
using quaternions::Qnormalize;
using quaternions::Qinv;
using quaternions::QRotate;

void DefaultType::init (vector<Point> p) {
  points = p;
  // Get the Center
  GetCenter();
  // Get the Angle information
  GetAngle();
  // Get the Dimensional information
  GetScale();
}
// GetFirstAxis finds (and sets if first time of i == 1) the local x_axis
// default first angle axis is defined by the furthest two points
Point GetFirstAxis(int i = 0){
  if (AxisPoints1.size() == 2 && i == 0) {
    if(AxisPoints1[0]->current == 0 || AxisPoints1[1]->current == 0) {
      return Axis1;
    }
    Axis1 = AxisPoints1[1]->sub(*AxisPoints1[0]).normalize();
    return Axis1;
  } else {
    // If there are not enough points then just return the 0 vector
    if (points.size() < 2) {
      Axis1.init();
      return Axis1;
    }
    AxisPoints1.clear();
    /* Max distance Squared to compare with */
    float maxdist2 = 0;
    /* P1 and P2 are the Points that define the vector */
    vector<Point>::iterator P1;
    vector<Point>::iterator P2;
    /*Iterators for going throught the points that define the object*/
    vector<Point>::iterator it1;
    vector<Point>::iterator it2;
    for (it1 = points.begin(); it1 != points.end(); ++it1) {
      if(!(it1->current)) continue;
      for (it2 = points.begin()+1; it2 != points.end(); ++it2) {
        if(!(it2->current)) continue;
          float dist2 = pow(it1->x - it2->x, 2)
                    + pow(it1->y - it2->y, 2)
                    + pow(it1->z - it2->z, 2);
        if (dist2 > maxdist2) {
          maxdist2 = dist2;
          P1 = it1;
          P2 = it2;
        }
      }
    }
    // Set the Points that define this axis so the angle can have a reference point
    AxisPoints1.push_back(P1);
    AxisPoints1.push_back(P2);
    OriginalAxis1 = P2->sub(*P1).normalize();
    Axis1 = OriginalAxis1;
    return Axis1;
  }
}
// GetSecondAxis gets the y_axis in the local object's frame of reference
Point GetSecondAxis(int i = 0){
  if(AxisPoints1.size() != 2) {
    Point p;
    p.init();
    return p;
  }
  if (AxisPoints2.size() == 2 && i == 0) {
    /*check first axis*/
    if(AxisPoints1[0]->current == 0 || AxisPoints1[1]->current == 0) {
      return Axis2;
    }
    Point u;
    /*check second axis*/
    if(AxisPoints2[0]->current == 0 || AxisPoints2[1]->current == 0) {
      return Axis2;
    } else {
      u = AxisPoints2[1]->sub(*AxisPoints2[0]);
    }
    GetAngle();
    Point v;
    if(AxisPoints1[1] == AxisPoints2[0]) {
      v = AxisPoints1[0]->sub(*AxisPoints1[1]);
    } else {
      v = AxisPoints1[1]->sub(*AxisPoints1[0]);
    }
    Axis2 = u.sub(v.normalize().times(u.dot(v))).normalize();
    // Now find the normal component
    return Axis2;
  } else {
    // Make sure that there are enough points to define this second axis
    if (points.size() < 3) {
      Axis2.init();
      return Axis2;
    }
    AxisPoints2.clear();
    // Make sure the first axis exists
    if(AxisPoints1[0]->current == 0 || AxisPoints1[1]->current == 0) {
      Axis2.init();
      return Axis2;
    }
    // max distance found so far
    float maxdist = 0;
    // P1 and P2 are the Points that define the vector for the y_axis
    vector<Point>::iterator P1;
    vector<Point>::iterator P2;
    vector<Point>::iterator iter;
    vector<Point>::iterator axis[2] = { AxisPoints1[0], AxisPoints1[1] };
    for(int i = 0; i <= 1; ++i) {
      vector<Point> line;
      line.push_back(*axis[i]);
      for (iter = points.begin(); iter != points.end(); ++iter) {
        if(iter == axis[0] || iter == axis[1]) continue;
        if(!(iter->current)) continue;
        line.push_back(*iter);
        float dist = iter->VectorPerpendicularTo(line).magnitude();
        if (dist > maxdist) {
          maxdist = dist;
          P1 = axis[i];
          P2 = iter;
          OriginalAxis2 = P2->sub(*P1).normalize();
        } line.erase(line.end()-1, line.end());
      } line.clear();
    }
    // Make sure that the Points are remembered for future reference
    AxisPoints2.push_back(P1);
    AxisPoints2.push_back(P2);
    Axis2 = OriginalAxis2;
    return Axis2;
  }
}
