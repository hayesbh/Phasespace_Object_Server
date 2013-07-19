// File: Point.h
// Author: Dylan Visher
// Date: 5/18/13
// About: Point Class for describing 3D points and vectors

#ifndef _SHL_COREOBJECTSERVER_POINT_H
#define _SHL_COREOBJECTSERVER_POINT_H

#include <vector>
#include <sstream>
#include <string>
#include <cmath>
#include "owl/owl.h"

namespace object_server {

using std::string;
using std::stringstream;
using std::vector;
// The Point Class handles Points and Vectors in the R3 Space
class Point {
 public:
  // idenfication number of the points
  int id;
  // 1 indicates that this was updated in the most recent update
  // 0 indicates that this point was not updated recently
  int current;
  // x y and z coordinates for the point
  float x;
  float y;
  float z;
  // init with x,y,z initializes the point at (x, y, z)
  // x, y, z: floats describing location
  void init(float X, float Y, float Z);
  // init without arguments initializes the point with at (0,0,0)
  void  init();
  // Update updates the point with marker information
  // mark: OWLMarker passed by the Server
  // return this new point
  void Update(OWLMarker mark);
  // equals checks whether this point occupies the same space as another
  // p: a Point to compare against this one
  // return bool indicating "Is that point at the same place as this one?"
  bool equals(Point p);
  // sub subtracts two points
  // p: the Point to subtract from this one  
  // return the new point
  Point sub(Point p);
  // times scalar multiplication of the point-vector
  // a: float scalar multiple
  // return: the new point
  Point times(float a);
  // add adds two vectors together
  // p: Point to add
  // return the new point
  Point add(Point p);
  // cross performs the cross product of this point on the given
  // p: the point to be crossed agains -> this X p
  // return the vector representing the cross product
  Point cross(Point p);
  // dot performs the dot product of this Point with the given one
  // p: the other point to be dotted with
  // return the scalar floar representing the dot product
  float dot(Point p);
  // The magnitude of this vector
  // return the float magnitude
  float magnitude();
  // normalize finds the unit vector
  // return the unit vector
  Point normalize();
  // DistanceToPoint finds the distance from this point to another
  // p: the other point to compare with
  // return the distance between this and p
  float DistanceToPoint(Point p);
  // VectorPerpendicularTo finds the vector perpendicular to a line
  // line: a vector of two points that define a line in R3
  // return: the shortest distance from this point to that line
  Point VectorPerpendicularTo(vector<Point> line);
  // DistanceToPlane finds the distance between a Point and a plane
  // plane: an array holding 4 elements representing
  //   A, B, C, D in the standard plane equation
  // One can convert three points to a plane by using the 
  //   PointsToPlane Function
  float DistanceToPlane(float plane[4]);
  // print returns the string representation of a point
  // return this string "(x, y, z)"
  string print();
};

const vector<Point>::iterator FindPointById (int id, vector<Point> &points);

}  // namespace object_server

#endif
