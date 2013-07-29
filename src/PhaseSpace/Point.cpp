// File: Point.cpp
// Author: Dylan Visher
// Date: 5/18/13
// About: Logic for Points
#include <string>
#include <vector>
#include "./Point.h"

namespace object_server {

// Initialize the point with the given x, y and z coordinates
// x, y, z: floats describing location
void Point::init(float X, float Y, float Z) {
  id = -1;
  current = 1;
  x = X;
  y = Y;
  z = Z;
  return;
}


// init without arguments initializes the point with at (0,0,0)
void  Point::init() {
  id = -1;
  current = 1;
  x = y = z = 0;
  return;
}
// Update updates the point with marker information
// mark: OWLMarker passed by the Server
// return: this new point
void Point::Update(OWLMarker mark) {
  current = 1;
  id = mark.id;
  x = mark.x;
  y = mark.y;
  z = mark.z;
}

// equals checks whether this point occupies the same space as another
// p: a Point to compare against this one
// return bool indicating "Is that point at the same place as this one?"
bool Point::equals(Point p) {
  float epsilon = .00001;
  return (fabs(p.x - x) < epsilon) &&
         (fabs(p.y - y) < epsilon) &&
         (fabs(p.z - z) < epsilon);
}


// sub subtracts two points
// p: the Point to subtract from this one  
// return the new point
Point Point::sub(Point p) {
  Point P;
  (p.current && current) ? (P.current = 1) : (P.current = 0);
  P.x = x - p.x;
  P.y = y - p.y;
  P.z = z - p.z;
  return P;
}
// times scalar multiplication of the point-vector
// a: float scalar multiple
// return: the new point
Point Point::times(float a) {
  Point P;
  P.current = current;
  P.x = x*a;
  P.y = y*a;
  P.z = z*a;
  return P;
}
// add adds two vectors together
// p: Point to add
// return the new point
Point Point::add(Point p) {
  Point P;
  (p.current && current) ? (P.current = 1) : (P.current = 0);
  P.x = x + p.x;
  P.y = y + p.y;
  P.z = z + p.z;
  return P;
}

// cross performs the cross product of this point on the given
// p: the point to be crossed agains -> this X p
// return the vector representing the cross product
Point Point::cross(Point p) {
  Point c;
  (p.current && current) ? (c.current = 1) : (c.current = 0);
  c.x = y * p.z - z * p.y;
  c.y = z * p.x - x * p.z;
  c.z = x * p.y - y * p.x;
  return c;
}

// dot performs the dot product of this Point with the given one
// p: the other point to be dotted with
// return the scalar floar representing the dot product
float Point::dot(Point p) {
  return p.x*x + p.y*y + p.z*z; 
}

// return the float magnitude
float Point::magnitude() {
  return sqrt(pow(x, 2)+pow(y, 2)+pow(z, 2));
}

// normalize finds the unit vector
// return the unit vector
Point Point::normalize() {
  Point p;
  float mag = magnitude();
  if (mag == 0) {
    p.init();
    return p;
  }
  p.x = x / mag;
  p.y = y / mag;
  p.z = z / mag;
  return p;
}

// DistanceToPoint finds the distance from this point to another
// p: the other point to compare with
// return the distance between this and p
float Point::DistanceToPoint(Point p) {
  return sub(p).magnitude();
}
// VectorPerpendicularTo finds the vector perpendicular to a line
// line: a vector of two points that define a line in R3
// return: the shortest distance from this point to that line
Point Point::VectorPerpendicularTo(vector<Point> line) {
  Point v = line[1].sub(line[0]);
  Point u = this->sub(line[0]);
  return u.sub(v.normalize().times(v.dot(u)));
}
// print returns the string representation of a point
// return this string "(x, y, z)"
string Point::print() {
  stringstream s;
  s << "(" << x << "," << y << "," << z << ")";
  return s.str();
}
// FindById Finds a Point by its ID from a vector of points
// id: the id of the point desired
// points: a vector of points to look in
// return an iterator to that point
const vector<Point>::iterator FindPointById(int id, vector<Point> &points) {
  vector<Point>::iterator iter;
  for (iter = points.begin(); iter != points.end(); ++iter) {
    if (iter->id == id)
      return iter;
  }
  return points.end();
}
::std::ostream& operator<<(::std::ostream& os, const Point& p) {
  return os << "[" << p.id << "," << p.current << "]:("
            << p.x << "," << p.y << "," << p.z << ")";
}

}  //  namespace object_server
