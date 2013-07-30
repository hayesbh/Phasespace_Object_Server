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
void Point::Init(float X, float Y, float Z) {
  id_ = -1;
  current_ = 1;
  x_ = X;
  y_ = Y;
  z_ = Z;
  return;
}


// Init without arguments initializes the point with at (0,0,0)
void Point::Init() {
  id_ = -1;
  current_ = 1;
  x_ = y_ = z_ = 0;
  return;
}
// Update Updates the point with marker information
// mark: OWLMarker passed by the Server
// return: this new point
void Point::Update(OWLMarker mark) {
  current_ = 1;
  id_ = mark.id;
  x_ = mark.x;
  y_ = mark.y;
  z_ = mark.z;
}

// Equals checks whether this point occupies the same space as another
// p: a Point to compare against this one
// return bool indicating "Is that point at the same place as this one?"
bool Point::Equals(Point p) {
  float epsilon = .00001;
  return (fabs(p.x_ - x_) < epsilon) &&
         (fabs(p.y_ - y_) < epsilon) &&
         (fabs(p.z_ - z_) < epsilon);
}


// Sub subtracts two points
// p: the Point to Subtract from this one  
// return the new point
Point Point::Sub(Point p) {
  Point P;
  (p.current_ && current_) ? (P.current_ = 1) : (P.current_ = 0);
  P.x_ = x_ - p.x_;
  P.y_ = y_ - p.y_;
  P.z_ = z_ - p.z_;
  return P;
}
// Times scalar multiplication of the point-vector
// a: float scalar multiple
// return: the new point
Point Point::Times(float a) {
  Point P;
  P.current_ = current_;
  P.x_ = x_*a;
  P.y_ = y_*a;
  P.z_ = z_*a;
  return P;
}
// Add adds two vectors together
// p: Point to Add
// return the new point
Point Point::Add(Point p) {
  Point P;
  (p.current_ && current_) ? (P.current_ = 1) : (P.current_ = 0);
  P.x_ = x_ + p.x_;
  P.y_ = y_ + p.y_;
  P.z_ = z_ + p.z_;
  return P;
}

// Cross performs the cross product of this point on the given
// p: the point to be Crossed agains -> this X p
// return the vector representing the Cross product
Point Point::Cross(Point p) {
  Point c;
  (p.current_ && current_) ? (c.current_ = 1) : (c.current_ = 0);
  c.x_ = y_ * p.z_ - z_ * p.y_;
  c.y_ = z_ * p.x_ - x_ * p.z_;
  c.z_ = x_ * p.y_ - y_ * p.x_;
  return c;
}

// Dot performs the dot product of this Point with the given one
// p: the other point to be Dotted with
// return the scalar floar representing the Dot product
float Point::Dot(Point p) {
  return p.x_*x_ + p.y_*y_ + p.z_*z_; 
}

// return the float Magnitude
float Point::Magnitude() {
  return sqrt(pow(x_, 2)+pow(y_, 2)+pow(z_, 2));
}

// Normalize finds the unit vector
// return the unit vector
Point Point::Normalize() {
  Point p;
  float mag = Magnitude();
  if (mag == 0) {
    p.Init();
    return p;
  }
  p.x_ = x_ / mag;
  p.y_ = y_ / mag;
  p.z_ = z_ / mag;
  return p;
}

// DistanceToPoint finds the distance from this point to another
// p: the other point to compare with
// return the distance between this and p
float Point::DistanceToPoint(Point p) {
  return Sub(p).Magnitude();
}
// VectorPerpendicularTo finds the vector perpendicular to a line
// line: a vector of two points that define a line in R3
// return: the shortest distance from this point to that line
Point Point::VectorPerpendicularTo(vector<Point> line) {
  Point v = line[1].Sub(line[0]);
  Point u = this->Sub(line[0]);
  return u.Sub(v.Normalize().Times(v.Dot(u)));
}
// Print returns the string representation of a point
// return this string "(x_, y_, z_)"
string Point::Print() {
  stringstream s;
  s << "(" << x_ << "," << y_ << "," << z_ << ")";
  return s.str();
}
// FindById Finds a Point by its ID from a vector of points
// id_: the id of the point desired
// points: a vector of points to look in
// return an iterator to that point
const vector<Point>::iterator FindPointById(int id_, vector<Point> &points) {
  vector<Point>::iterator iter;
  for (iter = points.begin(); iter != points.end(); ++iter) {
    if (iter->id_ == id_)
      return iter;
  }
  return points.end();
}
::std::ostream& operator<<(::std::ostream& os, const Point& p) {
  return os << "[" << p.id_ << "," << p.current_ << "]:("
            << p.x_ << "," << p.y_ << "," << p.z_ << ")";
}

}  //  namespace object_server
