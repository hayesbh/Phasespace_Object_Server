// File: Point.cpp
// Author: Dylan Visher
// Date: 5/18/13
// About: Logic for Points

#include "./Point.h"

// Initialize the point with the given x, y and z coordinates
// x, y, z: floats describing location
void Point::init(float X, float Y, float Z) {
  current = 1;
  x = X;
  y = Y;
  z = Z;
  return;
}

// Static initializer for the point
static Point Point::static_init(float X, float Y, float Z) {
  Point p;
  p.init(X, Y, Z);
  return p;
}

// init without arguments initializes the point with at (0,0,0)
void  Point::init() {
  current = 0;
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
  return (p.x == x && p.y == y && p.z == z);
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
// DistanceToPlane finds the distance between a Point and a plane
// plane: an array holding 4 elements representing
//   A, B, C, D in the standard plane equation
// One can convert three points to a plane by using the 
//   PointsToPlane Function
float Point::DistanceToPlane(float plane[4]) {
  return std::fabs(plane[0] * x + plane[1] * y + plane[2] * z + plane[3]) /
             sqrt(pow(plane[0], 2) + pow(plane[1], 2) + pow(plane[2], 2));
}
// print returns the string representation of a point
// return this string "(x, y, z)"
string Point::print() const {
  stringstream s;
  s << "(" << x << "," << y << "," << z << ")";
  return s.str();
}
// PointsToPlane takes in three points and converts it into a plane
// it finds the A, B, C, D of the standard plane equation
// and stores those in the plane array
// return a pointer to this plane array
float* PointsToPlane (Point p1, Point p2, Point p3, float plane[4]) {
  Point n = (p1.sub(p2)).cross(p2.sub(p3));
  float d = n.x * p1.x + n.y * p1.y + n.z * p1.z;
  plane[0] = n.x; plane[1] = n.y; plane[2] = n.z; plane[3] = d;
  return plane;
}
// FindById Finds a Point by its ID from a vector of points
// id: the id of the point desired
// points: a vector of points to look in
// return an iterator to that point
const vector<Point>::iterator FindById(int id, vector<Point> &points) {
  vector<Point>::iterator iter;
  for (iter = points.begin(); iter != points.end(); ++iter) {
    if (iter->id == id)
      return iter;
  }
  return points.end();
}



