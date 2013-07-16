/**
 * File: Point.h
 * Author: Dylan Visher
 * Date: 5/18/13
 * About: Point Class for describing 3D points
 */

#ifndef _SHL_COREOBJECTSERVER_POINT_H
#define _SHL_COREOBJECTSERVER_POINT_H

#include <vector>
#include <sstream>
#include <string>
#include "./owlAux.h"

namespace object_server {

using std::string;
using std::stringstream;
using std::vector;
/*Point Class for Handling Vector's in the Mathematical Sense*/
class Point {
 public:
  /*ID of point*/
  int id;
  /*1 for current, 0 for outdated*/
  int current;
  /*X,Y,Z coordinates*/
  float x;
  float y;
  float z;
  /**
   * [init Initialize the Point with given x, y, and z]
   * @param  X [x coordinate]
   * @param  Y [y coordinate]
   * @param  Z [z coordinate]
   * @return   [return the point]
   */
  void init(float X, float Y, float Z) {
    current = 1;
    x = X;
    y = Y;
    z = Z;
    return;
  }
  static Point static_init(float X, float Y, float Z) {
    Point p;
    p.init(X, Y, Z);
    return p;
  }
  /**
   * [init initialize the point with no coordinates]
   * @return [default point at origin]
   */
  void  init() {
    current = 0;
    x = y = z = 0;
    return;
  }
  /**
   * [Update updates the point information with marker info]
   * @param  mark [OWLMarker given by PhaseSpace API]
   * @return      [return this new point]
   */
  void Update(OWLMarker mark) {
    current = 1;
    id = mark.id;
    x = mark.x;
    y = mark.y;
    z = mark.z;
  }
  int equals(Point p) {
    int i = (p.x == x && p.y == y && p.z == z);
    return i;
  }
  /**
   * [sub subtract two points]
   * @param  p [the point to subtract from this]
   * @return   [the evaluated point]
   */
  Point sub(Point p) {
    Point P;
    (p.current && current) ? (P.current = 1) : (P.current = 0);
    P.x = x - p.x;
    P.y = y - p.y;
    P.z = z - p.z;
    return P;
  }
  /**
   * [times scalar multiplication of point]
   * @param  a [the scalar multiple]
   * @return   [return the new point]
   */
  Point times(float a) {
    Point P;
    P.current = current;
    P.x = x*a;
    P.y = y*a;
    P.z = z*a;
    return P;
  }
  /**
   * [add adds two points together]
   * @param  p [Point to Add]
   * @return   [Resulting point]
   */
  Point add(Point p) {
    Point P;
    (p.current && current) ? (P.current = 1) : (P.current = 0);
    P.x = x + p.x;
    P.y = y + p.y;
    P.z = z + p.z;
    return P;
  }
  Point add(float f) {
    Point P;
    P.current = current;
    P.x = x + f;
    P.y = y + f;
    P.z = z + f;
    return P;
  }
  /**
   * [cross performs the cross product of this point on the given]
   * @param  p [the given point]
   * @return   [the resulting point]
   */
  Point cross(Point p) {
    Point c;
    (p.current && current) ? (c.current = 1) : (c.current = 0);
    c.x = y * p.z - z * p.y;
    c.y = z * p.x - x * p.z;
    c.z = -(x * p.y - y * p.x);
    return c;
  }
  /**
   * [dot performs the dot product of this point with another]
   * @param  p [another given point]
   * @return   [the resulting number]
   */
  float dot(Point p) {
    return p.x*x + p.y*y + p.z*z;
  }
  /**
   * [magnitude finds the magnitude of the point vector]
   * @return [the number magnitude]
   */
  float magnitude() {
    return sqrt(pow(x, 2)+pow(y, 2)+pow(z, 2));
  }
  /**
   * [normalize finds the normalized point]
   * @return [the normalized point]
   */
  Point normalize() {
    Point p;
    float mag = magnitude();
    if (mag == 0) {
      printf("normalize failed: mag = 0");
      p.init();
      return p;
    }
    p.x = x / mag;
    p.y = y / mag;
    p.z = z / mag;
    return p;
  }
  /**
   * [DistanceToPoint finds the distance from this point to another]
   * @param  p [a point to find the distance to]
   * @return   [the resulting distance]
   */
  float DistanceToPoint(Point p) {
    return sub(p).magnitude();
  }
  /**
   * [distanceToLine finds the distance from this point to a line]
   * @param  line [two points that define a line]
   * @return      [the distacne]
   */
  Point VectorPerpendicularTo(vector<Point> line) {
    Point v = line[1].sub(line[0]);
    Point u = this->sub(line[0]);
    return u.sub(v.normalize().times(v.dot(u)));
  }
  float DistanceToPlane(float plane[4]) {
    return std::fabs(plane[0] * x + plane[1] * y + plane[2] * z + plane[3]) /
    sqrt(pow(plane[0], 2) + pow(plane[1], 2) + pow(plane[2], 2));
  }
  /**
   * [print returns the string representation of the points x,y,z]
   * @return [this new string]
   */
  string print() const {
    stringstream s;
    s << "(" << x << "," << y << "," << z << ")";
    return s.str();
  }
};
}  // namespace object_server

namespace points {

using object_server::Point;
using std::vector;
float* PointsToPlane (Point p1, Point p2, Point p3, float plane[4]) {
  Point n = (p1.sub(p2)).cross(p2.sub(p3));
  float d = n.x * p1.x + n.y * p1.y + n.z * p1.z;
  plane[0] = n.x; plane[1] = n.y; plane[2] = n.z; plane[3] = d;
  return plane;
}

/**
 * [FindById finds the point with the id given]
 * @param  id     [id of the point desired]
 * @param  points [vector of points to look in]
 * @return        [the index of the point with this id]
 */
const vector<Point>::iterator FindById(int id, vector<Point> &points) {
  vector<Point>::iterator iter;
  for (iter = points.begin(); iter != points.end(); ++iter) {
    if (iter->id == id)
      return iter;
  }
  return points.end();
}


}  // namespace points

#endif
