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
  }
  /**
   * [init initialize the point with no coordinates]
   * @return [default point at origin]
   */
  void init() {
    current = 0;
    x = y = z = 0;
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
    c.z = x * p.y - y * p.x;
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
    x /= mag;
    y /= mag;
    z /= mag;
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
  float distanceToLine(std::vector<Point> line) {
    Point x2x1 = line[1].sub(line[0]);
    return (x2x1.cross(line[0].sub(*this))).magnitude()
                / (x2x1).magnitude();
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

#endif
