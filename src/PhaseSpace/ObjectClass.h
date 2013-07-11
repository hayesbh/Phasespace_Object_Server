/**
 * File: ObjectClass.h
 * Author: Dylan Visher
 * Date: 5/18/13
 * About: Object Class for Tracking Purposes
 */

#ifndef _SHL_COREOBJECTSERVER_OBJECTCLASS_H
#define _SHL_COREOBJECTSERVER_OBJECTCLASS_H

#include <vector>
#include <string>
/*Including the owl standard library and error messages*/
#include "./ObjectType.h"
#include "./Glove.h"

namespace object_server {

using std::vector;
using std::string;

class ObjectClass {
 private:
  /*The user given name of the object*/
  string name;
  /*The ID assigned by the system*/
  int id;
  /*ObjectType for storing object type specific information*/
  ObjectType type;

 public:
  /**
   * [init initializes the Object with the given information]
   * @param identification [System generated ID for the Object]
   * @param called         [User given name]
   * @param points         [Vector of Points that define Object]
   */
  void init(int identification, string called, vector<Point> points) {
    name = called;
    id = identification;
    type.init(points);
  }
  /**
   * [get_id return system defined id]
   * @return [int id]
   */
  int get_id() {
    return id;
  }
  /**
   * [GetInfo provides return name]
   * @return           [name of object]
   */
  string get_name() {
    return name;
  }
  /**
   * [get_pos finds the position of this object]
   * @return [a point describing location rel to camera 0]
   */
  Point get_pos() {
    return type.GetCenter();
  }
  /**
   * [get_rotation finds the rotation of this object]
   * @return [a quaternion describing rotation]
   */
  vector<float> get_rotation() {
    return type.GetAngle();
  }
  /**
   * [get_points returns the points in Object]
   * @return [a vector of these points]
   */
  vector<Point> get_points() {
    return type.get_points();
  }
  void get_dimensions(float dim[3]){
    type.get_dimensions(dim);
  } 
  /**
   * [AddPoints adds points to this object]
   * @param  new_points [new points to also be in this object]
   * @return            [whether this addition was successful]
   */
  void AddPoints(vector<Point> new_points) {
    type.AddPoints(new_points);
  }
  /**
   * [update updates the objects points with new marker information]
   * @param markers [PhaseSpace markers with new info]
   * @param n       [number of markers in the markers array]
   */
  void Update(OWLMarker *markers, int n) {
    type.Update(markers, n);
  }

  void PrintPoints() {
    type.PrintPoints();
  }
};
}  // namespace object_server

namespace object {
using std::vector;
using object_server::Point;
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

}  // namespace object

#endif
