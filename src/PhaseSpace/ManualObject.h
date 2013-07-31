// File: ManualObject.h
// Author: Dylan Visher
// Date: 5/18/13
// About: Class that Extends Object

#ifndef _SHL_COREOBJECTSERVER_MANUALOBJECT_H
#define _SHL_COREOBJECTSERVER_MANUALOBJECT_H

#include <vector>
#include <string>
#include "Object.h"
#include "quaternion.h"

namespace object_server {

using object_server::Object;

class ManualObject : public Object {
  public:
  // Initialize the ManualObject
  // This just gives it an id number and something to call it
  // This also provides a base state at the origin with .01 width and normal orientation
  void Init(int ident, string called);
  // ManualObjects are of type manual
  string get_type() {
    return "manual";
  }
  // ManualObjects are by definition rigid because you must Set the variables by yourself
  bool get_rigidity() {
    return true;
  }
  // SetCenter sets the center of the object to be in a certain location
  bool SetCenter (Point c);
  bool SetCenter (float x, float y, float z);
  // SetAngle sets the angle of thje object
  bool SetAngle (float angle[4]);
  bool SetAngle (vector<float> angle);
  bool SetAngle (float w, float x, float y, float z);
  // SetDim sets the dimensions of the object
  bool SetDim (float x, float y, float z);
  // UpdateFields updates the fields (axes according to the angle)
  bool UpdateFields();
  // Update (with Markers) so that way update can be called on all objects
  // UpdatesFields and returns true to indicate that it has been updated
  bool Update (OWLMarker *marks, int n);
  // AddPoints returns false to indicate that points cannot be added to this object
  bool AddPoints(vector<Point> new_points){
    return false;
  }
  // get_pointer returns the center of the object
  virtual Point get_pointer() {
    return center_;
  }
};

}  // namespace object_server
#endif
