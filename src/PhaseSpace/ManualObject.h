// File: ManualObject.h
// Author: Dylan Visher
// Date: 5/18/13
// About: Class that Extends Object

#ifndef _SHL_OBJECT_SERVER_SRC_PHASESPACE_MANUALOBJECT_H_
#define _SHL_OBJECT_SERVER_SRC_PHASESPACE_MANUALOBJECT_H_

#include <vector>
#include <string>
#include "Object.h"
#include "quaternion.h"

namespace Phasespace_Object_Server {

using Phasespace_Object_Server::Object;

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
  // Sets the center based on x, y, and z (allows callers not to use Point specific information)
  bool SetCenter (float x, float y, float z);
  // SetAngle sets the angle of the object
  bool SetAngle (float angle[4]);
  bool SetAngle (vector<float> angle);
  bool SetAngle (float w, float x, float y, float z);
  // SetDim sets the dimensions of the object
  bool SetDimensions (float x, float y, float z);
  // UpdateFields updates the fields (axes according to the angle)
  bool UpdateFields();
  // Update (with Markers) so that way update can be called on all objects
  // UpdatesFields and returns true to indicate that it has been updated
  bool Update (OWLMarker *marks, int n);
  // AddPoints returns false to indicate that points cannot be added to this object
  bool AddPoints(vector<Point>& new_points){
    return false;
  }
  // get_pointer returns the center of the object as a Point
  virtual Point get_pointer() {
    return center_;
  }
};

}  // namespace Phasespace_Object_Server
#endif  // _SHL_OBJECT_SERVER_SRC_PHASESPACE_MANUALOBJECT_H_
