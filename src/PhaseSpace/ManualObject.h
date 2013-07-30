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
  void Init(int ident, string called);
  string get_type() {
    return "manual";
  }
  bool get_rigidity() {
    return true;
  }
  bool SetCenter (Point c);
  bool SetCenter (float x, float y, float z);
  bool SetAngle (float angle[4]);
  bool SetAngle (vector<float> angle);
  bool SetAngle (float w, float x, float y, float z);
  bool SetDim (float x, float y, float z);
  bool update();
  bool Update (OWLMarker *marks, int n);
  bool AddPoints(vector<Point> new_points){
    return false;
  }
  virtual Point get_pointer() {
    return center;
  }
};

}  // namespace object_server
#endif
