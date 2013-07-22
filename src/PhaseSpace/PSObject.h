// File: PSObject.h
// Author: Dylan Visher
// Date: 5/18/13
// About: Object Extension for PhaseSpace Objects

#ifndef _SHL_COREOBJECTSERVER_PSOBJECT_H
#define _SHL_COREOBJECTSERVER_PSOBJECT_H

#include <vector>
#include <string>
#include "owl/owl.h"
#include "./Object.h"
#include "./ObjectType.h"
#include "./GloveType.h"
#include "./DefaultType.h"

namespace object_server {

using std::vector;
using std::string;
using object_server::Object;

class PSObject : public Object {
 protected:
  /*ObjectType for storing object type specific information*/
  ObjectType* type;

 public:
  // init initializes the PSObject with the given information
  // identification : the id of the object
  // called : the name of the object
  // points : a vector of Points that define the PhaseSpace Object
  // t : the type of object
  void init(int identification, string called, vector<Point> points, string t);
  // get_pointer returns a Point that represents where this object is pointing
  //   in this case that information is stored
  //    within the object type specific information
  Point get_pointer() {
    return type->get_pointer();
  }
  // get_points returns the points that define the PhaseSpace Object
  // return a vector of these points
  vector<Point> get_points() {
    return type->get_points();
  }
  
  // AddPoints adds points to this object
  // new_points : a vector of new Point s to add to this object
  virtual bool AddPoints(vector<Point> new_points) {
    return type->AddPoints(new_points);
  }
  // Update updates the PhaseSpace Object
  // Uses updated PhaseSpace OWLMarker s to update points
  // n : the number of markers that have been updated
  bool Update(OWLMarker *markers, int n);
};
}  // namespace object_server

#endif
