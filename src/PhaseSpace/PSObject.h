// File: PSObject.h
// Author: Dylan Visher
// Date: 5/18/13
// About: Object Extension for PhaseSpace Objects

#ifndef _SHL__OBJECT_SERVER_SRC_PHASESPACE_PSOBJECT_H_
#define _SHL__OBJECT_SERVER_SRC_PHASESPACE_PSOBJECT_H_

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
  string ext;

 public:
  // init initializes the PSObject with the given information
  // ident : the id of the object
  // call : the name of the object
  // points : a vector of Points that define the PhaseSpace Object
  // t : the type of object
  // rig : boolean that indicates that the object is rigid
  void init(int ident, string call, vector<Point> points, string t, bool rig);
  ~PSObject() {
    delete type;
  }
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
  // get_type grabs the type of PSobject
  // return a string representing the type
  string get_type() {
    return ext;
  }
  // get_rigidity returns whether the type of object is rigid
  // retrun a bool indicating rigidity
  bool get_rigidity() {
    return type->get_rigidity();
  }
  vector<int> get_axis1_ids() {
    return type->get_axis1_ids();
  }
  Point get_OriginalAxis1() {
    return type->get_OriginalAxis1();
  }
  Point get_OriginalAxis2() {
    return type->get_OriginalAxis2();
  }
  vector<int> get_axis2_ids() {
    return type->get_axis2_ids();
  }
  bool SetAxis1IDs(vector<int> ids) {
    return type->SetAxis1IDs(ids);
  }
  bool SetAxis2IDs(vector<int> ids) {
    return type->SetAxis2IDs(ids);
  }
  bool SetOriginalAxis1(Point p) {
    return type->SetOriginalAxis1(p);
  }
  bool SetOriginalAxis2(Point p) {
    return type->SetOriginalAxis2(p);
  }
  bool SetDimensions(float x, float y, float z) {
    return type->SetDimensions(x, y, z);
  }
  bool SetRigid(bool rigid) {
    return type->SetRigid(rigid);
  }
  // AddPoints adds points to this object
  // new_points : a vector of new Point s to add to this object
  // return bool indicating whether this addition was successful
  virtual bool AddPoints(vector<Point> new_points) {
    return type->AddPoints(new_points);
  }
  // Update updates the PhaseSpace Object
  // Uses updated PhaseSpace OWLMarker s to update points
  // n : the number of markers that have been updated
  // return bool indicating whether this update was successful
  bool Update(OWLMarker *markers, int n);
  bool update();
  void reset(){
    type->reset();
  }
};
}  // namespace object_server

#endif  // _SHL__OBJECT_SERVER_SRC_PHASESPACE_PSOBJECT_H_
