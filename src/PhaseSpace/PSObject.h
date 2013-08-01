// File: PSObject.h
// Author: Dylan Visher
// Date: 5/18/13
// About: Object Extension for PhaseSpace Objects

#ifndef _SHL_OBJECT_SERVER_SRC_PHASESPACE_PSOBJECT_H_
#define _SHL_OBJECT_SERVER_SRC_PHASESPACE_PSOBJECT_H_

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
  /*ObjectType for storing object type_ specific information*/
  ObjectType* type_;
  string ext_;

 public:
  // init initializes the PSObject with the given information
  // ident : the id of the object
  // call : the name of the object
  // points : a vector of Points that define the PhaseSpace Object
  // t : the type_ of object
  // rig : boolean that indicates that the object is rigid
  bool Init(int ident, string call, vector<Point> points, string t, bool rig);
  // get_pointer returns a Point that represents where this object is pointing
  //   in this case that information is stored
  //    within the object type_ specific information
  Point get_pointer() {
    return type_->get_pointer();
  }
  // get_points returns the points that define the PhaseSpace Object
  // return a vector of these points
  vector<Point> get_points() {
    return type_->get_points();
  }
  // get_type_ grabs the type_ of PSobject
  // return a string representing the type_
  //   "glove" if glove PSObject
  //   "default" if just the standard PSObject
  string get_type() {
    return ext_;
  }
  // get_rigidity returns whether the type_ of object is rigid
  // retrun a bool indicating rigidity
  bool get_rigidity() {
    return type_->get_rigidity();
  }
  // get_axis1_ids returns a vector of ints of the Point IDs
  // that describe the first axis
  vector<int> get_axis1_ids() {
    return type_->get_axis1_ids();
  }
  // get_original_axis1 returns a Point vector descibing the original
  //  orientation of the first axis
  Point get_original_axis1() {
    return type_->get_original_axis1();
  }
  // get_original_axis2 returns a Point vector describing the original
  //  orientation of the second axis
  Point get_original_axis2() {
    return type_->get_original_axis2();
  }
  // get_axis1_ids returns a vector of ints (the Point IDs)
  // that describes the second axis
  vector<int> get_axis2_ids() {
    return type_->get_axis2_ids();
  }
  // SetAxis1IDs takes a vector of ints, (size 2)
  //  and sets the First Axis ids to be that
  // return bool indicating success
  //  false if ids are not contained in the object points
  bool SetAxis1IDs(vector<int> ids) {
    return type_->SetAxis1IDs(ids);
  }
  // SetAxis2IDs takes a vector of ints, (size 2)
  //  and sets the Second Axis ids to be that
  // return bool indicating success
  //  false if ids are not contained in the object points
  bool SetAxis2IDs(vector<int> ids) {
    return type_->SetAxis2IDs(ids);
  }
  // SetOriginalAxis1 takes in a Point vector
  // p: vector to set the original_axis1_ as
  // return bool to indicate whether this was successfully set
  bool SetOriginalAxis1(Point p) {
    return type_->SetOriginalAxis1(p);
  }
  // SetOriginalAxis2 takes in a Point vector
  // p: vector to set the original_axis2_ as
  // return bool to indicate whether this was successfully set
  bool SetOriginalAxis2(Point p) {
    return type_->SetOriginalAxis2(p);
  }
  // SetDimensions sets the dimensions of the object to be x, y, z
  //   for the local axes respectively
  // These only will stick if the object is rigid
  bool SetDimensions(float x, float y, float z) {
    dim_.clear();
    dim_.push_back(x);
    dim_.push_back(y);
    dim_.push_back(z);
    return type_->SetDimensions(x, y, z);
  }
  // SetRigid sets the rigidity of the object to equal "rigid"
  bool SetRigid(bool rigid) {
    return type_->SetRigid(rigid);
  }
  // AddPoints adds points to this object
  // new_points : a vector of new Point s to add to this object
  // return bool indicating whether this addition was successful
  virtual bool AddPoints(vector<Point> new_points) {
    return type_->AddPoints(new_points);
  }
  // Update updates the PhaseSpace Object
  // Uses updated PhaseSpace OWLMarker s to update points
  // n : the number of markers that have been updated
  // return bool indicating whether this update was successful
  bool Update(OWLMarker *markers, int n);
  // UpdateFields updates the fields of the object to represent all of the information
  //  Within the object (useful for Updating the Object if there are no new points)
  //                    (or you have one or more of the objects parameters to equal something)
  bool UpdateFields();
  // reset resets the object (in the case of default object it refinds the original axes)
  void reset(){
    type_->reset();
  }
};
}  // namespace object_server

#endif  // _SHL_OBJECT_SERVER_SRC_PHASESPACE_PSOBJECT_H_
