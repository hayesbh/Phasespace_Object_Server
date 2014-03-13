// File: PSObject.cpp
// Author: Dylan Visher
// Date: 5/18/13
// About: Logic for PhaseSpace Objects

#include <string>
#include <vector>
#include "./PSObject.h"

namespace Phasespace_Object_Server {

// Init initializes the PSObject with the given information
// identification : the id of the object
// called : the name of the object
// points : a vector of Points that define the PhaseSpace Object
// t : the type_ of object
bool PSObject::Init(int ident, string call,
                    vector<Point> points, string t, bool rigid) {
  Point p;
  p.Init();
  center_ = p;
  angle_.push_back(1);
  angle_.push_back(0);
  angle_.push_back(0);
  angle_.push_back(0);
  Point x_axis;
  x_axis.Init(1, 0, 0);
  Point y_axis;
  y_axis.Init(0, 1, 0);
  Point z_axis;
  z_axis.Init(0, 0, 1);
  axes_.push_back(x_axis);
  axes_.push_back(y_axis);
  axes_.push_back(z_axis);
  dim_.push_back(.1);
  dim_.push_back(.1);
  dim_.push_back(.1);
  
  name_ = call;
  id_ = ident;
  bool success;
  if (t == "glove") {
    GloveType* g = new GloveType;
    success = g->Init(points, rigid);
    type_ = static_cast<ObjectType*>(g);
    ext_ = "glove";
  } else {
    DefaultType* d = new DefaultType;
    success = d->Init(points, rigid);
    type_ = static_cast<ObjectType*>(d);
    ext_ = "default";
  }
  UpdateFields();
  return success;
}


bool PSObject::UpdateFields(){
  type_->UpdateFields();
  center_ = type_->get_center();
  angle_ = type_->get_angle();
  axes_ = type_->GetAxes();
  dim_ = type_->get_dimensions();
  return true;
}

// Update updates the PhaseSpace Object
// Uses updated PhaseSpace OWLMarker s to update points
// n : the number of markers that have been updated
bool PSObject::Update(OWLMarker *markers, int n) {
  type_->Update(markers, n);
  UpdateFields();
  return true;
}

}  // namespace Phasespace_Object_Server
