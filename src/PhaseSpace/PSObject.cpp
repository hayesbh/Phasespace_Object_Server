// File: PSObject.cpp
// Author: Dylan Visher
// Date: 5/18/13
// About: Logic for PhaseSpace Objects

#include <string>
#include <vector>
#include "./PSObject.h"

namespace object_server {

// Init initializes the PSObject with the given information
// identification : the id of the object
// called : the name of the object
// points : a vector of Points that define the PhaseSpace Object
// t : the type of object
void PSObject::Init(int ident, string call,
                    vector<Point> points, string t, bool rigid) {
  Point p;
  p.Init();
  center = p;
  angle.push_back(1);
  angle.push_back(0);
  angle.push_back(0);
  angle.push_back(0);
  Point x_axis;
  x_axis.Init(1, 0, 0);
  Point y_axis;
  y_axis.Init(0, 1, 0);
  Point z_axis;
  z_axis.Init(0, 0, 1);
  axes.push_back(x_axis);
  axes.push_back(y_axis);
  axes.push_back(z_axis);
  dim.push_back(.1);
  dim.push_back(.1);
  dim.push_back(.1);
  
  name = call;
  id = ident;
  if (t == "glove") {
    GloveType* g = new GloveType;
    g->Init(points, rigid);
    type = static_cast<ObjectType*>(g);
    ext = "glove";
  } else {
    DefaultType* d = new DefaultType;
    d->Init(points, rigid);
    type = static_cast<ObjectType*>(d);
    ext = "default";
  }
  printf("In ps Object\n");
  center = type->get_center();
  angle = type->get_angle();
  axes = type->GetAxes();
  dim = type->get_dimensions();
  printf("pre-updated\n");
  update();
}


bool PSObject::update(){
  printf("in PSObject::update\n");
  type->update();
  printf("type updated\n");
  center = type->get_center();
  angle = type->get_angle();
  axes = type->GetAxes();
  dim = type->get_dimensions();
  return true;
}

// Update updates the PhaseSpace Object
// Uses updated PhaseSpace OWLMarker s to update points
// n : the number of markers that have been updated
bool PSObject::Update(OWLMarker *markers, int n) {
  printf("Updating PSObject\n");
  type->Update(markers, n);
  center = type->get_center();
  angle = type->get_angle();
  axes = type->GetAxes();
  dim = type->get_dimensions();
  return true;
}

}  // namespace object_server
