// File: PSObject.cpp
// Author: Dylan Visher
// Date: 5/18/13
// About: Logic for PhaseSpace Objects

#include "PSObject.h"

namespace object_server {

// init initializes the PSObject with the given information
// identification : the id of the object
// called : the name of the object
// points : a vector of Points that define the PhaseSpace Object
// t : the type of object
void PSObject::init(int identification, string called, vector<Point> points, string t){
  ext = "PSObject";
  Point p;
  p.init();
  center = p;
  angle.push_back(1);
  angle.push_back(0);
  angle.push_back(0);
  angle.push_back(0);
  Point x_axis;
  x_axis.init(1, 0, 0);
  Point y_axis;
  y_axis.init(0, 1, 0);
  Point z_axis;
  z_axis.init(0, 0, 1);
  axes.push_back(x_axis);
  axes.push_back(y_axis);
  axes.push_back(z_axis);
  dim.push_back(.1);
  dim.push_back(.1);
  dim.push_back(.1);
  
  name = called;
  id = identification;
  if(t == "glove") {
    GloveType* g = new GloveType;
    g->init(points);
    type = dynamic_cast<ObjectType*>(g);
  }
  else {
    DefaultType* d = new DefaultType;
    d->init(points);
    type = dynamic_cast<ObjectType*>(d);
  }
  center = type->get_center();
  angle = type->get_angle();
  axes = type->GetAxes();
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
