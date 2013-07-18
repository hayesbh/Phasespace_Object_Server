// File: PSObject.cpp
// Author: Dylan Visher
// Date: 5/18/13
// About: Logic for PhaseSpace Objects

#include "PSObject.h"

// init initializes the PSObject with the given information
// identification : the id of the object
// called : the name of the object
// points : a vector of Points that define the PhaseSpace Object
// t : the type of object
void PSObject::init(int identification, string called, vector<Point> points, string t){
  name = called;
  id = identification;
  if(t == "glove") {
    GloveType g;
    g.init(points);
    type = g;
    ext = "glove";
  }
  else {
    DefaultType d;
    d.init(points);
    ext = "default"
  }
}

// Update updates the PhaseSpace Object
// Uses updated PhaseSpace OWLMarker s to update points
// n : the number of markers that have been updated
void Update(OWLMarker *markers, int n) {
  type.Update(markers, n);
  center = type.get_center();
  angle = get_angle();
}

const vector<Point>::iterator FindPointById(int id, vector<Point> &points) {
  vector<Point>::iterator iter;
  for (iter = points.begin(); iter != points.end(); ++iter) {
    if (iter->id == id)
      return iter;
  }
  return points.end();
}


