// File: ManualObject.cpp
// Author: Dylan Visher
// Date: 5/18/13
// About: Logic for ManualObject class, extension of Object

#include "ManualObject.h"

void ManualObject::init(int ident, string called) {
  id = ident;
  name = called;
  ext = "manual";
}  
bool ManualObject::SetCenter (Point c) {
  center = c;
  return true;
}
bool ManualObject::SetCenter (float x, float y, float z) {
  center.x = x;
  center.y = y;
  center.z = z;
  return true;
}
bool ManualObject::SetAngle (float angle[4]) {
  angle.clear();
  angle.push_back(angle[0]);
  angle.push_back(angle[1]);
  angle.push_back(angle[2]);
  angle.push_back(angle[3]);
  return true;
}
bool ManualObject::SetAngle (vector<float> angle) {
  if (angle.size() != 4) return false;
  angle.clear();
  angle.push_back(angle[0]);
  angle.push_back(angle[1]);
  angle.push_back(angle[2]);
  angle.push_back(angle[3]);
  return true;
}

bool ManualObject::SetAngle (float w, float x, float y, float z) {
  angle.clear();
  angle.push_back(w);
  angle.push_back(x);
  angle.push_back(y);
  angle.push_back(z);
  return true;
}
bool ManualObject::SetDim (float x, float y, float z) {
  dim.clear();
  dim.push_back(x);
  dim.push_back(y);
  dim.push_back(z);
  return true;
}

