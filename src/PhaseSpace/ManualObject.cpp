// File: ManualObject.cpp
// Author: Dylan Visher
// Date: 5/18/13
// About: Logic for ManualObject class, extension of Object

#include "./ManualObject.h"

namespace object_server {

using quaternions::QRotate;
using quaternions::Qinv;
using std::vector;

void ManualObject::init(int ident, string called) {
  id = ident;
  name = called;
  ext = "ManualObject";
  SetCenter(0, 0, 0);
  SetAngle(1, 0, 0, 0);
  SetDim(.01, .01, .01);
  update();
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
bool ManualObject::SetAngle (float ang[4]) {
  angle.clear();
  angle.push_back(ang[0]);
  angle.push_back(ang[1]);
  angle.push_back(ang[2]);
  angle.push_back(ang[3]);
  update();
  return true;
}
bool ManualObject::SetAngle (vector<float> ang) {
  if (angle.size() != 4) return false;
  angle.clear();
  angle.push_back(ang[0]);
  angle.push_back(ang[1]);
  angle.push_back(ang[2]);
  angle.push_back(ang[3]);
  update();
  return true;
}

bool ManualObject::SetAngle (float w, float x, float y, float z) {
  angle.clear();
  angle.push_back(w);
  angle.push_back(x);
  angle.push_back(y);
  angle.push_back(z);
  update();
  return true;
}
bool ManualObject::SetDim (float x, float y, float z) {
  dim.clear();
  dim.push_back(x);
  dim.push_back(y);
  dim.push_back(z);
  return true;
}
bool ManualObject::Update(OWLMarker *marks, int i) {
  update();
  return true;
}
bool ManualObject::update() {
  Point x_axis;
  x_axis.init(1, 0, 0);
  Point y_axis;
  y_axis.init(0, 1, 0);
  x_axis = QRotate(x_axis, Qinv(angle));
  y_axis = QRotate(y_axis, Qinv(angle));
  Point z_axis = x_axis.cross(y_axis);
  axes.clear();
  axes.push_back(x_axis);
  axes.push_back(y_axis);
  axes.push_back(z_axis);
  return true;
}

} // namespace object_server
