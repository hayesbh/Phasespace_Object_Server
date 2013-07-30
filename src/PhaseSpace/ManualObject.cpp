// File: ManualObject.cpp
// Author: Dylan Visher
// Date: 5/18/13
// About: Logic for ManualObject class, extension of Object

#include "./ManualObject.h"

namespace object_server {

using quaternions::QRotate;
using quaternions::QInv;
using std::vector;

void ManualObject::Init(int ident, string called) {
  id_ = ident;
  name_ = called;
  SetCenter(0, 0, 0);
  SetAngle(1, 0, 0, 0);
  SetDim(.01, .01, .01);
  UpdateFields();
}  
bool ManualObject::SetCenter (Point c) {
  center_ = c;
  return true;
}
bool ManualObject::SetCenter (float x, float y, float z) {
  center_.x_ = x;
  center_.y_ = y;
  center_.z_ = z;
  return true;
}
bool ManualObject::SetAngle (float ang[4]) {
  angle_.clear();
  angle_.push_back(ang[0]);
  angle_.push_back(ang[1]);
  angle_.push_back(ang[2]);
  angle_.push_back(ang[3]);
  UpdateFields();
  return true;
}
bool ManualObject::SetAngle (vector<float> ang) {
  if (angle_.size() != 4) return false;
  angle_.clear();
  angle_.push_back(ang[0]);
  angle_.push_back(ang[1]);
  angle_.push_back(ang[2]);
  angle_.push_back(ang[3]);
  UpdateFields();
  return true;
}

bool ManualObject::SetAngle (float w, float x, float y, float z) {
  angle_.clear();
  angle_.push_back(w);
  angle_.push_back(x);
  angle_.push_back(y);
  angle_.push_back(z);
  UpdateFields();
  return true;
}
bool ManualObject::SetDim (float x, float y, float z) {
  dim_.clear();
  dim_.push_back(x);
  dim_.push_back(y);
  dim_.push_back(z);
  return true;
}
bool ManualObject::Update(OWLMarker *marks, int i) {
  UpdateFields();
  return true;
}
bool ManualObject::UpdateFields() {
  Point x_axis;
  x_axis.Init(1, 0, 0);
  Point y_axis;
  y_axis.Init(0, 1, 0);
  x_axis = QRotate(x_axis, QInv(angle_));
  y_axis = QRotate(y_axis, QInv(angle_));
  Point z_axis = x_axis.Cross(y_axis);
  axes_.clear();
  axes_.push_back(x_axis);
  axes_.push_back(y_axis);
  axes_.push_back(z_axis);
  return true;
}

} // namespace object_server
