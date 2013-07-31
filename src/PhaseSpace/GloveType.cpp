// File : Glove.cpp
// Author: Dylan Visher
// Date: 5/18/13
// About: Logic for the Glove Object Type

#include <vector>
#include "./GloveType.h"

namespace object_server {

// Init sets initializes the points_ of the glove
// finds all of the fingers' specific Points
// Finds the center, the angle_ and the scale
// p: is the list of points for the glove
// rig: is the rigidity of the glove
bool GloveType::Init(vector<Point> p, bool rig) {
  if (p.size() != 7) return false;
  vector<Point>::iterator iter;
  // Push back each of the points_ and blank ones where they are needed
  points_ = p;
  axis1_.Init(1, 0, 0);
  axis2_.Init(0, 1, 0);
  angle_.push_back(1);
  angle_.push_back(0);
  angle_.push_back(0);
  angle_.push_back(0);
  center_.Init(0, 0, 0);
  dim_.push_back(.1);
  dim_.push_back(.1);
  dim_.push_back(.1);
  int mult = points_[0].id_ / 7;
  // Set up finger specific pointers
  for (iter = points_.begin(); iter != points_.end(); ++iter) {
    if ( iter->id_ % 7 == 1 ) thumb_ = iter;
    else if ( iter->id_ % 7 == 2 ) base_left_ = iter;
    else if ( iter->id_ % 7 == 3 ) fore_ = iter;
    else if ( iter->id_ % 7 == 4 ) middle_ = iter;
    else if ( iter->id_ % 7 == 0 ) ring_ = iter;
    else if ( iter->id_ % 7 == 5 ) pinkey_ = iter;
    else if ( iter->id_ % 7 == 6 ) base_right_ = iter;
    if (iter->id_  / 7 != mult) return false;
  }
  GetCenter(1);
  GetAngle(1);
  GetScale(1);
  rigid_ = rig;
  return true;
}

Point GloveType::get_pointer() {
  return *fore_;
}
// GetFirstAxisAngle for a glove is defined by the two leds
// That are located on the base of the hand
bool GloveType::GetFirstAxis(int i) {
  if (axis_points1_.size() == 2) {
    if (base_right_->current_ == 0 || base_left_->current_ == 0) {
      return false;
    }
    axis1_ = base_right_->Sub(*base_left_).Normalize();
    return true;
  } else {
    if (base_right_->current_ == 0 || base_left_->current_ == 0) return false;
    axis_points1_.push_back(base_left_);
    axis_points1_.push_back(base_right_);
    Point x_axis;
    x_axis.Init(1, 0, 0);
    original_axis1_ = x_axis;
    axis1_ = axis_points1_[1]->Sub(*axis_points1_[0]).Normalize();
    return true;
  }
}
// GetSecondAngleAxis looks for the axis defined by the pointer finger
// And the left base
bool GloveType::GetBothAxes(int i) {
  if (!GetFirstAxis(i)) return false;
  if (axis_points2_.size() == 2) {
  // Check the first Axis validity
  if (base_left_->current_ == 0 || base_right_->current_ == 0) {
    return false;
  }
  /*check second axis*/
  Point u;
  if (fore_->current_ == 0) {
    return false;
  } else {
    u = fore_->Sub(*base_left_);
  }
  Point v = fore_->Sub(*base_left_);
  axis2_ = u.Sub(v.Normalize().Times(u.Dot(v))).Normalize();
  // Now find the normal component
  return true;
  } else {
    if (base_left_->current_ == 0 || fore_->current_ == 0) return false;
    axis_points2_.push_back(fore_);
    axis_points2_.push_back(base_left_);
    Point v = fore_->Sub(*base_left_);
    axis2_ =  v.Sub(axis1_.Times(v.Dot(axis1_))).Normalize();
    Point y_axis;
    y_axis.Init(0, 1, 0);
    original_axis2_ = y_axis;
    return true;
  }
}
}  // namespace object_server
