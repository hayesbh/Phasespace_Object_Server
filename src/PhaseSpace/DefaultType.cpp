// File: DefaultType.cpp
// Author: Dylan Visher
// Date: 5/18/13
// About: Logic for Default Type

#include "./DefaultType.h"

namespace Phasespace_Object_Server {

// Math And Vectors
using std::vector;

using std::pow;
// Quaternions
using quaternions::QMult;
using quaternions::QNormalize;
using quaternions::QInv;
using quaternions::QRotate;

// init initializes the Object to hold in it the points given
// p: The points that this object will track
// rig: bool value indicating whether the object is rigid (true if it is)
// return: bool indicated whether this object was successfully initialized
bool DefaultType::Init (vector<Point> p, bool rig) {
  points_ = p;
  // Need DefaultState because of loading
  // Just in case none of the points of the object are current
  original_axis1_.Init(1, 0, 0);
  original_axis2_.Init(0, 1, 0);
  axis1_.Init(1, 0, 0);
  axis2_.Init(0, 1, 0);
  angle_.push_back(1);
  angle_.push_back(0);
  angle_.push_back(0);
  angle_.push_back(0);
  center_.Init(0, 0, 0);
  dim_.push_back(.01);
  dim_.push_back(.01);
  dim_.push_back(.01);
  // Get the Center
  GetCenter(1);
  // Get the Angle information
  GetAngle(1);
  // Get the Dimensional information
  GetScale(1);
  rigid_ = rig;
  UpdateFields();
  return true;
}
// get_pointer returns the pointer of the Object (specifically in the glove's case)
Point DefaultType::get_pointer() {
  return center_;
}

// GetFirstAngleAxis finds (and sets if first time of i == 1) the local x_axis
// default first angle axis is defined by the two points with the y values closest to zero
// the vector points in the +x direction
// i: default set to 0 but if it is ever set to 1 the first axis will be reset according to the rules above
// return: bool indicating whether this was successful at getting the first axis
bool DefaultType::GetFirstAxis(int i = 0){
  if (axis_points1_.size() == 2 && i == 0) {
    if(axis_points1_[0]->current_ == 0 || axis_points1_[1]->current_ == 0) {
      return false;
    }
    axis1_ = axis_points1_[1]->Sub(*axis_points1_[0]).Normalize();
    return true;
  } else {
    // If there are not enough points_ then just return the 0 vector
    if (points_.size() < 2) {
      return false;
    }
    // compare with max distance squared
    float low1 = 100000000000;
    float low2 = 1000000000000;
    // P1 and P2 are the Points that define the vector
    vector<Point>::iterator P1 = points_.end();
    vector<Point>::iterator P2 = points_.end();
    /*Iterators for going through the points_ that define the object*/
    vector<Point>::iterator iter;
    for (iter = points_.begin(); iter != points_.end(); ++iter) {
      if (!iter->current_) continue;
      // low1 is the smaller of the twop lows
      if (iter->y_ < low1) {
        low2 = low1;
        low1 = iter->y_;
        P2 = P1;
        P1 = iter;
      } else if (iter->y_ < low2) {
        low2 = iter->y_;
        P2 = iter;
      } 
    }
    if (P1 == points_.end() || P2 == points_.end()) {
      axis1_.Init(1, 0, 0);
      return false;  
    }
    // Remember the points_ that make up this axis
    // for later reference when finding the angle_ from the start
    vector<Point>::iterator temp;
    if (P2->x_ < P1->x_) {
      temp = P2;
      P2 = P1;
      P1 = temp;
    }
    axis_points1_.clear();
    axis_points1_.push_back(P1);
    axis_points1_.push_back(P2);
    original_axis1_ = P2->Sub(*P1).Normalize();
    axis1_ = original_axis1_;
    return true;
  }
}
// GetBothAxes ginds both Axes (calls on GetFirstAxis) and sets both (if the first time or i == 1)
// this is the local y axis and is defined by the point that has the closest x value to one of the base points
// these axes that are initialized define the angle of the object
// i: default set to 0 but if it is ever set to 1 the second axis and first axis will be reset according to the rules above
// return: bool that indicates whether both axes were set
bool DefaultType::GetBothAxes(int i = 0){
  if(!GetFirstAxis(i)) {
    return false;
  }
  if (axis_points1_.size() == 2 && axis_points2_.size() == 2 &&  i == 0) {
    Point u;
    // Make sure that both axes are up to date
    if(axis_points1_[0]->current_ == 0 || axis_points1_[1]->current_ == 0 || axis_points2_[0]->current_ == 0 || axis_points2_[1]->current_ == 0 ) {
      return false;
    } else {
      u = axis_points2_[1]->Sub(*axis_points2_[0]);
    }
    Point v;
    // upon initial set up ensure that the x_axis is in the +x direction --> right handed coordinate system
    if(axis_points1_[1] == axis_points2_[0]) {
      v = axis_points1_[0]->Sub(*axis_points1_[1]);
    } else {
      v = axis_points1_[1]->Sub(*axis_points1_[0]);
    }
    // find the normal component between the two vectors
    axis2_ = u.Sub(v.Normalize().Times(u.Dot(v))).Normalize();
    return true;
  } else {
    // Make sure that there are enough points_ to define this second axis
    if (points_.size() < 3) {
      return false;
    }
    // Make sure the first axis exists
    if(axis_points1_[0]->current_ == 0 || axis_points1_[1]->current_ == 0) {
      return false;
    }
    //  distance found so far
    float x_dist = 10000000;
    // P1 and P2 are the Points that define the vector for the y_axis
    // minimize the distance in regards to the x_axis
    vector<Point>::iterator P1 = points_.end();
    vector<Point>::iterator P2 = points_.end();
    vector<Point>::iterator iter;
    for(int i = 0; i <= 1; ++i) {
      vector<Point>::iterator temp_base = axis_points1_[i];
      for (iter = points_.begin(); iter != points_.end(); ++iter) {
        if(iter == axis_points1_[0] || iter == axis_points1_[1]) continue;
        if(!(iter->current_)) continue;
        float temp_dist;
        if ((temp_dist = fabs(iter->Sub(*temp_base).x_)) < x_dist) {
          P1 = temp_base;
          P2 = iter;
          x_dist = temp_dist;
        }
      }
    }
    // if there were not enough current points then try again next time GetBothAxes is called
    if (P1 == points_.end() || P2 == points_.end()) {
      return false;
    }
    axis_points2_.clear();
    // Make sure that the Points are remembered for future reference
    axis_points2_.push_back(P1);
    axis_points2_.push_back(P2);
    axis2_ = original_axis2_;
    return true;
  }
}
}  // namespace Phasespace_Object_Server
