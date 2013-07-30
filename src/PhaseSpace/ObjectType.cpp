// File: ObjectType.cpp
// Author: Dylan visher
// Date: 5/18/13
// About: Logic for the ObjectType Class

#include "ObjectType.h"

namespace object_server {

using std::vector;
using quaternions::QNormalize;
using quaternions::QRotate;
using quaternions::QMult;
using quaternions::QInv;
// Reset resets all the stored information and finds it again
void ObjectType::reset(){
  GetCenter(1);
  GetAngle(1);
  GetScale();
}
// GetAxes default is the right handed coordinate system defined by the first and second Axes
vector<Point> ObjectType::GetAxes() {
  vector<Point> axes;
  axes.push_back(axis1_);
  axes.push_back(axis2_);
  axes.push_back(axis1_.Cross(axis2_).Normalize());
  return axes;
}
// Update the points with new marker information
void ObjectType::Update(OWLMarker *markers, int n) {
  for (int i = 0; i < n; ++i) {
    vector<Point>::iterator iter;
      for (iter = points_.begin(); iter != points_.end(); ++iter) {
        if (markers[i].id == iter->id_) {
          if (markers[i].cond > 0)
            iter->Update(markers[i]);
          else
              iter->current_ = 0;
          }
       }
  }
  // Update Center information
  GetCenter();
  // Get the Angle Information
  GetAngle();
  // Get the Dimensional Information
  GetScale();
  return;
}
bool ObjectType::update() {
  GetCenter();
  printf("GetCenter() complete\n");
  GetAngle();
  printf("GetAngle() complete\n");
  GetScale();
  printf("GetScale() complete\n");
  return true;
}
// MaxDimensionalDistance find the dimensions along the local axis
// dimension: int representing axis (0 --> x axis, 1 --> y axis, 2 --> z axis)
// return  float giving the largest distance from the center along that axis
//         if the dimensional distance is 0 then return .01
float ObjectType::MaxDimensionalDistance(int dimension) {
  float max = .01;
  Point axis;
  /*find what axis we are looking at*/
  if(dimension == 0) axis.Init(1, 0, 0);
  else if(dimension == 1) axis.Init(0, 1, 0);
  else axis.Init(0, 0, 1);
  /*Rotate that axis to find the local axis*/
  axis = QRotate(axis, angle_);
  /*Find the maximum amount a vector lies along that axis*/
  vector<Point>::iterator iter;
  for(iter = points_.begin(); iter != points_.end(); ++iter) {
    if(!(iter->current_)) continue;
    float temp;
    if((temp = fabs(iter->Sub(center_).Dot(axis))) > max)
      max = temp;
  } return max;
}
// GetScale finds and sets the dimensions of the object
void ObjectType::GetScale(int i){
  if (rigid_) return;
  printf("getting Scale\n");
  float buffer = 1.10;
  dim_.clear();
  dim_.push_back(2 * MaxDimensionalDistance(0) * buffer);
  dim_.push_back(2 * MaxDimensionalDistance(1) * buffer);
  dim_.push_back(2 * MaxDimensionalDistance(2) * buffer);
  printf("scale recived\n");
}
// AddPoints adds new_points to the object
// new_points is a list of Points that are to be added to the Object
// The default is to also reset the AngleAxes to this new set of points
bool ObjectType::AddPoints(vector<Point> new_points) {
  vector<Point>::iterator i;
  points_.insert(points_.end(), new_points.begin(), new_points.end());
  GetCenter();
  GetAngle();
  GetScale();
  return true;
}
// GetCenter is finds the center of the object
// The standard is just to find the average
void ObjectType::GetCenter(int i) {
  Point p;
  p.Init();
  int pointsUsed = 0;
  /*The Default Center is the Average of the points*/
  vector<Point>::iterator iter;
  for (iter = points_.begin(); iter != points_.end(); ++iter) {
    if (iter->current_) {
      p.x_ += iter->x_;
      p.y_ += iter->y_;
      p.z_ += iter->z_;
      pointsUsed++;
    }
  }
  if (pointsUsed != 0) {
    p.x_ /= pointsUsed;
    p.y_ /= pointsUsed;
    p.z_ /= pointsUsed;
    center_ = p;
    return;
  } else {
    center_ = points_[0];
    return;
  }
}
// GetAngle sets the angle Quaterion to represent this Object's rotation from its original position
// This angle is the rotation required to turn the current_ vector of the x-axis to the original x-axis vector
// And the rotation to turn the y-axis to the original y-axis vector
void ObjectType::GetAngle(int i) {
  printf("GetBothAxes(%i) getting\n", i);
  bool success = GetBothAxes(i);
  // If the Axis is unpopulated, it was unable to find two points
  if (!success) {
    printf("GetBothAxes(%i) failed\n", i);
    return;
  }
  printf("GetBothAxes(%i) succeeded\n", i);
  vector<float> Q1;
  Point cross1 = original_axis1_.Cross(axis1_);
  Q1.push_back(sqrt(pow(axis1_.Magnitude(), 2) * pow(original_axis1_.Magnitude(), 2)) + axis1_.Dot(original_axis1_));
  Q1.push_back(cross1.x_);
  Q1.push_back(cross1.y_);
  Q1.push_back(cross1.z_);
  angle_ = QNormalize(Q1);
  //return;
  // If the Second Axis is unpopulated it was unable to find two points
  if (axis_points2_.size() != 2) {
    return;
  }
  if(original_axis2_.Magnitude() == 0 || axis2_.Magnitude() == 0) {
    return;
  }
  vector<float> Q2;
  Point temp = QRotate(axis2_, QInv(angle_));
  Point cross2 = original_axis2_.Cross(temp);
  Q2.push_back(sqrt(pow(temp.Magnitude(), 2) * pow(original_axis2_.Magnitude(), 2)) + temp.Dot(original_axis2_));
  Q2.push_back(cross2.x_);
  Q2.push_back(cross2.y_);
  Q2.push_back(cross2.z_);
  // angle = Q;
  Q2 = QNormalize(Q2);
  printf("final2\n");
  angle_ = QNormalize(QMult(Q1, Q2));
  printf("final3\n");
  return;
}
}  // namespace object_server
