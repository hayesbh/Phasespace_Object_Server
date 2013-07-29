// File: ObjectType.cpp
// Author: Dylan visher
// Date: 5/18/13
// About: Logic for the ObjectType Class

#include "ObjectType.h"

namespace object_server {

using std::vector;
using quaternions::Qnormalize;
using quaternions::QRotate;
using quaternions::Qmult;
using quaternions::Qinv;
// Reset resets all the stored information and finds it again
void ObjectType::reset(){
  GetCenter(1);
  GetAngle(1);
  GetScale();
}
// GetAxes default is the right handed coordinate system defined by the first and second Axes
vector<Point> ObjectType::GetAxes() {
  vector<Point> axes;
  axes.push_back(Axis1);
  axes.push_back(Axis2);
  axes.push_back(Axis1.cross(Axis2).normalize());
  return axes;
}
// Update the points with new marker information
void ObjectType::Update(OWLMarker *markers, int n) {
  for (int i = 0; i < n; ++i) {
    vector<Point>::iterator iter;
      for (iter = points.begin(); iter != points.end(); ++iter) {
        if (markers[i].id == iter->id) {
          if (markers[i].cond > 0)
            iter->Update(markers[i]);
          else
              iter->current = 0;
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
  if(dimension == 0) axis.init(1, 0, 0);
  else if(dimension == 1) axis.init(0, 1, 0);
  else axis.init(0, 0, 1);
  /*Rotate that axis to find the local axis*/
  axis = QRotate(axis, angle);
  /*Find the maximum amount a vector lies along that axis*/
  vector<Point>::iterator iter;
  for(iter = points.begin(); iter != points.end(); ++iter) {
    if(!(iter->current)) continue;
    float temp;
    if((temp = fabs(iter->sub(center).dot(axis))) > max)
      max = temp;
  } return max;
}
// GetScale finds and sets the dimensions of the object
void ObjectType::GetScale(int i){
  if (rigid) return;
  printf("getting Scale\n");
  float buffer = 1.10;
  dim.clear();
  dim.push_back(2 * MaxDimensionalDistance(0) * buffer);
  dim.push_back(2 * MaxDimensionalDistance(1) * buffer);
  dim.push_back(2 * MaxDimensionalDistance(2) * buffer);
  printf("scale recived\n");
}
// AddPoints adds new_points to the object
// new_points is a list of Points that are to be added to the Object
// The default is to also reset the AngleAxes to this new set of points
bool ObjectType::AddPoints(vector<Point> new_points) {
  vector<Point>::iterator i;
  points.insert(points.end(), new_points.begin(), new_points.end());
  GetCenter();
  GetAngle();
  GetScale();
  return true;
}
// GetCenter is finds the center of the object
// The standard is just to find the average
void ObjectType::GetCenter(int i) {
  Point p;
  p.init();
  int pointsUsed = 0;
  /*The Default Center is the Average of the points*/
  vector<Point>::iterator iter;
  for (iter = points.begin(); iter != points.end(); ++iter) {
    if (iter->current) {
      p.x += iter->x;
      p.y += iter->y;
      p.z += iter->z;
      pointsUsed++;
    }
  }
  if (pointsUsed != 0) {
    p.x /= pointsUsed;
    p.y /= pointsUsed;
    p.z /= pointsUsed;
    center = p;
    return;
  } else {
    center = points[0];
    return;
  }
}
// GetAngle sets the angle Quaterion to represent this Object's rotation from its original position
// This angle is the rotation required to turn the current vector of the x-axis to the original x-axis vector
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
  Point cross1 = OriginalAxis1.cross(Axis1);
  Q1.push_back(sqrt(pow(Axis1.magnitude(), 2) * pow(OriginalAxis1.magnitude(), 2)) + Axis1.dot(OriginalAxis1));
  Q1.push_back(cross1.x);
  Q1.push_back(cross1.y);
  Q1.push_back(cross1.z);
  angle = Qnormalize(Q1);
  //return;
  // If the Second Axis is unpopulated it was unable to find two points
  if (AxisPoints2.size() != 2) {
    return;
  }
  if(OriginalAxis2.magnitude() == 0 || Axis2.magnitude() == 0) {
    return;
  }
  vector<float> Q2;
  Point temp = QRotate(Axis2, Qinv(angle));
  Point cross2 = OriginalAxis2.cross(temp);
  // Point cross2 = temp.cross(OriginalAxis2).times(.5);
  Q2.push_back(sqrt(pow(temp.magnitude(), 2) * pow(OriginalAxis2.magnitude(), 2)) + temp.dot(OriginalAxis2));
  Q2.push_back(cross2.x);
  Q2.push_back(cross2.y);
  Q2.push_back(cross2.z);
  // angle = Q;
  Q2 = Qnormalize(Q2);
  printf("final2\n");
  angle = Qnormalize(Qmult(Q1, Q2));
  printf("final3\n");
  return;
}
}  // namespace object_server
