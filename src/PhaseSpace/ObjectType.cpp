// File: ObjectType.cpp
// Author: Dylan visher
// Date: 5/18/13
// About: Logic for the ObjectType Class

#include "ObjectType.h"

using object_server::Point;
using object_server::ObjectType;
using points::FindByID;

// Reset resets all the stored information and finds it again
void ObjectType::reset(){
  GetCenter(1);
  GetAngle(1);
  GetScale(1);
}
// GetAxes default is the right handed coordinate system defined by the first and second Axes
vector<Point> ObjectType::GetAxes() {
  vector<Point> axes;
  axes.push_back(vAngleAxis1);
  axes.push_back(vAngleAxis2);
  axes.push_back(vAngleAxis1.cross(vAngleAxis2).normalize());
  return axes;
}
// Defualt get_pointer returns the position (0,0,0) out of range
Point ObjectType::get_pointer() {
  Point P;
  P.init(0, 0, 0);
  return P;
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

// MaxDimensionalDistance find the dimensions along the local axis
// dimension: int representing axis (0 --> x axis, 1 --> y axis, 2 --> z axis)
// return  float giving the largest distance from the center along that axis
float ObjectType::MaxDimensionalDistance(int dimension) {
  float max = 0;
  Point axis;
  /*find what axis we are looking at*/
  if(dimension == 0) axis.init(1, 0, 0);
  else if(dimension == 1) axis.init(0, 1, 0);
  else axis.init(0, 0, 1);
  /*Rotate that axis to find the local axis*/
  //axis = QRotate(axis, Qinv(angle));
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
void ObjectType::GetScale(int i = 0);
  float buffer = 1.10;
  dim.clear();
  dim.push_back(2 * MaxDimensionalDistance(0) * buffer);
  dim.push_back(2 * MaxDimensionalDistance(1) * buffer);
  dim.push_back(2 * MaxDimensionalDistance(2) * buffer);
}
// AddPoints adds new_points to the object
// new_points is a list of Points that are to be added to the Object
// The default is to also reset the AngleAxes to this new set of points
void AddPoints(vector<Point> new_points) {
  vector<Point>::iterator i;
  points.insert(points.end(), new_points.begin(), new_points.end());
  GetFirstAngleAxis(1);
  GetSecondAngleAxis(1);
}
// GetCenter is finds the center of the object
// The standard is just to find the average
virtual void GetCenter(int i = 0) {
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
void GetAngle(int i = 0) {
  GetFirstAngleAxis(i);
  GetSecondAngleAxis(i);
  // If the Axis is unpopulated, it was unable to find two points
  vector<float> default_Q;
  default_Q.push_back(1);
  default_Q.push_back(0);
  default_Q.push_back(0);
  default_Q.push_back(0);
  if (AngleAxis1.size() != 2) {
    angle = default_Q;
    return;
  }
  if(OriginalAxis1.magnitude() == 0 || vAngleAxis1.magnitude() == 0) {
    angle = default_Q;
    return;
  }
  vector<float> Q1;
  // Point cross1 = OriginalAxis1.cross(vAngleAxis1);
  Point cross1 = vAngleAxis1.cross(OriginalAxis1).times(.5);
  Q1.push_back(sqrt(pow(vAngleAxis1.magnitude(), 2) * pow(OriginalAxis1.magnitude(), 2)) + vAngleAxis1.dot(OriginalAxis1));
  Q1.push_back(cross1.z);
  Q1.push_back(cross1.y);
  Q1.push_back(cross1.x);
  angle = Qnormalize(Q1);
  // If the Second Axis is unpopulated it was unable to find two points
  if (AngleAxis2.size() != 2) {
    return;
  }
  if(OriginalAxis2.magnitude() == 0 || vAngleAxis2.magnitude() == 0) {
    return;
  }
  vector<float> Q2;
  Point temp = QRotate(vAngleAxis2, angle);
  Point cross2 = temp.cross(OriginalAxis2).times(.5);
  Q2.push_back(sqrt(pow(temp.magnitude(), 2) * pow(OriginalAxis2.magnitude(), 2)) + temp.dot(OriginalAxis2));
  Q2.push_back(cross2.z);
  Q2.push_back(cross2.y);
  Q2.push_back(cross2.x);
  // angle = Q;
  printf("final\n");
  angle = Qnormalize(Qmult(Q1, Q2));
  return;
}
void PrintPoints() {
  vector<Point>::iterator i;
  for (i = points.begin(); i != points.end(); ++i) {
    printf("%s\n",i->print().c_str());
  }
}

