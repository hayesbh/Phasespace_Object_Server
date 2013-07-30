// File: Object.cpp
// Author: Dylan Visher
// Date: 7/18/13
// About: General Object Class

#include "./Object.h"

namespace object_server {

//Default CollidesWith(Object obj)

// IntersectsObject asks whether this object intersects another given object
//   This collision detection employs the Seperation of Oriented Bouunding Boxes
//     to determine if two objects are in fact intersecting
//
//   Technique: 
//     1. Find Axes and normals between them
//        These define all possible directions the objects could be seperated
//     2. Project the Object onto these lines (like shadows of the boxes)
//     3. Determine if these objects projections are in fact overlapping
//     4. If all shadows are overlapping then the objects are colliding
//        If there is even one shadow not overlapping then the objects are non intersecting
//
//   Source: "Dynamic Collision Detection using Oriented Bounding Boxes"
//   Source Author: David Eberly
//   Section: 2 Oriented Bounding boxes
// 
// obj: The Object that this one might intersect this one
// return bool: Does this object intersect obj?
bool Object::CollidesWith(Object* obj) {
  // Unit Axes of First Object Stored in A
  Point A[3] = { axes_[0], axes_[1], axes_[0].Cross(axes_[1]).Normalize() };
  // Unit Axes of the Second Object Stored in B
  vector<Point> ax = obj->get_axes();
  Point B[3] = { ax[0],
                 ax[1],
                 ax[2] };
  // D is the vector from First Object to Second Object
  Point D = center_.Sub(obj->get_center());
  // The extents of object 1 : 
  // How far does it extend from the center to the edges in the local x, y, and z axis
  // Half the total width, length, and height of the object
  float a[3] = { dim_[0]/2, dim_[1]/2, dim_[2]/2 };
  // The extents for object 2 :
  vector<float> dim2 = obj->get_dimensions();
  float b[3] = { dim2[0]/2, dim2[1]/2, dim2[2]/2 };
  // Matrix c is a 3x3 Matrix of Dot Procducts s.t. c[i][j] = A[i] Dot B[j]
  // These are set as the if statements are evaluated to prevent unneeded computation
  float c[3][3] = {{0,0,0},{0,0,0},{0,0,0}};
  // The following equations are taken from Chapter 2 Table 1 Eberly
  //   Is the sum of radii of the two Objects' projections greater than the distance between them
  //   If this is the case then the two shadows must be intersecting
  //   This is the nonintersection test for two Oriented Bounding Boxes
  if ((a[0] +
       b[0]*(c[0][0] = fabs(A[0].Dot(B[0]))) + b[1]*(c[0][1] = fabs(A[0].Dot(B[1]))) + b[2]*(c[0][2] = fabs(A[0].Dot(B[2]))))
       < fabs(A[0].Dot(D)))
       return false;
  if ((a[1] +
       b[0]*(c[1][0] = fabs(A[1].Dot(B[0]))) + b[1]*(c[1][1] = fabs(A[1].Dot(B[1]))) + b[2]*(c[1][2] = fabs(A[1].Dot(B[2]))))
       < fabs(A[1].Dot(D)))
       return false;
  if ((a[2] +
       b[0]*(c[2][0] = fabs(A[2].Dot(B[0]))) + b[1]*(c[2][1] = fabs(A[2].Dot(B[1]))) + b[2]*(c[2][2] = fabs(A[2].Dot(B[2]))))
       < fabs(A[2].Dot(D)))
      return false;
  if ((a[0] * c[0][0] + a[1] * c[1][0] + a[2] * c[2][0]) +
       b[0]
       < fabs(B[0].Dot(D)))
      return false;
  if ((a[0] * c[0][1] + a[1] * c[1][1] + a[2] * c[2][1]) +
       b[1]
       < fabs(B[1].Dot(D)))
      return false;
  if ((a[0] * c[0][2] + a[1] * c[1][2] + a[2] * c[2][2]) +
       b[2]
       < fabs(B[2].Dot(D)))
      return false;
  if ((a[1] * c[2][0] + a[2] * c[1][0]) +
      (b[1] * c[0][2] + b[2] * c[0][1])
       < fabs(c[1][0] * A[2].Dot(D) - c[2][0] * A[1].Dot(D)))
      return false;
  if ((a[1] * c[2][1] + a[2] * c[1][1]) +
      (b[0] * c[0][2] + b[2] * c[0][0])
       < fabs(c[1][1] * A[2].Dot(D) - c[2][1] * A[1].Dot(D))) 
      return false;
  if ((a[1] * c[2][2] + a[2] * c[1][2]) +
      (b[0] * c[0][1] + b[1] * c[0][0])
       < fabs(c[1][2] * A[2].Dot(D) - c[2][2] * A[1].Dot(D)))
      return false;

  if ((a[0] * c[2][0] + a[2] * c[0][0]) +
      (b[1] * c[1][2] + b[2] * c[1][1])
       < fabs(c[2][0] * A[0].Dot(D) - c[0][0] * A[2].Dot(D)))
      return false;
  if ((a[0] * c[2][1] + a[2] * c[0][1]) +
      (b[0] * c[1][2] + b[2] * c[1][0])
     < fabs(c[2][1] * A[0].Dot(D) - c[0][1] * A[2].Dot(D)))
      return false;
  if ((a[0] * c[2][2] + a[2] * c[0][2]) +
      (b[0] * c[1][1] + b[1] * c[1][0])
     < fabs(c[2][2] * A[0].Dot(D) - c[0][2] * A[2].Dot(D)))
      return false;

  if ((a[0] * c[1][0] + a[1] * c[0][0]) +
      (b[1] * c[2][2] + b[2] * c[2][1])
     < fabs(c[0][0] * A[1].Dot(D) - c[1][0] * A[0].Dot(D))) 
      return false;
  if ((a[0] * c[1][1] + a[1] * c[0][1]) +
      (b[0] * c[2][2] + b[2] * c[2][0])
      < fabs(c[0][1] * A[1].Dot(D) - c[1][1] * A[0].Dot(D)))
      return false;
  if ((a[0] * c[1][2] + a[1] * c[0][2]) +
      (b[0] * c[2][1] + b[1] * c[2][0])
      < fabs(c[0][2] * A[1].Dot(D) - c[1][2] * A[0].Dot(D)))
      return false;
  return true;
}

const vector<Object*>::iterator FindObjectByName(string name, vector<Object*> &objects) {
  vector<Object*>::iterator iter;
  for (iter = objects.begin(); iter != objects.end(); ++iter) {
    if ((*iter)->get_name() == name) return iter;
  }
  return objects.end();
}

}  // namespace object_server
