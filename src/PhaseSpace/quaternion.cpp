// File: Quaternion.cpp
// Author: Dylan Visher
// Date: 5/18/13
// About: Logic for Quaternions

// Note: Notation for quaternions q1 can be represented as (w1, v1)
//   w1 is the scalar part of the quaternion
//   v1 is the vector part of the quaternion
// x is the cross product
// o is the dot product
// * is scalar multiplication

#include <vector>
#include <cstdio>
#include "./quaternion.h"

namespace quaternions {

// Qmult multiplies two quaternions together
// The resultant quaternion is defined as
//   w: (w1 * w2) + (v1 o v2)
//   v: (v2 * w1) + (v1 * w2) + (v1 x v2)
// This result q = (w, v) is returned
vector<float> Qmult(vector<float> q1, vector<float> q2) {
  Point v1; 
  v1.init(q1[1], q1[2], q1[3]);
  Point v2; 
  v2.init(q2[1], q2[2], q2[3]);
  vector<float> result;
  result.push_back(q1[0]*q2[0] - v1.dot(v2));
  Point vec;
  vec = v2.times(q1[0]).add(v1.times(q2[0])).add(v1.cross(v2));
  result.push_back(vec.x);
  result.push_back(vec.y);
  result.push_back(vec.z);
  return result;
}

// Qconj finds the conjugate of the qiven quaternion
// It can be defined as (-w, v) or (w, -v)
// tither one of these represent the same quaternion
vector<float> Qconj(vector<float> q) {
  vector<float> conj;
  conj.push_back(q[0]);
  conj.push_back(-1*q[1]);
  conj.push_back(-1*q[2]);
  conj.push_back(-1*q[3]);
  return conj;
}

// Qnorm finds the magnitude of the quaternion in the standard way
//   sqrt(q o q)
// This returns the length or magnitude of the vector (a quaternion)
float Qnorm(vector<float> q) {
  return sqrt(pow(q[0], 2) + pow(q[1], 2) + pow(q[2], 2) + pow(q[3], 2));
}

// Qnormalize returns the unit vector
//  It divides each of the components of q by its magnitude
vector<float> Qnormalize(vector<float> q) {
  vector<float>::iterator iter;
  float mag = Qnorm(q);
  if(q.size() != 4) {
    printf("Invalid quaternion\n");
  }
  if (fabs(mag) < .0000001) return q;
  for (iter = q.begin(); iter != q.end(); ++iter) {
    *iter = *iter / mag;
  } return q;
}

// Qinv finds the inverse of a quaternion
// This represents the opposite rotation of the given quaternion
//   It is made by dividing each of the components of the conjugate
//   By the squared magnitude
//   conjugate(q) / |q|^2
vector<float> Qinv(vector<float> q) {
  float divisor = pow(Qnorm(q), 2);
  if (divisor == 0) {
    vector<float> failed;
    return failed;
  }
  vector<float> inverse = Qconj(q);
  inverse[0] /= divisor;
  inverse[1] /= divisor;
  inverse[2] /= divisor;
  inverse[3] /= divisor;
  return inverse;
}

// QRotate rotates the given vector by the quaternion
// Q * v * Q^-1
// Return the new vector
Point QRotate(Point v, vector<float> q) {
  if (q.size() != 4) return v;
  if (Qnorm(q) == 0) return v;
  q = Qnormalize(q);
  Point u;
  u.init(q[1], q[2], q[3]);
  float s = q[0];
  Point result;
  result = u.times(2.0f * u.dot(v));
  result = result.add(v.times(s*s - u.dot(u)));
  result = result.add(u.cross(v).times(s * 2.0f));
  return result;
}

}  // namespace quaternions
