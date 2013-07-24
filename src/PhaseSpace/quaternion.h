// File: quaternion.h
// Author: Dylan Visher
// Date: 5/18/13
// About: Quaternions

#ifndef _SHL__OBJECT_SERVER_SRC_PHASESPACE_QUATERNION_H_
#define _SHL__OBJECT_SERVER_SRC_PHASESPACE_QUATERNION_H_


// Note: Notation for quaternions q1 can be represented as (w1, v1)
//   w1 is the scalar part of the quaternion
//   v1 is the vector part of the quaternion
// x is the cross product
// o is the dot product
// * is scalar multiplication


#include <vector>
#include "./Point.h"

// Quaternions are 4 dimensional vectors which employ the std::vector class
namespace quaternions {

using std::vector;
using object_server::Point;

// Qmult uses Quaternion multiplication to multiply two quaternions together
// v1: first quaternion to multiply
// v2: second quaternion to multiply
// return: the computed product
vector<float> Qmult(vector<float> v1, vector<float> v2);

// Qconj finds the Conjugate of the given quaternion
// v: the quaternion to compute the conjugate of
// return: the computed conjugate
vector<float> Qconj(vector<float> v);

// Qnorm finds the magnitude of the the quaternion
// v: the quaternion to find the magnitude of
// return: the magnitude of the vector 
float Qnorm(vector<float> v);

// Qnormalize finds the normalized unit-quaternion
// v: the quaternion to find the unit-quaternion of
// return: the unit quaternion
vector<float> Qnormalize(vector<float> v);

// Qinv finds the inverse of the quaternion
// v: the quaternion to find the inverse of
// return : the inverse quaternion
vector<float> Qinv(vector<float> v);

// QRotate rotates the given Point (which could represent a vector)
//   by the angle represented by the quaternion
// p: The Point or vector (mathematic) to rotate
// q: the quaternion angle to rotate the Point by
// return: the new rotated vector or point
Point QRotate(Point p, vector<float> q);

}  // namespace quaternions

#endif  // _SHL__OBJECT_SERVER_SRC_PHASESPACE_QUATERNION_H_
