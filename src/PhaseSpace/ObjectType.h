/**
 * File: ObjectType.h
 * Author: Dylan Visher
 * Date: 5/18/13
 * About: ObjectType contains type specific information
 */
#ifndef _SHL_COREOBJECTSERVER_OBJECTTYPE_H
#define _SHL_COREOBJECTSERVER_OBJECTTYPE_H

#include <vector>
#include "./Point.h"

namespace object_server {

const float PI = 3.14159265358979f;

using std::vector;
using object_server::Point;

/**
 * [FindById finds the point with the id given]
 * @param  id     [id of the point desired]
 * @param  points [vector of points to look in]
 * @return        [the index of the point with this id]
 */
vector<Point>::iterator FindById(int id, vector<Point> points) {
  vector<Point>::iterator iter;
  for (iter = points.begin(); iter != points.end(); ++iter) {
    if (iter->id == id)
      return iter;
  }
  return points.end();
}
vector<float> Qmult(vector<float> v1, vector<float> v2) {
  Point p1; p1.init(v1[1], v1[2], v1[3]);
  Point p2; p2.init(v2[1], v2[2], v2[3]);
  vector<float> result;
  result.push_back(v1[0]*v2[0] - p1.dot(p2));
  Point vec;
  vec = p2.times(v1[0]).add(p1.times(v2[0])).add(p1.cross(p2));
  result.push_back(vec.x);
  result.push_back(vec.y);
  result.push_back(vec.z);
  return result;
}
vector<float> Qconj(vector<float> v) {
  vector<float> conj;
  conj.push_back(v[0]);
  conj.push_back(-1*v[1]);
  conj.push_back(-1*v[2]);
  conj.push_back(-1*v[3]);
  return conj;
}
float Qnorm(vector<float> v) {
  return sqrt(pow(v[0], 2) + pow(v[1], 2) + pow(v[2], 2) + pow(v[3], 2));
}
vector<float> Qinv(vector<float> v) {
  float divisor = pow(Qnorm(v), 2);
  vector<float> inverse = Qconj(v);
  inverse[0] /= divisor;
  inverse[1] /= divisor;
  inverse[2] /= divisor;
  inverse[3] /= divisor;
  return inverse;
}
/*This class is a wrapper for object type specific information*/
class ObjectType {
  private:
    vector<Point> points;
    vector<int> Axis;

  public:
  /**
   * [init initializes the object with the given points]
   * @param p [points for the object]
   */
  void init(vector<Point> p) {
    points = p;
    GetCenter();
    GetAxis();
    GetAngle();
  }
  /**
   * [update updates the objects points with new marker information]
   * @param markers [PhaseSpace markers with new info]
   * @param n       [number of markers in the markers array]
   */
  void Update(OWLMarker *markers, int n) {
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
    return;
  }
  /**
   * [get_points return the points associated with the object]
   * @return [these points]
   */
  vector<Point> get_points() {
    return points;
  }
  /**
   * [AddPoints adds new_points to the Object]
   * @param new_points [a list of Points]
   */
  void AddPoints(vector<Point> new_points) {
    points.insert(points.end(), new_points.begin(), new_points.end());
  }
  /**
   * [getCenter Default algorithm for getting the center]
   * @return [A point describing the center]
   */
  Point GetCenter() {
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
        return p;
     } else {
        return points[0];
     }
  }
  /**
   * [GetAxis sets the two Axis point ids]
   * @return [returns the vector between these points]
   */
  Point GetAxis() {
     /*If Axis has already been set*/
     if (Axis.size() == 2) {
      vector<Point>::iterator it1 = FindById(Axis[0], points);
      vector<Point>::iterator it2 = FindById(Axis[1], points);
      return (*it2).sub(*it1);
     }
     float maxdist2 = 0;
     Point P1;
     Point P2;
     if (points.size() < 2) {
       P1.init();
       return P1;
     }
     vector<Point>::iterator it1;
     vector<Point>::iterator it2;
     for (it1 = points.begin(); it1 != points.end(); ++it1) {
       for (it2 = points.begin()+1; it2 != points.end(); ++it2) {
         float dist2 = pow(it1->x - it2->x, 2)
                     + pow(it1->y - it2->y, 2)
                     + pow(it1->z - it2->z, 2);
         if (dist2 > maxdist2) {
           maxdist2 = dist2;
           P1 = *it1;
           P2 = *it2;
         }
       }
     }
     Axis.push_back(P1.id);
     Axis.push_back(P2.id);
     return P2.sub(P1);
  }
  /**
   * [GetAngle gets the Euler Angles of the Object]
   * @return [the Euler Angles]
   */
  /*http://www.vbforums.com/showthread.php?
  584390-Quaternion-from-two-3D-Position-Vectors*/
  vector<float> GetAngle() {
    Point v = GetAxis();
    /*If the Axis is unpopulated, it was unable to find two points*/
    if (Axis.size() != 2) {
      vector<float> P(3, 0);
      P.push_back(1);
      return P;
    }
    Point Z;
    Z.init(0, 0, 1);
    /*Find axis of rotation as unit vector*/
    Point axis = v.cross(Z);
    Point u = axis.times(v.magnitude());
    float alpha = v.z/(v.magnitude());
    /*the quaternion*/
    vector<float> Q;
    float plus = cos(alpha/2);
    float mult = sin(alpha/2);
    /*set w*/
    Q.push_back(plus);
    /*set x*/
    Q.push_back(plus + u.x*mult);
    /*set y*/
    Q.push_back(plus + u.y*mult);
    /*set z*/
    Q.push_back(plus + u.z*mult);
    return Q;
  }
};

}  // namespace object_server
#endif
