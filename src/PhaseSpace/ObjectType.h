/**
 * File: ObjectType.h
 * Author: Dylan Visher
 * Date: 5/18/13
 * About: ObjectType contains type specific information
 */
#ifndef _SHL_COREOBJECTSERVER_OBJECTTYPE_H
#define _SHL_COREOBJECTSERVER_OBJECTTYPE_H

#include <vector>
#include <math.h>
#include "./Point.h"
#include "./quaternion.h"

namespace object_server {

const float PI = 3.14159265358979f;

using std::vector;
using object_server::Point;
using std::acos;
using points::PointsToPlane;
using quaternions::Qmult;
using quaternions::Qconj;
using quaternions::Qnorm;
using quaternions::Qinv;
using points::FindByID;

/*This class is a wrapper for object type specific information*/
class ObjectType {
  private:
    vector<Point> points;
    vector<int> Axis;
    vector<int> ShortAxis;
    float width;
    float length;
    float height;

  public:
  /**
   * [init initializes the object with the given points]
   * @param p [points for the object]
   */
  void init(vector<Point> p) {
    points = p;
    PrintPoints();
    GetCenter();
    length = GetAxis().magnitude();
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
    vector<Point>::iterator i;
    /*for (i = new_points.begin(); i != new_points.end(); ++i) {
      printf("%s\n",i->print().c_str());
    }*/
    points.insert(points.begin(), new_points.begin(), new_points.end());
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
      vector<Point>::iterator it1 = FindByID(Axis[0], points);
      vector<Point>::iterator it2 = FindByID(Axis[1], points);
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
     /* Set Length of the Object */
     length = P2.sub(P1).magnitude();
     return P2.sub(P1);
  }
  Point GetShortAxis() {
    Point P;
    if (Axis.size () != 2 || points.size() < 3) {
      P.init(1000000, 1000000, 1000000);
      width = 0;
      return P;
    } else {
      Point P = points[0];
      vector<Point>::iterator it1 = FindByID(Axis[0], points);
      vector<Point>::iterator it2 = FindByID(Axis[1], points);
      vector<Point> Ax;
      Ax.push_back(*it1);
      Ax.push_back(*it2);
      float maxdist = P.DistanceToLine(Ax);
      vector<Point>::iterator it;
      for (it = points.begin()+1; it != points.end(); ++it) {
          float dist = it->DistanceToLine(Ax);
          if (dist > maxdist) {
            maxdist = dist;
            P = *it;
          }
        }
        width = maxdist;
        ShortAxis.push_back(P.id);
        return P;
    }
  }
  void GetHeight() {
    if ( (Axis.size () != 2 && ShortAxis.size() != 1 )|| points.size() < 4) {
      height = 0;
    } else {
      float plane[4] = {0, 0, 0, 0};
      vector<Point>::iterator p1 = FindByID(Axis[0], points);
      vector<Point>::iterator p2 = FindByID(Axis[1], points);
      vector<Point>::iterator p3 = FindByID(ShortAxis[0], points);
      PointsToPlane(*p1, *p2, *p3, plane);
      float maxdist = 0;
      vector<Point>::iterator it;
      for (it = points.begin()+1; it != points.end(); ++it) {
        if(it->id != Axis[0] && it->id != Axis[1] && it->id != ShortAxis[0]) {
          float dist = it->DistanceToPlane(plane);
          if(dist > maxdist) {
            maxdist = dist;
          }
        }
      }
      height = maxdist;
    }
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
    Point u = v.cross(Z).normalize();
    float alpha = acos(v.z * (1 / v.magnitude()));
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
  void PrintPoints() {
    vector<Point>::iterator i;
    for (i = points.begin(); i != points.end(); ++i) {
      printf("%s\n",i->print().c_str());
    }
  }
};

}  // namespace object_server
#endif
