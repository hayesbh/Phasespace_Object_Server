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
using std::abs;
using std::pow;
using points::PointsToPlane;
using quaternions::Qmult;
using quaternions::Qconj;
using quaternions::Qnorm;
using quaternions::Qnormalize;
using quaternions::Qinv;
using points::FindByID;

/*This class is a wrapper for object type specific information*/
class ObjectType {
  private:
    /* Location Information */
    vector<Point> points;
    Point center;
    /* Angle Information */
    Point OriginalAxis;
    vector<int> AngleAxis;
    Point vAngleAxis;
    /* Quaternion Rotation Information */
    vector<float> angle;
    /* x, y, z, scale for global representation of dimension */
    float x_scale;
    float y_scale;
    float z_scale;

  public:
  /**
   * [init initializes the object with the given points]
   * @param p [points for the object]
   */
  void init(vector<Point> p) {
    points = p;
    /* Get the Center and the Axis for Angle information */
    GetCenter();
    GetAngleAxis();
    /* Get Angle information */
    GetAngle();
    /* Get Dimensional information */
    GetScale();
  }
  /**
   * [get_dimensions returns the global dimension information]
   * @param dim[3] [A vector that stores the dimensional information]
   */
  void get_dimensions(float dim[3]) {
    dim[0] = x_scale;
    dim[1] = y_scale;
    dim[2] = z_scale;
  }
  /**
   * [get_points return the points associated with the object]
   * @return [these points]
   */
  vector<Point> get_points() {
    return points;
  }
  /**
   * [get_center return the stored information about the center]
   * @return [Point describing center of object]
   */
  Point get_center() {
    return center;
  }
  /**
   * [get_angle return the stored rotation information]
   * @return [a quaternion representing the angle]
   */
  vector<float> get_angle() {
    return angle;
  }

  /**
   * [Update updates the objects points with new marker information]
   * @param markers [PhaseSpace markers with new info]
   * @param n       [number of markers in the markers array]
   */
  void Update(OWLMarker *markers, int n) {
    /* Update markers information */
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
    /* Update Information about the center */
    GetCenter();
    /* Update Information about the Angle Axis */
    GetAngleAxis();
    /* Find the New Angle */
    GetAngle();
    /* Get the Scaling information */
    GetScale();
    return;
  }
  Point MaxPoint(int dimension) {
    Point max = center;
    vector<Point>::iterator iter;
    for(iter = points.begin(); iter != points.end(); ++iter) {
      if(!(iter->current)) continue;
      if (dimension == 0) {
        if(iter->x > max.x) {
          max = *iter;
        }
      } else if (dimension == 1) {
        if(iter->y > max.y) {
          max = *iter;
        }
      } else {
        if(iter->z > max.z) {
          max = *iter;
        }
      }
    } return max;
  }
  Point MinPoint(int dimension) {
    Point min = center;
    vector<Point>::iterator iter;
    for(iter = points.begin(); iter != points.end(); ++iter) {
      if(!(iter->current)) continue;
      if (dimension == 0) {
        if(iter->x < min.x) {
          min = *iter;
        }
      } else if (dimension == 1) {
        if(iter->y < min.y) {
          min = *iter;
        }
      } else {
        if(iter->z < min.z) {
          min = *iter;
        }
      }
    } return min;
  }
  /**
   * [GetScale finds the global x, y, and z scale of the object]
   */
  void GetScale() {
    float buffer = 1.20;
    x_scale = MaxPoint(0).sub(MinPoint(0)).x * buffer;
    y_scale = MaxPoint(1).sub(MinPoint(1)).y * buffer;
    z_scale = MaxPoint(2).sub(MinPoint(2)).z * buffer;
  }
  /**
   * [AddPoints adds new_points to the Object]
   * @param new_points [a list of Points]
   */
  void AddPoints(vector<Point> new_points) {
    vector<Point>::iterator i;
    points.insert(points.begin(), new_points.begin(), new_points.end());
  }
  /**
   * [getCenter Default algorithm for getting the center]
   * @return [A point describing the center]
   */
  void GetCenter() {
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
  /**
   * [GetAngleAxis sets or finds the vector of the Axis that defines angle]
   * @return [this vector axis]
   */
  Point GetAngleAxis(){
    if (AngleAxis.size() == 2) {
      vector<Point>::iterator it1 = FindByID(AngleAxis[0], points);
      vector<Point>::iterator it2 = FindByID(AngleAxis[1], points);
      if(it1->current == 0 || it2->current == 0) {
        return vAngleAxis;
      }
      vAngleAxis = it2->sub(*it1);
      return vAngleAxis;
    } else {
      /* If there are not enough points then just return the 0 vector */ 
      if (points.size() < 2) {
        vAngleAxis.init();
        return vAngleAxis;
      }
      /* Max distance Squared to compare with */
      float maxdist2 = 0;
      /* P1 and P2 are the Points that define the vector */
      Point P1;
      Point P2;
      /*Iterators for going throught the points that define the object*/
      vector<Point>::iterator it1;
      vector<Point>::iterator it2;
      for (it1 = points.begin(); it1 != points.end(); ++it1) {
      if(!(it1->current)) continue;
        for (it2 = points.begin()+1; it2 != points.end(); ++it2) {
          if(!(it2->current)) continue;
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
      /* If we are looking for the Axis that defines the angle
         Make sure that the id's are set so that it can be consistent */
      AngleAxis.push_back(P1.id);
      AngleAxis.push_back(P2.id);
      OriginalAxis = P2.sub(P1);
      vAngleAxis = OriginalAxis;
      return vAngleAxis;
    }
  }
  /**
   * [GetAngle gets the Euler Angles of the Object]
   * @return [the Euler Angles]
   */
  /*http://www.vbforums.com/showthread.php?
  584390-Quaternion-from-two-3D-Position-Vectors*/
  void GetAngle() {
    /*If the Axis is unpopulated, it was unable to find two points*/
    vector<float> default_Q;
    default_Q.push_back(1);
    default_Q.push_back(0);
    default_Q.push_back(0);
    default_Q.push_back(0);
    if (AngleAxis.size() != 2) {
      angle = default_Q;
      return;
    }
    if(OriginalAxis.magnitude() == 0 || vAngleAxis.magnitude() == 0) {
      angle = default_Q;
      return;
    }
    vector<float> Q;
    Point cross = vAngleAxis.cross(OriginalAxis);
    Q.push_back(sqrt(pow(vAngleAxis.magnitude(), 2) * pow(OriginalAxis.magnitude(), 2)) + vAngleAxis.dot(OriginalAxis));
    Q.push_back(cross.x);
    Q.push_back(cross.y);
    Q.push_back(cross.z);
    angle = Q;
    // angle = Qnormalize(Q);
    return;
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
