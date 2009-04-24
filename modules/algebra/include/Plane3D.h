/**
 *  \file Plane3D.h   \brief Simple 3D plane class.
 *
 *  Copyright 2007-9 Sali Lab. All rights reserved.
 *
 */
#ifndef IMPALGEBRA_PLANE_3D_H
#define IMPALGEBRA_PLANE_3D_H

#include "Vector3D.h"

IMPALGEBRA_BEGIN_NAMESPACE

/** Represent a plane using the distance and a normal.
*/
class IMPALGEBRAEXPORT Plane3D: public UninitializedDefault {
public:
  Plane3D(){}
  Plane3D(const Vector3D& point_on_plane,const Vector3D &normal_to_plane);
  Plane3D(double distance_to_plane ,const Vector3D &normal_to_plane);
  Vector3D get_point_on_plane() const {return normal_*distance_;}
  const Vector3D &get_normal() const {return normal_;}
  //! Project the point onto the plane
  Vector3D get_projection(const Vector3D &p) const;
  /** @name Orientation
       Up is the direction of the normal.
       @{
  */
  /** \cgalpredicate
   */
  bool get_is_above(const Vector3D &p) const;
  /** \cgalpredicate
   */
  bool get_is_below(const Vector3D &p) const;
  /** @} */
  void show(std::ostream &out=std::cout) const;

  //! Return the plane with the opposite normal
  Plane3D get_opposite() const {
    return Plane3D(-distance_, -normal_);
  }
private:
  double distance_;
  Vector3D normal_; //normal to plane
};


//! Return the distance between a plane and a point in 3D
/** \relates Plane3D */
inline double distance(const Plane3D& pln, const Vector3D &p) {
  return (pln.get_projection(p)-p).get_magnitude();
}


IMP_OUTPUT_OPERATOR(Plane3D);

IMPALGEBRA_END_NAMESPACE

#endif /* IMPALGEBRA_PLANE_3D_H */
