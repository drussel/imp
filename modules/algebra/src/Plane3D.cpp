/**
 *  \file Plane3D.cpp   \brief Simple 3D plane class.
 *
 *  Copyright 2007-2010 Sali Lab. All rights reserved.
 */
#include <IMP/algebra/Plane3D.h>
#include <IMP/algebra/internal/cgal_predicates.h>

IMPALGEBRA_BEGIN_NAMESPACE
Plane3D::Plane3D(const Vector3D& point_on_plane,
                 const Vector3D &normal_to_plane) {
  normal_ = normal_to_plane.get_unit_vector();
  distance_= normal_*point_on_plane;
}
Plane3D::Plane3D(double distance,
                 const Vector3D &normal):
  distance_(distance),
  normal_(normal){
  IMP_USAGE_CHECK(std::abs(normal.get_squared_magnitude()-1) < .05,
            "The normal vector must be normalized",
            ValueException);
  }

Vector3D Plane3D::get_projection(const Vector3D &p) const {
  return p-normal_*(normal_*p-distance_);
}
bool Plane3D::get_is_above(const Vector3D &p) const {
#ifdef IMP_USE_CGAL
  return internal::cgal_plane_compare_above(*this, p) > 0;
#else
  return normal_*p > distance_;
#endif
}
bool Plane3D::get_is_below(const Vector3D &p) const {
#ifdef IMP_USE_CGAL
  return internal::cgal_plane_compare_above(*this, p) < 0;
#else
  return normal_*p < distance_;
#endif
}

void Plane3D::show(std::ostream &out) const {
    out << "(" << distance_ << ": " << spaces_io(normal_)
        << ")";
}

IMPALGEBRA_END_NAMESPACE
