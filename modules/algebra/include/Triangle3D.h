/**
 *  \file  Triangle3D.h
 *  \brief simple implementation of a triangle in 3D
 *
 *  Copyright 2007-2011 IMP Inventors. All rights reserved.
 */
#ifndef IMPALGEBRA_TRIANGLE_3D_H
#define IMPALGEBRA_TRIANGLE_3D_H

#include "Vector3D.h"
#include "Transformation3D.h"
#include "ReferenceFrame3D.h"
#include "algebra_macros.h"
#include <iostream>
#include <IMP/constants.h>


IMPALGEBRA_BEGIN_NAMESPACE
/**
    \geometry
 */
class IMPALGEBRAEXPORT Triangle3D
{
 public:
  Triangle3D(){}
  //! The passed points must not be colinear
  Triangle3D(const VectorD<3> &p1,const VectorD<3> &p2,const VectorD<3> &p3);
  //! Get the start=0/end=1 point of the segment
  const VectorD<3>& get_point(unsigned int i) const {
    IMP_USAGE_CHECK(i<3,"invalid point index");
    return p_[i];
  }
  IMP_SHOWABLE_INLINE(Triangle3D, out << "(" << p_[0] << ", "
                      << p_[1] << ", " << p_[2] << ")");
  Floats get_edge_lengths() const;
 private:
  VectorD<3> p_[3];
};

IMP_VALUES(Triangle3D, Triangle3Ds);

//! Return the largest triangle defined by 3 points from the input
/** \relatesalso Triangle3D */
IMPALGEBRAEXPORT Triangle3D get_largest_triangle(const Vector3Ds &points);

//! Return a transformation between two triangles
IMPALGEBRAEXPORT Transformation3D
   get_transformation_from_first_triangle_to_second(
                      Triangle3D first_tri,Triangle3D second_tri);
//! Return true if the three points are co-linear
IMPALGEBRAEXPORT bool
get_are_colinear(
      const VectorD<3> &p1,const VectorD<3> &p2,const VectorD<3> &p3);

IMPALGEBRA_END_NAMESPACE

#endif /* IMPALGEBRA_TRIANGLE_3D_H */
