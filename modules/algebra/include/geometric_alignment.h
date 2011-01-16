/**
 *  \file geometric_alignment.h
 *  \brief align sets of points.
 *
 *  Copyright 2007-2011 IMP Inventors. All rights reserved.
 */

#ifndef IMPALGEBRA_GEOMETRIC_ALIGNMENT_H
#define IMPALGEBRA_GEOMETRIC_ALIGNMENT_H

#include "algebra_config.h"

#include "Vector3D.h"
#include "Rotation3D.h"
#include "Transformation3D.h"
#include "Transformation2D.h"
#include "internal/tnt_array2d.h"
#include "internal/jama_svd.h"
#include <IMP/log.h>


IMPALGEBRA_BEGIN_NAMESPACE


//! Compute the rigid transform bringing the first point set to the second
/** The points are assumed to be corresponding (that is, from[0] is aligned
    to to[0] etc.). The alignment computed is that which minimized the
    sum of squared distances between corresponding points. Return the
    \f[ \operatornamewithlimits{argmin}_T
    \sum \left|T\left(f\left[i\right]\right)-t[i]\right|^2 \f]

    If the point sets lie in a 1 or 2 dimensional subspace,
    the alignment algorithm
    is unstable and not guaranteed to work. A warning is printed in this
    case.

    \genericgeometry

    \relatesalso Transformation3D
    \see VectorD<3>
 */
template <class Vector3DsOrXYZs0,
          class Vector3DsOrXYZs1>
inline IMP::algebra::Transformation3D
get_transformation_aligning_first_to_second(
IMP_RESTRICT const Vector3DsOrXYZs0 &from,
IMP_RESTRICT const Vector3DsOrXYZs1 &to) {
  IMP_INTERNAL_CHECK(from.size() == to.size(), "sizes don't match");
  IMP_INTERNAL_CHECK(from.size() >0, "Points are needed");
  // compute the centroid of the points and transform
  // pointsets so that their centroids coinside

  VectorD<3> center_from(0,0,0), center_to(0,0,0);
  for (unsigned int i=0; i< from.size(); ++i) {
    //double x= p_it->x();
    center_from+= get_vector_d_geometry(from[i]);
    center_to += get_vector_d_geometry(to[i]);
    IMP_LOG(VERBOSE, i << ": (" << get_vector_d_geometry(from[i])
            << ") (" << get_vector_d_geometry(to[i]) << ")\n");
  }
  center_from = center_from/from.size();
  center_to = center_to/to.size();

  IMP_LOG(VERBOSE, "Centers are (" << center_from << ") (" << center_to
          << ")\n");
  std::vector<VectorD<3> > shifted_from(from.size()), shifted_to(to.size());
  for (unsigned int i=0; i< from.size(); ++i) {
    shifted_from[i]=get_vector_d_geometry(from[i])-center_from;
    shifted_to[i]= get_vector_d_geometry(to[i])-center_to;
  }

  // covariance matrix
  internal::TNT::Array2D<double> H(3, 3);
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      H[i][j] = 0;
    }
  }
  for (unsigned int i = 0; i < from.size(); i++) {
    for (int j = 0; j < 3; j++) {
      for (int k = 0; k < 3; k++) {
        H[j][k] += shifted_from[i][j]*shifted_to[i][k];
      }
    }
  }

  IMP_LOG(VERBOSE, "H is " << H << std::endl);

  internal::JAMA::SVD<double> svd(H);
  internal::TNT::Array2D<double> U(3, 3), V(3, 3);
  svd.getU(U);
  svd.getV(V);

  IMP_LOG(VERBOSE, "SVD is " << U << std::endl << V << std::endl);

  internal::TNT::Array1D<double> SV;
  svd.getSingularValues(SV);
  double det= SV[0]*SV[1]*SV[2];
  IMP_IF_CHECK(USAGE) {
    if (det < .00001) {
      IMP_LOG(TERSE, "FROM:\n");
      for (unsigned int i=0; i< from.size(); ++i) {
        IMP_LOG(TERSE, from[i] << std::endl);
      }
      IMP_LOG(TERSE, "TO:\n");
      for (unsigned int i=0; i< from.size(); ++i) {
        IMP_LOG(TERSE, to[i] << std::endl);
      }
      IMP_LOG(TERSE, H);
      IMP_WARN("Degenerate point set. I may not be able to align them."
               << std::endl);
    }
  }

  IMP_IF_LOG(VERBOSE) {
    internal::TNT::Array2D<double> Sigma(3,3, 0.0);

    for (int i=0; i < 3; ++i) {
      Sigma[i][i]= SV[i];
    }

    IMP_LOG(VERBOSE, "Reconstructed is "
            << internal::TNT::matmult(internal::TNT::matmult(U,Sigma),
                                      internal::TNT::transpose(V))
            << std::endl);
  }

  // the rotation matrix is R = VU^T
  internal::TNT::Array2D<double> UT = internal::TNT::transpose(U);
  internal::TNT::Array2D<double> rot(3, 3);
  rot = matmult(V, UT);

  // check for reflection
  if (determinant(rot) < 0) {
    IMP_LOG(VERBOSE, "Flipping matrix"<<std::endl);
    internal::TNT::Array2D<double> VT = internal::TNT::transpose(V);
    internal::TNT::Array2D<double> UVT = internal::TNT::matmult(U, VT);
    internal::TNT::Array2D<double> S(3, 3);
    S[0][0] = S[1][1] = 1;
    S[2][2] = determinant(UVT);
    S[0][1] = S[0][2] = S[1][0] = S[1][2] = S[2][0] = S[2][1] = 0;
    rot = internal::TNT::matmult(internal::TNT::matmult(U, S), VT);
  }

  IMP_LOG(VERBOSE, "Rotation matrix is " << rot << std::endl);

  Rotation3D rotation
    = get_rotation_from_matrix(rot[0][0], rot[0][1], rot[0][2],
                               rot[1][0], rot[1][1], rot[1][2],
                               rot[2][0], rot[2][1], rot[2][2]);
  IMP_LOG(VERBOSE, "Rotation is " << rotation << std::endl);
  VectorD<3> translation=center_to - rotation.get_rotated(center_from);

  Transformation3D ret(rotation, translation);
  return ret;
}



//! Builds the transformation required to obtain a set of points from
//! the first one
/**
  \note The function assumes that the relative distances between points
  are conserved.
**/
IMPALGEBRAEXPORT Transformation2D get_transformation_aligning_pair(
                       const std::vector<VectorD<2> >& set_from,
                       const std::vector<VectorD<2> >& set_to);

// implemented in Transformation2D

IMPALGEBRA_END_NAMESPACE

#endif  /* IMPALGEBRA_GEOMETRIC_ALIGNMENT_H */
