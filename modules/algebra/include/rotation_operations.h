/**
 *  \file rotation_operations.h
 *  \brief Classes and operations related with rotations
 *  \author Javier Velazquez-Muriel
 *
 *  Copyright 2007-9 Sali Lab. All rights reserved.
*/

#ifndef IMPALGEBRA_ROTATION_OPERATIONS_H
#define IMPALGEBRA_ROTATION_OPERATIONS_H

#include "config.h"
#include "utility.h"
#include "VectorD.h"
#include "Vector3D.h"
#include "Matrix2D.h"
#include "Rotation3D.h"
#include <IMP/deprecation.h>

IMPALGEBRA_BEGIN_NAMESPACE

//! Stores a 2D rotation matrix
/**
  \note This class requires the angles to be given in radians, and the
  onvention used is that the rotations are performed rotating counterclockwise
  (right hand side convention).

  \note The name of this class is liable to change to Rotation2D at any time.
**/
class Rotation2D
{
public:
  Rotation2D() {
    set_angle(0.0);
  };

  //! Builds the matrix for the given angle
  Rotation2D(double psi) {
    set_angle(psi);
  }

  //! sets the angle for the rotation
  /**
  * \param[in] psi the angle
  */
  void set_angle(double psi) {
    _psi = psi;
    _c = cos(psi);
    _s = sin(psi);
  }

  //! rotates a 2D point
  /**
  * \param[in] o a 2D vector to be rotated
  */
  VectorD<2> rotate(const VectorD<2> &o) const {
    return rotate(o[0],o[1]);
  }

  //! rotates a 2D point
  VectorD<2> rotate(const double x,const double y) const {
    return VectorD<2>(_c*x-_s*y , _s*x+_c*y);
  }

  //! Returns the matrix for the inverse rotation
  Rotation2D get_inverse() {
    return Rotation2D(-_psi);
  }
private:
  double _psi; // angle
  double _c; // cosine of the angle
  double _s; // sine of the angle
};


//! Rotates a Matrix2D a given angle (in radians). The center of the matrix
//! is used as the center of rotation. The input matrix is not modified.
/**
  \param[in] m the matrix to rotate
  \param[out] result the result matrix. It cannot be input matrix.
  \param[in] ang angle
  \param[in] wrap if true, the Matrix2D is wrapped.
                      See help for bilinear_interp()
  \param[in] wrap if true, the Matrix2D is wrapped.
                      See help for bilinear_interp()
  \param[in] outside Value to apply if a rotated point falls outside the
               limits of the matrix. (default = 0.0)
**/
template<typename T>
void rotate_matrix_2D(Matrix2D<T>& m,double ang,
              Matrix2D<T>& result,
              bool wrap=false,T outside=0.0) {
  result.resize(m);
  // Save the origin and center
  std::vector<int> orig(2);
  orig[0]=m.get_start(0);
  orig[1]=m.get_start(1);
  m.centered_start();
  result.centered_start();
  // Build the rotation needed for the angle (build the inverse directly)
  Rotation2D inv_rot((-1)*ang);
  // Roll over the destination matrix
  for(int i = result.get_start(0);i<=result.get_finish(0);i++) { // Y
    for(int j = result.get_start(1);j<=result.get_finish(1);j++) { // X
      // Compute the equivalent point in the original matrix
      VectorD<2> v = inv_rot.rotate((double)i,(double)j);
      // Interpolate the value from the original matrix
      result(i,j) = m.bilinear_interp(v,wrap,outside);
    }
  }
  // Restore the origins
  m.set_start(orig);
  result.set_start(orig);
}


//! Rotates a Matrix2D a given angle (in radians). The center of the matrix
//! is used as the center of rotation. The matrix is modified
/**
  \param[in] m the matrix to rotate
  \param[in] ang angle
  \param[in] wrap if true, the Matrix2D is wrapped.
                      See help for bilinear_interp()
  \param[in] outside Value to apply if a rotated point falls outside the
               limits of the matrix. (default = 0.0)
**/
template<typename T>
void auto_rotate_matrix_2D(Matrix2D<T>& m,double ang,
                           bool wrap=false,T outside=0.0) {
  Matrix2D<T> result;
  rotate_matrix_2D(m,ang,result,wrap,outside);
  m.copy(result);
}

IMPALGEBRA_END_NAMESPACE
#endif  /* IMPALGEBRA_ROTATION_OPERATIONS_H */
