/**
 *  \file Rotation2D.h
 *  \brief Classes and operations related with rotations
 *  \author Javier Velazquez-Muriel
 *
 *  Copyright 2007-2010 Sali Lab. All rights reserved.
*/

#ifndef IMPALGEBRA_ROTATION_2D_H
#define IMPALGEBRA_ROTATION_2D_H

#include "config.h"
#include "utility.h"
#include "Vector2D.h"
#include "Matrix2D.h"
#include "IMP/constants.h"
#include <cmath>
#include <stdlib.h>

IMPALGEBRA_BEGIN_NAMESPACE


#if !defined(IMP_DOXYGEN) && !defined(SWIG)
class Rotation2D;
Rotation2D compose(const Rotation2D &a, const Rotation2D &b) ;
#endif


//! Stores a 2D rotation matrix
/**
  \note This class requires the angles to be given in radians, and the
  convention used is that the rotations are performed rotating counterclockwise
  (right hand side convention).

  \geometry
**/
class Rotation2D
{
public:
  Rotation2D(): angle_(std::numeric_limits<double>::quiet_NaN()) {};

  //! Builds the matrix for the given angle
  Rotation2D(double angle) {
    set_angle(angle);
  }

  //! rotates a 2D point
  /**
  * \param[in] o a 2D vector to be rotated
  */
   Vector2D rotate(const  Vector2D &o) const {
    IMP_INTERNAL_CHECK(!is_nan(angle_),
               "Attempting to use uninitialized rotation");
    return rotate(o[0],o[1]);
  }

  //! rotates a 2D point
   Vector2D rotate(const double x,const double y) const {
    IMP_INTERNAL_CHECK(!is_nan(angle_),
               "Attempting to use uninitialized rotation");
    return  Vector2D(c_*x-s_*y , s_*x+c_*y);
  }

  //! Returns the matrix for the inverse rotation
  Rotation2D get_inverse() const {
    IMP_INTERNAL_CHECK(!is_nan(angle_),
               "Attempting to use uninitialized rotation");
    return Rotation2D(-angle_);
  }

  //! sets the angle for the rotation
  /**
  * \param[in] angle the angle
  */
  void set_angle(double angle) {
    angle_ = angle;
    c_ = cos(angle);
    s_ = sin(angle);
  }

  //! gets the angle
  double get_angle() const {
    return angle_;
  }

  //! Prints the angle
  void show(std::ostream& out = std::cout, std::string delim=" ") const {
    out << "Rotation2D (radians): " << angle_;
  }

private:
  double angle_; // angle
  double c_; // cosine of the angle
  double s_; // sine of the angle
};


//! Builds an identity rotation in 2D
inline Rotation2D identity_rotation2D() {
  return Rotation2D(0.0);
};

//! Builds an identity rotation in 2D
inline Rotation2D random_rotation2D() {
  return Rotation2D(2*PI*((double)rand() /((double)RAND_MAX+1)));
};


//! Builds the rotation that transforms the vector X of the origin
//! of coordinates into the given vector
inline Rotation2D build_Rotation2D_from_Vector2D(const Vector2D &v) {
  return Rotation2D(atan2(v[1],v[0]));
};

//! compose two rotations a and b
/**
  For any vector v (a*b)*v = a*(b*v).
*/
inline Rotation2D compose(const Rotation2D &a,const Rotation2D &b) {
  double new_angle = a.get_angle()+b.get_angle();
  Rotation2D R(new_angle);
  return R;
};



IMPALGEBRA_END_NAMESPACE
#endif  /* IMPALGEBRA_ROTATION_2D_H */
