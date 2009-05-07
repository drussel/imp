/**
 *  \file Rotation3D.h   \brief Simple 3D rotation class.
 *
 *  Copyright 2007-9 Sali Lab. All rights reserved.
 *
 */

#ifndef IMPALGEBRA_ROTATION_3D_H
#define IMPALGEBRA_ROTATION_3D_H

#include "config.h"
#include "Vector3D.h"
#include <iostream>
#include <algorithm>
IMPALGEBRA_BEGIN_NAMESPACE

#ifndef IMP_DOXYGEN
class Rotation3D;
Rotation3D compose(const Rotation3D &a, const Rotation3D &b) ;
#endif


#ifndef IMP_DOXYGEN
/* According to benchmarks/rotate having the cache is 20% faster. Since space
   is unlikely to be an issue for rotations, we can keep the cache. */
#define IMP_ROTATION_CACHE
#endif

//! 3D rotation class.
/** Holds a three dimensional rotation compactly using a quaternion (4 numbers).
    Advantages using quaternion:
    1. Easy convertion between axis/angle to quaternion reprsentation
    2. Robustness to rounding errors.
    3. Is not subject to "Gimbal lock" (i.e. attempts to rotate an
       object fail to appear as expected, due to the order in which the
       rotations are performed) like Euler angles.
    4. Can be interpolated
    5. The quaternions representation does not harm the performance too much.
    http://www.euclideanspace.com/maths/algebra/matrix/orthogonal/rotation/index.htm

    6. a nice comparison of different implementations of rotations:
    http://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation
    Currently the rotation can be initialized from either:
    - XYZ Euler angles
    - Rotation Matrix
    - Quaternion
    - angle/axis representation
*/
class IMPALGEBRAEXPORT Rotation3D: public UninitializedDefault {
  VectorD<4> v_;
#ifdef IMP_ROTATION_CACHE
  Vector3D matrix_[3];
#endif
  friend Rotation3D compose(const Rotation3D &a, const Rotation3D &b);

public:
  //! Create an invalid rotation
  Rotation3D():v_(0,0,0,0) {}
  //! Create a rotation from a quaternion
  /** \throw ValueException if the rotation is not a unit vector.
   */
  Rotation3D(double a, double b, double c, double d): v_(a,b,c,d) {
    IMP_check(std::abs(v_.get_squared_magnitude() - 1.0) < .05,
              "Attempting to construct a rotation from a non-quaternion value."
              << " The coefficient vector must have a length of 1. Got: "
              << a << " " << b << " " << c << " " << d,
              ValueException);
    if (a<0) {
      // make them canonical
      v_=-v_;
    }
#ifdef IMP_ROTATION_CACHE
    matrix_[0][0]= v_[0]*v_[0]+v_[1]*v_[1]-v_[2]*v_[2]-v_[3]*v_[3];
    matrix_[0][1]= 2*(v_[1]*v_[2]-v_[0]*v_[3]);
    matrix_[0][2]= 2*(v_[1]*v_[3]+v_[0]*v_[2]);
    matrix_[1][0]= 2*(v_[1]*v_[2]+v_[0]*v_[3]);
    matrix_[1][1]= v_[0]*v_[0]-v_[1]*v_[1]+v_[2]*v_[2]-v_[3]*v_[3];
    matrix_[1][2]= 2*(v_[2]*v_[3]-v_[0]*v_[1]);
    matrix_[2][0]= 2*(v_[1]*v_[3]-v_[0]*v_[2]);
    matrix_[2][1]= 2*(v_[2]*v_[3]+v_[0]*v_[1]);
    matrix_[2][2]= v_[0]*v_[0]-v_[1]*v_[1]-v_[2]*v_[2]+v_[3]*v_[3];
#endif
  }
  ~Rotation3D();


#ifndef IMP_DOXYGEN
  Vector3D rotate_no_cache(const Vector3D &o) const {
    return Vector3D((v_[0]*v_[0]+v_[1]*v_[1]-v_[2]*v_[2]-v_[3]*v_[3])*o[0]
                    + 2*(v_[1]*v_[2]-v_[0]*v_[3])*o[1]
                    + 2*(v_[1]*v_[3]+v_[0]*v_[2])*o[2],
                    2*(v_[1]*v_[2]+v_[0]*v_[3])*o[0]
                    + (v_[0]*v_[0]-v_[1]*v_[1]+v_[2]*v_[2]-v_[3]*v_[3])*o[1]
                    + 2*(v_[2]*v_[3]-v_[0]*v_[1])*o[2],
                    2*(v_[1]*v_[3]-v_[0]*v_[2])*o[0]
                    + 2*(v_[2]*v_[3]+v_[0]*v_[1])*o[1]
                    + (v_[0]*v_[0]-v_[1]*v_[1]-v_[2]*v_[2]+v_[3]*v_[3])*o[2]);
  }
#endif

  //! Rotate a vector around the origin
  /**
     (q0*q0+q1*q1-q2*q2-q3*q3)*x
     + 2*(q1*q2-q0*q3)*y
     + 2*(q1*q3+q0*q2)*z,
     2*(q1*q2+q0*q3)*x
     + (q0*q0-q1*q1+q2*q2-q3*q3)*y
     + 2*(q2*q3-q0*q1)*z,
     2*(q1*q3-q0*q2)*x
     + 2*(q2*q3+q0*q1)*y
     + (q0*q0-q1*q1-q2*q2+q3*q3)*z
   */
  Vector3D rotate(const Vector3D &o) const {
    IMP_check(v_.get_squared_magnitude() >0,
              "Attempting to apply uninitialized rotation",
              InvalidStateException);
#ifdef IMP_ROTATION_CACHE
    Vector3D ret;
    for (unsigned int i=0; i< 3; ++i) {
      ret[i]=0;
      for (unsigned int j=0; j< 3; ++j) {
        ret[i]+= o[j]*matrix_[i][j];
      }
    }
    return ret;
#else
    return rotate_no_cache(o);
#endif
  }

  //! Rotate a vector around the origin
  Vector3D operator*(const Vector3D &v) const {
    return rotate(v);
  }

  void show(std::ostream& out = std::cout, std::string delim=" ") const {
    out << v_[0] << delim << v_[1]<< delim <<v_[2]<< delim <<v_[3];
  }

  //! Return the rotation which undoes this rotation.
  Rotation3D get_inverse() const;

  //! return the quaterion so that it can be stored
  const VectorD<4>& get_quaternion() const {
    return v_;
  }

  //! multiply two rotations
  Rotation3D operator*(const Rotation3D& q) const {
    return compose(*this, q);
  }

  /** \brief Return the derivative of the position x with respect to
      internal variable i. */
  const Vector3D get_derivative(const Vector3D &o, unsigned int i) const {
    /* The computation was derived in maple. Source code is probably in
       modules//algebra/tools
     */
    double t4 = v_[0]*o[0] - v_[3]*o[1] + v_[2]*o[2];
    double t5 = square(v_[0]);
    double t6 = square(v_[1]);
    double t7 = square(v_[2]);
    double t8 = square(v_[3]);
    double t9 = t5 + t6 + t7 + t8;
    double t10 = 1.0/t9;
    double t11 = 2*t4*t10;
    double t14 = v_[1]*v_[2];
    double t15 = v_[0]*v_[3];

    double t19 = v_[1]*v_[3];
    double t20 = v_[0]*v_[2];
    double t25 = square(t9);
    double t26 = 1.0/t25;

    double t27 = ((t5 + t6 - t7 - t8)*o[0] + 2*(t14 - t15)*o[1]
                  + 2*(t19 + t20)*o[2])*t26;

    double t34 = v_[3]*o[0] + v_[0]*o[1] - v_[1]*o[2];
    double t35 = 2*t34*t10;
    double t41 = v_[2]*v_[3];
    double t42 = v_[0]*v_[1];

    double t47 = (2*(t14 + t15)*o[0] + (t5 - t6 + t7 - t8)*o[1]
                  + 2*(t41 - t42)*o[2])*t26;

    double t54 = -v_[2]*o[0] + v_[1]*o[1] + v_[0]*o[2];
    double t55 = 2*t54*t10;

    double t65 = (2*(t19 - t20)*o[0] + 2*(t41 + t42)*o[1]
                  + (t5 - t6 - t7 + t8)*o[2])*t26;

    double t73 = 2*(v_[1]*o[0] + v_[2]*o[1] + v_[3]*o[2])*t10;

    /*all[1, 1] = t11 - 2*t27*v_[0];
      all[1, 2] = t35 - 2*t47*v_[0];
      all[1, 3] = t55 - 2*t65*v_[0];

      all[2, 1] = t73 - 2*t27*v_[1];
      all[2, 2] = -2*t54 t10 - 2*t47*v_[1];
      all[2, 3] = t35 - 2*t65*v_[1];

      all[3, 1] = t55 - 2*t27*v_[2];
      all[3, 2] = t73 - 2*t47*v_[2];
      all[3, 3] = -2*t4 t10 - 2*t65*v_[2];

      all[4, 1] = -2*t34 t10 - 2*t27*v_[3];
      all[4, 2] = t11 - 2*t47*v_[3];
      all[4, 3] = t73 - 2*t65*v_[3];
    */

    switch (i) {
    case 0:
    return Vector3D(t11 - 2*t27*v_[0],
                    t35 - 2*t47*v_[0],
                    t55 - 2*t65*v_[0]);
    case 1:
    return Vector3D(t73 - 2*t27*v_[1],
                    -2*t54*t10 - 2*t47*v_[1],
                    t35 - 2*t65*v_[1]);
    case 2:
    return Vector3D(t55 - 2*t27*v_[2],
                    t73 - 2*t47*v_[2],
                    -2*t4*t10 - 2*t65*v_[2]);
    case 3:
    return Vector3D(-2*t34*t10 - 2*t27*v_[3],
                    t11 - 2*t47*v_[3],
                    t73 - 2*t65*v_[3]);
    default:
      throw IndexException("Invalid derivative component");
    };
    return Vector3D(0,0,0);
  }
};


IMP_OUTPUT_OPERATOR(Rotation3D)


//! Return a rotation that does not do anything
/** \relatesalso Rotation3D */
inline Rotation3D identity_rotation() {
  return Rotation3D(1,0,0,0);
}

//! Initialize a rotation in x-y-z order from three angles
/** \param[in] xr Rotation around the X axis in radians
    \param[in] yr Rotation around the Y axis in radians
    \param[in] zr Rotation around the Z axis in radians
    \note The three rotations are represented in the original (fixed)
    coordinate frame. http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
    \relatesalso Rotation3D
*/
inline Rotation3D rotation_from_fixed_xyz(double xr,double yr, double zr)
{
  double a,b,c,d;
  double cx = cos(xr);  double cy = cos(yr);  double cz = cos(zr);
  double sx = sin(xr);  double sy = sin(yr);  double sz = sin(zr);
  double m00 = cz*cy;
  double m11 = -sy*sx*sz + cx*cz;
  double m22 = cy*cx;
  double zero =0.0;
  a = std::sqrt(std::max(1+m00+m11+m22,zero))/2.0;
  b = std::sqrt(std::max(1+m00-m11-m22,zero))/2.0;
  c = std::sqrt(std::max(1-m00+m11-m22,zero))/2.0;
  d = std::sqrt(std::max(1-m00-m11+m22,zero))/2.0;
  if (cy*sx + sy*cx*sz + sx*cz < 0.0) b = -b;
  if (sz*sx - sy*cx*cz - sy < 0.0)    c = -c;
  if (sz*cy + sy*sx*cz + sz*cx < 0.0) d = -d;
  return Rotation3D(a,b,c,d);
}

//! Initialize a rotation from euler angles
/**
    \param[in] phi   Rotation around the Z axis in radians
    \param[in] theta Rotation around the X axis in radians
    \param[in] psi   Rotation around the Z axis in radians
    \note The first rotation is by an angle phi about the z-axis.
          The second rotation is by an angle theta in [0,pi] about the
          former x-axis , and the third rotation is by an angle psi
          about the former z-axis.
    \note http://en.wikipedia.org/wiki/
         Conversion_between_quaternions_and_Euler_angles
    \relatesalso Rotation3D
*/
inline Rotation3D rotation_from_fixed_zxz(double phi, double theta, double psi)
{
  double a,b,c,d;
  double c1,c2,c3,s1,s2,s3;
  c2=std::cos(theta/2);c1=cos(phi/2);c3=cos(psi/2);
  s2=std::sin(theta/2);s1=sin(phi/2);s3=sin(psi/2);
  a = c1*c2*c3+s1*s2*s3;
  b = s1*c2*c3-c1*s2*s3;
  c = c1*s2*c3+s1*c2*s3;
  d = c1*c2*s3-s1*s2*c3;
  return Rotation3D(a,b,c,d);
}

//! Generate a Rotation3D object from a rotation matrix
/**
   \throw ValueException if the rotation is not a rotation matrix.
   \relatesalso Rotation3D
 */
IMPALGEBRAEXPORT Rotation3D
rotation_from_matrix(double m11,double m12,double m13,
                     double m21,double m22,double m23,
                     double m31,double m32,double m33);


//! Generate a rotation object from Euler Angles
/** \note Javi needs to fill in whether these are fixed and all the details to
    make it as clear as the other ones.
    \note This could be made more efficient by generating the quaternion
    terms directly.
    \param[in] Rot First Euler angle (radians) defining the rotation (Z axis)
    \param[in] Tilt Second Euler angle (radians) defining the rotation (Y axis)
    \param[in] Psi Third Euler angle (radians) defining the rotation (Z axis)
    \relatesalso Rotation3D
*/
inline Rotation3D rotation_from_fixed_zyz(double Rot, double Tilt, double Psi) {
  double c1 = std::cos(Rot);
  double c2 = std::cos(Tilt);
  double c3 = std::cos(Psi);
  double s1 = std::sin(Rot);
  double s2 = std::sin(Tilt);
  double s3 = std::sin(Psi);

  double d00 = c1 * c2 * c3 - s1 * s3;
  double d01 = (-1.0) * c2 * c3 * s1 - c1 * s3;
  double d02 = c3 * s2;
  double d10 = c3 * s1 + c1 * c2 * s3;
  double d11 = c1 * c3 - c2 * s1 * s3;
  double d12 = s2 * s3;
  double d20 = (-1.0) * c1 * s2;
  double d21 = s1 * s2;
  double d22 = c2;
  return rotation_from_matrix(d00, d01, d02,
                              d10, d11, d12,
                              d20, d21, d22);
}


//! Generate a Rotation3D object from a rotation around an axis
/**
  \param[in] axis the rotation axis passes through (0,0,0)
  \param[in] angle the rotation angle in radians
  \note http://en.wikipedia.org/wiki/Rotation_matrix
  \note www.euclideanspace.com/maths/geometry/rotations/conversions/
  angleToQuaternion/index.htm
  \relatesalso Rotation3D
*/
inline Rotation3D rotation_in_radians_about_axis(const Vector3D& axis,
                                                 double angle)
{
  //normalize the vector
  Vector3D axis_norm = axis.get_unit_vector();
  double s = std::sin(angle/2);
  double a,b,c,d;
  a = std::cos(angle/2);
  b = axis_norm[0]*s;
  c = axis_norm[1]*s;
  d = axis_norm[2]*s;
  return Rotation3D(a,b,c,d);
}

//! Create a rotation from the first vector to the second one.
/** \relatesalso Rotation3D
 */
inline Rotation3D rotation_taking_first_to_second(const Vector3D &v1,
                                                  const Vector3D &v2) {
    Vector3D v1_norm = v1.get_unit_vector();
    Vector3D v2_norm = v2.get_unit_vector();
    //get a vector that is perpendicular to the plane containing v1 and v2
    Vector3D vv = vector_product(v1_norm,v2_norm);
    //get the angle between v1 and v2
    double dot = v1_norm*v2_norm;
    dot = ( dot < -1.0 ? -1.0 : ( dot > 1.0 ? 1.0 : dot ) );
    double angle = std::acos(dot);
    //check a special case: the input vectors are parallel / antiparallel
    if (std::abs(dot) == 1.0) {
      IMP_LOG(VERBOSE," the input vectors are (anti)parallel "<<std::endl);
      return rotation_in_radians_about_axis(orthogonal_vector(v1),angle);
    }
    return rotation_in_radians_about_axis(vv,angle);
}


//! Pick a rotation at random from all possible rotations
/** \relatesalso Rotation3D */
IMPALGEBRAEXPORT Rotation3D random_rotation();


//! Compute a rotatation from an unnormalized quaternion
/** \relatesalso Rotation3D */
inline Rotation3D rotation_from_vector4d(const VectorD<4> &v) {
  VectorD<4> uv= v.get_unit_vector();
  return Rotation3D(uv[0], uv[1], uv[2], uv[3]);
}


/** \relatesalso Rotation3D
 */
inline Rotation3D compose(const Rotation3D &a, const Rotation3D &b) {
  return Rotation3D(a.v_[0]*b.v_[0] - a.v_[1]*b.v_[1]
                    - a.v_[2]*b.v_[2] - a.v_[3]*b.v_[3],
                    a.v_[0]*b.v_[1] + a.v_[1]*b.v_[0]
                    + a.v_[2]*b.v_[3] - a.v_[3]*b.v_[2],
                      a.v_[0]*b.v_[2] - a.v_[1]*b.v_[3]
                    + a.v_[2]*b.v_[0] + a.v_[3]*b.v_[1],
                    a.v_[0]*b.v_[3] + a.v_[1]*b.v_[2]
                    - a.v_[2]*b.v_[1] + a.v_[3]*b.v_[0]);
}

IMPALGEBRA_END_NAMESPACE
#endif  /* IMPALGEBRA_ROTATION_3D_H */
