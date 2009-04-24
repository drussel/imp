/**
 *  \file Transformation3D.h   \brief Simple 3D transformation class.
 *
 *  Copyright 2007-9 Sali Lab. All rights reserved.
 *
 */

#ifndef IMPALGEBRA_TRANSFORMATION_3D_H
#define IMPALGEBRA_TRANSFORMATION_3D_H

#include "config.h"

#include "Vector3D.h"
#include "Rotation3D.h"

IMPALGEBRA_BEGIN_NAMESPACE

#ifndef IMP_DOXYGEN
class Transformation3D;
Transformation3D compose(const Transformation3D &a,
                         const Transformation3D &b);
#endif

//! Simple 3D transformation class
/** \see IMP::core::Transform
*/
class IMPALGEBRAEXPORT Transformation3D: public UninitializedDefault
{
public:
  // public for swig
  typedef Transformation3D This;
  //! construct and invalid transformation
  Transformation3D(){}
  /** basic constructor*/
  Transformation3D(const Rotation3D& r,
                   const Vector3D& t=Vector3D(0,0,0)):
    trans_(t), rot_(r){}
  /** Construct a transformation with an identity rotation.*/
  Transformation3D(const Vector3D& t):
    trans_(t), rot_(identity_rotation()){}
  ~Transformation3D();
  //! transform
  Vector3D transform(const Vector3D &o) const {
    return rot_.rotate(o) + trans_;
  }
  //! apply transformation
  Vector3D operator*(const Vector3D &v) {
    return transform(v);
  }
  /** multiply two rigid transformation such that for any vector v
      (rt1*rt2)*v = rt1*(rt2*v) */
  Transformation3D operator*(const Transformation3D &tr) {
    return compose(*this, tr);
  }
  //! get the rotation part
  const Rotation3D& get_rotation() const {
    return rot_;
  }
  //! Get the translation part
  const Vector3D& get_translation()const{return trans_;}

  void show(std::ostream& out = std::cout) const {
    rot_.show(out);
    out<<" || "<<trans_;
  }
  Transformation3D get_inverse() const;
private:
  Vector3D trans_; //tranlation
  Rotation3D rot_;  //rotation
};

IMP_OUTPUT_OPERATOR(Transformation3D)


//! Return a transformation that does not do anything
/** \relatesalso Transformation3D */
inline Transformation3D identity_transformation() {
  return Transformation3D(identity_rotation(),Vector3D(0.0,0.0,0.0));
}

//! Generate a transformation from the natural reference-frame to
//  a different one
/**
  \param[in] u     vector used to define the new reference frame
  \param[in] w     vector used to define the new reference frame
  \param[in] base  the center of the new reference frame
  \brief A rotation from the natural reference frame to one defined by u,w
         and base.
         The x-axis lies on u-base.
         The y-axis is perpendicular to both x and z.
         The z-axis is perpendicular to the u-base , w-base plane

   \note This function is poorly designed and liable the change. The
   main problem comes from having the three arguments of the same type
   with no natural order amongst them.

   \relatesalso Transformation3D
 */
inline Transformation3D transformation_from_reference_frame(const Vector3D &u,
                                                const Vector3D &w,
                                                const Vector3D &base) {
  Vector3D x = (u-base);
  Vector3D z = vector_product(x,w-base);
  Vector3D y = vector_product(z,x);
  Vector3D xu = x.get_unit_vector();
  Vector3D zu = z.get_unit_vector();
  Vector3D yu = y.get_unit_vector();
  Rotation3D rot = rotation_from_matrix(xu[0],xu[1],xu[2],
                                        yu[0],yu[1],yu[2],
                                        zu[0],zu[1],zu[2]).get_inverse();
  return Transformation3D(rot,base);
}

//! Generate a Transformation3D object from a rotation around an axis
/**
  \param[in] point the rotation axis passes through the point
  \param[in] direction the direction of the rotation axis
  \param[in] angle the rotation angle in radians
  \relatesalso Transformation3D
*/
inline Transformation3D
rotation_in_radians_about_axis(const Vector3D &point,
                               const Vector3D &direction,
                               double angle) {
  Rotation3D r = rotation_in_radians_about_axis(direction,angle);
  return Transformation3D(r, (r*(-point)+point));
}

//! compose two transformations
  /** For any vector v (a*a)*v = a*(b*v).
      \relatesalso Transformation3D
   */
inline Transformation3D compose(const Transformation3D &a,
                                const Transformation3D &b){
  return Transformation3D(compose(a.get_rotation(), b.get_rotation()),
                          a.transform(b.get_translation()));
}

IMPALGEBRA_END_NAMESPACE

#endif  /* IMPALGEBRA_TRANSFORMATION_3D_H */
