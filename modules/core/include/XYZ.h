/**
 *  \file XYZ.h     \brief Simple xyz decorator.
 *
 *  Copyright 2007-2011 IMP Inventors. All rights reserved.
 *
 */

#ifndef IMPCORE_XY_Z_H
#define IMPCORE_XY_Z_H

#include "core_config.h"
#include "../macros.h"
#include "internal/dihedral_helpers.h"

#include <IMP/Decorator.h>
#include <IMP/algebra/Vector3D.h>
#include <IMP/algebra/Transformation3D.h>
#include <vector>
#include <limits>

IMPCORE_BEGIN_NAMESPACE

//! A a decorator for a particle with x,y,z coordinates.
/** \inlineimage{xyz.png, 50} Using the decorator one can
    get and set coordinates and modify derivatives.

    \ingroup helper
    \ingroup decorators
    \pythonexample{XYZ_Decorator}
    \see XYZR
 */
class IMPCOREEXPORT XYZ: public Decorator
{
 public:

  static FloatKey get_coordinate_key(unsigned int i) {
    IMP_USAGE_CHECK(i <3, "Out of range coordinate");
    return IMP::internal::xyzr_keys[i];
  }

  IMP_DECORATOR(XYZ, Decorator);

  /** Create a decorator with the passed coordinates. */
  static XYZ setup_particle(Particle *p,
                    const algebra::VectorD<3> &v=
                    algebra::VectorD<3>(0,0,0)) {
    p->add_attribute(get_coordinate_key(0),v[0]);
    p->add_attribute(get_coordinate_key(1),v[1]);
    p->add_attribute(get_coordinate_key(2),v[2]);
    return XYZ(p);
  }

  IMP_DECORATOR_GET_SET(x, get_coordinate_key(0), Float, Float);
  IMP_DECORATOR_GET_SET(y, get_coordinate_key(1), Float, Float);
  IMP_DECORATOR_GET_SET(z, get_coordinate_key(2), Float, Float);
  //! set the ith coordinate
  void set_coordinate(unsigned int i, Float v) {
    get_particle()->set_value(get_coordinate_key(i), v);
  }
  //! set all coordinates from a vector
  void set_coordinates(const algebra::VectorD<3> &v) {
    get_particle()->_access_coordinates()._access_center()=v;
  }

  //! Get the ith coordinate
  Float get_coordinate(int i) const {
    return get_coordinates()[i];
  }
  //! Get the ith coordinate derivative
  Float get_derivative(int i) const {
    return get_particle()->get_derivative(get_coordinate_key(i));
  }
  //! Add something to the derivative of the ith coordinate
  void add_to_derivative(int i, Float v,
                                    DerivativeAccumulator &d) {
    get_particle()->add_to_derivative(get_coordinate_key(i), v, d);
  }
  //! Add something to the derivative of the coordinates
  void add_to_derivatives(const algebra::VectorD<3>& v,
                          DerivativeAccumulator &d) {
    add_to_derivative(0, v[0], d);
    add_to_derivative(1, v[1], d);
    add_to_derivative(2, v[2], d);
  }
  //! Get whether the coordinates are optimized
  /** \return true only if all of them are optimized.
    */
  bool get_coordinates_are_optimized() const {
    return get_particle()->get_is_optimized(get_coordinate_key(0))
      && get_particle()->get_is_optimized(get_coordinate_key(1))
      && get_particle()->get_is_optimized(get_coordinate_key(2));
  }
  //! Set whether the coordinates are optimized
  void set_coordinates_are_optimized(bool tf) const {
    get_particle()->set_is_optimized(get_coordinate_key(0), tf);
    get_particle()->set_is_optimized(get_coordinate_key(1), tf);
    get_particle()->set_is_optimized(get_coordinate_key(2), tf);
  }

  //! Get the vector from this particle to another
  algebra::VectorD<3> get_vector_to(const XYZ &b) const {
    return algebra::VectorD<3>(b.get_coordinate(0) - get_coordinate(0),
                    b.get_coordinate(1) - get_coordinate(1),
                    b.get_coordinate(2) - get_coordinate(2));
  }

  //! Convert it to a vector.
  /** Somewhat suspect based on wanting a Point/Vector differentiation
      but we don't have points */
  const algebra::VectorD<3>& get_coordinates() const {
    return get_particle()->_get_coordinates().get_center();;
  }

  //! Get the vector of derivatives.
  /** Somewhat suspect based on wanting a Point/Vector differentiation
      but we don't have points */
  algebra::VectorD<3> get_derivatives() const {
    return algebra::VectorD<3>(get_derivative(0),
                             get_derivative(1),
                             get_derivative(2));
  }

  static bool particle_is_instance(Particle *p) {
    IMP_USAGE_CHECK((p->has_attribute(get_coordinate_key(2))
               && p->has_attribute(get_coordinate_key(0))
               && p->has_attribute(get_coordinate_key(1)))
              || (!p->has_attribute(get_coordinate_key(2))
                  && !p->has_attribute(get_coordinate_key(0))
                  && !p->has_attribute(get_coordinate_key(1))),
              "Particle expected to either have all of x,y,z or none.");
    return p->has_attribute(get_coordinate_key(2));
  }

  //! Get a vector containing the keys for x,y,z
  /** This is quite handy for initializing movers and things.
   */
  static const FloatKeys& get_xyz_keys();
};

IMP_OUTPUT_OPERATOR(XYZ);

//! Compute the distance between a pair of particles
/** \ingroup helper
    \relatesalso XYZ
 */
inline double get_distance(XYZ a, XYZ b) {
  return algebra::get_distance(a.get_coordinates(),b.get_coordinates());
}

//! Compute the dihedral angle (in radians) between the four particles
/** \ingroup helper
    \relatesalso XYZ
 */
inline double get_dihedral(XYZ a, XYZ b, XYZ c, XYZ d) {
  return internal::dihedral(a, b, c, d, NULL, NULL, NULL, NULL);
}

//! Apply a transformation to the particle
/** \relatesalso XYZ
    \relatesalso algebra::Transformation3D
*/
IMPCOREEXPORT void transform(XYZ a, const algebra::Transformation3D &tr);

/** \genericgeometry */
inline const algebra::VectorD<3> get_vector_d_geometry(XYZ d) {
  return d.get_coordinates();
}
/** \genericgeometry */
inline void set_vector_d_geometry(XYZ d, const algebra::VectorD<3> &v) {
  d.set_coordinates(v);
}

IMP_DECORATORS(XYZ,XYZs, Particles);


IMPCORE_END_NAMESPACE


#ifndef SWIG
// use koenig lookup
// swig doesn't like having the overloads in different namespaces
// it will do the conversion implicitly anyway
IMP_BEGIN_NAMESPACE
/** \genericgeometry */
inline const algebra::VectorD<3> get_vector_d_geometry(Particle *p) {
  return core::XYZ(p).get_coordinates();
}
/** \genericgeometry */
inline void set_vector_d_geometry(Particle *p, const algebra::VectorD<3> &v) {
  core::XYZ(p).set_coordinates(v);
}

IMP_END_NAMESPACE
#endif

#endif  /* IMPCORE_XY_Z_H */
