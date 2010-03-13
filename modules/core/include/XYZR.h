/**
 *  \file XYZR.h
 *  \brief Decorator for a sphere-like particle.
 *
 *  Copyright 2007-2010 IMP Inventors. All rights reserved.
 *
 */

#ifndef IMPCORE_XYZ_R_H
#define IMPCORE_XYZ_R_H

#include "XYZ.h"
#include <IMP/algebra/Sphere3D.h>

#include <limits>

IMPCORE_BEGIN_NAMESPACE

//! A decorator for a particle with x,y,z coordinates and a radius.
/** \ingroup decorators

    A simple example illustrating some of the functionality.
    \htmlinclude XYZR_Decorator.py.html
 */
class IMPCOREEXPORT XYZR: public DecoratorWithTraits<XYZ, FloatKey>
{
public:
  IMP_DECORATOR_TRAITS(XYZR, XYZ, FloatKey,
                       radius_key, get_default_radius_key());

  /** Create a decorator using radius_key to store the FloatKey.
     \param[in] p The particle to wrap.
     \param[in] radius_key The (optional) key name to use.
     The default is "radius".
   */
  static XYZR setup_particle(Particle *p,
                     FloatKey radius_key= get_default_radius_key()) {
    if (!XYZ::particle_is_instance(p)) {
      XYZ::setup_particle(p);
    }
    p->add_attribute(radius_key, 0, false);
    return XYZR(p, radius_key);
  }


 /** Create a decorator using radius_key to store the FloatKey.
     The particle should already be an XYZ particle.
     \param[in] p The particle to wrap.
     \param[in] radius The radius to set initially
     \param[in] radius_key The (optional) key name to use.
     The default is "radius".
   */
  static XYZR setup_particle(Particle *p,
                     Float radius,
                     FloatKey radius_key= get_default_radius_key()) {
    p->add_attribute(radius_key, radius, false);
    return XYZR(p, radius_key);
  }

  /** Create a decorator using radius_key to store the FloatKey.
     \param[in] p The particle to wrap.
     \param[in] s The sphere to use to set the position and radius
     \param[in] radius_key The (optional) key name to use.
     The default is "radius".
   */
  static XYZR setup_particle(Particle *p,
                     const algebra::SphereD<3> &s,
                     FloatKey radius_key= get_default_radius_key()) {
    XYZ::setup_particle(p, s.get_center());
    p->add_attribute(radius_key, s.get_radius(), false);
    return XYZR(p, radius_key);
  }

  //! Check if the particle has the required attributes
  static bool particle_is_instance(Particle *p,
                             FloatKey radius_key= get_default_radius_key()) {
    return p->has_attribute(radius_key);
  }
  IMP_DECORATOR_GET_SET(radius, get_radius_key(), Float, Float);


  //! Return a sphere object
  algebra::SphereD<3> get_sphere() const {
    return algebra::SphereD<3>(get_coordinates(), get_radius());
  }

  //! Set the attributes from a sphere
  void set_sphere(const algebra::SphereD<3> &s) {
    set_coordinates(s.get_center());
    set_radius(s.get_radius());
  }
  //! Get the default radius key.
  static FloatKey get_default_radius_key() {
    return IMP::internal::xyzr_keys[3];
  }
  void add_to_radius_derivative(double v,
                                DerivativeAccumulator &d) {
    get_particle()->add_to_derivative(get_radius_key(), v, d);
  }
};

IMP_OUTPUT_OPERATOR(XYZR);

IMP_DECORATORS(XYZR,XYZRs, XYZs);

//! Compute the distance between a pair of particles
/** \relatesalso XYZR
 */
inline double get_distance(XYZR a, XYZR b) {
  return IMP::algebra::get_distance(a.get_sphere(), b.get_sphere());
}

//! Set the coordinates and radius of the first to enclose the list
/** \param[in] v The vector of XYZ or XYZR particles to enclose
    \param[out] b The one whose values should be set
    \param[in] slack An amount to add to the radius.
    Any particle which does not have the attribute b.get_radius()
    is assumed to have a radius of 0.

    \note This function produces tighter bounds if the \ref cgal "CGAL"
    library is available.
    \ingroup CGAL
    \relatesalso XYZR
 */
IMPCOREEXPORT void set_enclosing_sphere(XYZR b,
                                        const XYZsTemp &v,
                                        double slack=0);

//! Set the radius of the first to enclose the list
/** \param[in] v The vector of XYZ or XYZR particles to enclose
    \param[out] b The one whose radius should be set
    Any particle which does not have the attribute b.get_radius()
    is assumed to have a radius of 0.

    \relatesalso XYZR
 */
IMPCOREEXPORT void set_enclosing_radius(XYZR b,
                                        const XYZsTemp &v);

//! Get a sphere enclosing the set of XYZRs
/** \param[in] v The one whose radius should be set
    \param[in] rk The radius key to use
    Any particle which does not have the attribute b.get_radius()
    is assumed to have a radius of 0.

    \relatesalso XYZR
 */
IMPCOREEXPORT algebra::SphereD<3> get_enclosing_sphere(const XYZsTemp& v,
                             FloatKey rk=XYZR::get_default_radius_key());

//! Create a set of particles which random coordinates
/** This function is mostly to be used to keep demo code brief.
    \param[in] m The model to add them to.
    \param[in] num The number of particles to create.
    \param[in] radius The radius to give them.
    \param[in] box_side The particles have coordinates from -box_side
    to box_side.
    \relatesalso XYZR

    The particles coordinates are optimized.
 */
IMPCOREEXPORT XYZRs create_xyzr_particles(Model *m,
                                          unsigned int num,
                                          Float radius,
                                          Float box_side=10);

/** \genericgeometry */
inline const algebra::SphereD<3> get_geometry(XYZR d) {return d.get_sphere();}
/** \genericgeometry */
inline const algebra::SphereD<3>& get_geometry(const algebra::SphereD<3> &v) {
  return v;
}
/** \genericgeometry */
inline void set_geometry(XYZR d, const algebra::SphereD<3> &v) {
  d.set_sphere(v);
}
/** \genericgeometry */
inline void set_geometry(algebra::SphereD<3> &vbase,
                         const algebra::SphereD<3> &v) {vbase=v;}

IMPCORE_END_NAMESPACE

#endif  /* IMPCORE_XYZ_R_H */
