/**
 *  \file XYZR.h
 *  \brief Decorator for a sphere-like particle.
 *
 *  Copyright 2007-9 Sali Lab. All rights reserved.
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
    \verbinclude xyzrdecorator.py
 */
class IMPCOREEXPORT XYZR: public XYZ
{
public:
  IMP_DECORATOR_TRAITS(XYZR, XYZ, FloatKey,
                       radius_key, get_default_radius_key());

  /** Create a decorator using radius_key to store the FloatKey.
     \param[in] p The particle to wrap.
     \param[in] radius_key The (optional) key name to use.
     The default is "radius".
   */
  static XYZR create(Particle *p,
                              FloatKey radius_key= FloatKey("radius")) {
    if (!XYZ::is_instance_of(p)) {
      XYZ::create(p);
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
  static XYZR create(Particle *p,
                              Float radius,
                              FloatKey radius_key= FloatKey("radius")) {
    p->add_attribute(radius_key, radius, false);
    return XYZR(p, radius_key);
  }

  /** Create a decorator using radius_key to store the FloatKey.
     \param[in] p The particle to wrap.
     \param[in] s The sphere to use to set the position and radius
     \param[in] radius_key The (optional) key name to use.
     The default is "radius".
   */
  static XYZR create(Particle *p,
                              const algebra::Sphere3D &s,
                              FloatKey radius_key= FloatKey("radius")) {
    XYZ::create(p, s.get_center());
    p->add_attribute(radius_key, s.get_radius(), false);
    return XYZR(p, radius_key);
  }

  //! Check if the particle has the required attributes
  static bool is_instance_of(Particle *p,
                             FloatKey radius_key= FloatKey("radius")) {
    return p->has_attribute(radius_key);
  }
  IMP_DECORATOR_GET_SET(radius, get_radius_key(), Float, Float);


  //! Return a sphere object
  algebra::Sphere3D get_sphere() const {
    return algebra::Sphere3D(get_coordinates(), get_radius());
  }

  //! Set the attributes from a sphere
  void set_sphere(const algebra::Sphere3D &s) {
    set_coordinates(s.get_center());
    set_radius(s.get_radius());
  }
  //! Get the default radius key.
  static FloatKey get_default_radius_key() {
    static FloatKey rk("radius");
    return rk;
  }
};

IMP_OUTPUT_OPERATOR(XYZR);

//! Compute the distance between a pair of particles
/** \relatesalso XYZR
 */
IMPCOREEXPORT Float distance(XYZR a, XYZR b);

//! Set the coordinates and radius of the first to enclose the list
/** \param[in] v The vector of XYZ or XYZR particles to enclose
    \param[out] b The one whose values should be set
    Any particle which does not have the attribute b.get_radius()
    is assumed to have a radius of 0.

    \note This takes a Particles object rather than a vector of
    something else since you can't easily cast vectors of
    different things to one another.

    \note This function produces tighter bounds if the \ref CGAL "CGAL"
    library is available.
    \ingroup CGAL
    \relatesalso XYZR
 */
IMPCOREEXPORT void set_enclosing_sphere(XYZR b,
                                        const Particles &v);

//! Set the radius of the first to enclose the list
/** \param[in] v The vector of XYZ or XYZR particles to enclose
    \param[out] b The one whose radius should be set
    Any particle which does not have the attribute b.get_radius()
    is assumed to have a radius of 0.

    \note This takes a Particles object rather than a vector of
    something else since you can't easily cast vectors of
    different things to one another.

    \relatesalso XYZR
 */
IMPCOREEXPORT void set_enclosing_radius(XYZR b,
                                        const Particles &v);

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
IMPCOREEXPORT Particles create_xyzr_particles(Model *m,
                                              unsigned int num,
                                              Float radius,
                                              Float box_side=10);

class HierarchyTraits;
class Hierarchy;

//! Create a hierarchical cover of a set of XYZR particles
/** \unimplemented{create_sphere_hierarchy}
*/
IMPCOREEXPORT Hierarchy
create_sphere_hierarchy(const Particles &ps,
                        const HierarchyTraits &traits);

IMPCORE_END_NAMESPACE

#endif  /* IMPCORE_XYZ_R_H */
