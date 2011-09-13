/**
 *  \file atom/Diffusion.h
 *  \brief A decorator for a diffusing particle.
 *
 *  Copyright 2007-2011 IMP Inventors. All rights reserved.
 *
 */

#ifndef IMPATOM_DIFFUSION_H
#define IMPATOM_DIFFUSION_H

#include "atom_config.h"

#include <IMP/core/XYZR.h>
#include <IMP/algebra/Vector3D.h>
#include <IMP/internal/constants.h>

#include <vector>
#include <limits>

IMPATOM_BEGIN_NAMESPACE

//! A decorator for a diffusing particle.
/** \ingroup helper
    \ingroup decorators
    \see BrownianDynamics
    \unstable{Diffusion} The name really should be fixed.

    D is assumed to be in \f$A^2/fs\f$
 */
class IMPATOMEXPORT Diffusion:
  public IMP::core::XYZ
{
 public:
  IMP_DECORATOR(Diffusion, IMP::core::XYZ);

  /** Create a decorator with the passed coordinates and D.
  */
  static Diffusion setup_particle(Particle *p,
                          const algebra::Vector3D &v,
                          Float D) {
    XYZ::setup_particle(p, v);
    p->add_attribute(get_d_key(), D);
    return Diffusion(p);
  }

  /** Create a decorator with the a given D.
      The particle
      is assumed to already have x,y,z attributes
  */
  static Diffusion setup_particle(Particle *p,
                          Float D) {
    IMP_USAGE_CHECK(XYZ::particle_is_instance(p),
              "Particle must already be an XYZ particle");
    p->add_attribute(get_d_key(), D);
    return Diffusion(p);
  }

  /** Create a decorator with the D determined from its
      radius.
  */
  static Diffusion setup_particle(Particle *p);

  //! Return true if the particle is an instance of an Diffusion
  static bool particle_is_instance(Particle *p) {
    return XYZ::particle_is_instance(p)
      && p->has_attribute(get_d_key());
  }

  //! Return true if the particle is an instance of an Diffusion
  static bool particle_is_instance(Model *m, ParticleIndex p) {
    return m->get_has_attribute(get_d_key(), p);
  }

  void set_d(double d) {
    get_particle()->set_value(get_d_key(), d);
  }
  double get_d() const {
    return get_particle()->get_value(get_d_key());
  }

  //! Get the D key
  static FloatKey get_d_key();
};

IMPATOMEXPORT double get_d_from_cm2_per_second(double din);

IMP_OUTPUT_OPERATOR(Diffusion);


IMP_DECORATORS(Diffusion, Diffusions, core::XYZs);







/** A rigid body that is diffusing, so it also has a rotation diffusion
    coefficient. The units on the rotational coefficient are radians^2/fs.*/
class IMPATOMEXPORT RigidBodyDiffusion: public Diffusion {
 public:
  IMP_DECORATOR(RigidBodyDiffusion, Diffusion);
  /** All diffusion coefficients are determined from the radius */
  static RigidBodyDiffusion setup_particle(Particle *p);

  double get_d_rotation() const {
    return get_particle()->get_value(get_d_rotation_key());
  }
  void set_d_rotation(double d) const {
    return get_particle()->set_value(get_d_rotation_key(), d);
  }

  //! Return true if the particle is an instance of an Diffusion
  static bool particle_is_instance(Particle *p) {
    return XYZ::particle_is_instance(p)
      && p->has_attribute(get_d_key());
  }

  //! Return true if the particle is an instance of an Diffusion
  static bool particle_is_instance(Model *m, ParticleIndex p) {
    return m->get_has_attribute(get_d_rotation_key(), p);
  }

  //! Get the D key
  static FloatKey get_d_rotation_key();
};

IMP_OUTPUT_OPERATOR(RigidBodyDiffusion);


IMP_DECORATORS(RigidBodyDiffusion, RigidBodyDiffusions, Diffusions);

IMPATOM_END_NAMESPACE

#endif  /* IMPATOM_DIFFUSION_H */
