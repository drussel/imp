/**
 *  \file Diffusion.cpp   \brief Simple xyzr decorator.
 *
 *  Copyright 2007-2012 IMP Inventors. All rights reserved.
 *
 */

#include "IMP/atom/Diffusion.h"
#include <IMP/algebra/Vector3D.h>
#include <IMP/constants.h>
#include <IMP/atom/estimates.h>
IMPATOM_BEGIN_NAMESPACE






FloatKey Diffusion::get_d_key() {
  static FloatKey k("D");
  return k;
}


Diffusion Diffusion::setup_particle(Particle *p) {
  IMP_USAGE_CHECK(core::XYZR::particle_is_instance(p),
                  "Particle must already be an XYZR particle");
  double r= core::XYZR(p).get_radius();
  p->add_attribute(get_d_key(), get_einstein_diffusion_coefficient(r));
  return Diffusion(p);
}


void Diffusion::show(std::ostream &out) const
{
  XYZ::show(out);
  out << "D= " << get_d() << "A^2/fs";

}

double get_d_from_cm2_per_second(double din) {
  unit::SquareCentimeterPerSecond dinv(din);
  unit::SquareAngstromPerFemtosecond ret=dinv;
  return ret.get_value();
}

RigidBodyDiffusion RigidBodyDiffusion::setup_particle(Particle *p) {
  Diffusion::setup_particle(p);
  core::XYZR d(p);
  p->add_attribute(get_d_rotation_key(),
                get_einstein_rotational_diffusion_coefficient(d.get_radius()));
  return RigidBodyDiffusion(p);
}
FloatKey RigidBodyDiffusion::get_d_rotation_key() {
  static FloatKey k("D rotation");
  return k;
}


void RigidBodyDiffusion::show(std::ostream &out) const
{
  Diffusion::show(out);
  out << "D rotation= " << get_d() << "cm^2/sec";

}

IMPATOM_END_NAMESPACE
