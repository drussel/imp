/**
 *  \file ParticleRefiner.cpp \brief Refine a particle into a list of particles.
 *
 *  Copyright 2007-9 Sali Lab. All rights reserved.
 *
 */

#include "IMP/ParticleRefiner.h"

IMP_BEGIN_NAMESPACE

Particles ParticleRefiner::get_refined(Particle *p) const {
  throw ErrorException("Can't refine");
  return Particles();
}

IMP_END_NAMESPACE
