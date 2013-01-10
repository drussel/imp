/**
 *  \file Log.cpp   \brief Logging and error reporting support.
 *
 *  Copyright 2007-2013 IMP Inventors. All rights reserved.
 *
 */

#include "IMP/particle_index.h"
#include "IMP/internal/container_helpers.h"
IMP_BEGIN_NAMESPACE
ParticleIndexes get_indexes(const ParticlesTemp &ps) {
  return internal::get_index(ps);
}

ParticlesTemp get_particles(Model *m, const ParticleIndexes &ps) {
  return internal::get_particle(m, ps);
}

ParticleIndexPairs get_indexes(const ParticlePairsTemp &ps) {
  return internal::get_index(ps);
}


IMP_END_NAMESPACE
