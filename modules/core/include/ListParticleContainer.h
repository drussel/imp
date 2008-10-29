/**
 *  \file ListParticleContainer.h    \brief Store a list of Particles
 *
 *  This file is generated by a script (core/tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-8 Sali Lab. All rights reserved.
 */

#ifndef IMPCORE_LIST_PARTICLE_CONTAINER_H
#define IMPCORE_LIST_PARTICLE_CONTAINER_H

#include "core_exports.h"
#include "ParticleContainer.h"
#include "internal/core_version_info.h"

IMPCORE_BEGIN_NAMESPACE

//! Store a list of Particles
/** \note The indexes can change when particles are inserted
    as the list is maintained in sorted order.
 */
class IMPCOREEXPORT ListParticleContainer : public ParticleContainer
{
public:
  ListParticleContainer(const Particles &ps= Particles());

  virtual ~ListParticleContainer();

  //! log n time
  virtual bool get_contains_particle(Particle* vt) const;

  IMP_LIST(public, Particle, particle, Particle*);

  IMP::VersionInfo get_version_info() const {
    return internal::core_version_info;
  }

  //!
  virtual void show(std::ostream &out = std::cout) const;
};


IMPCORE_END_NAMESPACE

#endif  /* IMPCORE_LIST_PARTICLE_CONTAINER_H */
