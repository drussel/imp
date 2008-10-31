/**
 *  \file ParticleContainerSet.h
 *  \brief Store a set of ParticleContainers
 *
 *  This file is generated by a script (core/tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-8 Sali Lab. All rights reserved.
 */

#ifndef IMPCORE_PARTICLE_CONTAINER_SET_H
#define IMPCORE_PARTICLE_CONTAINER_SET_H

#include "core_exports.h"
#include "ParticleContainer.h"
#include "internal/core_version_info.h"

IMPCORE_BEGIN_NAMESPACE

//! Stores a set of ParticleContainers
/**
 */
class IMPCOREEXPORT ParticleContainerSet
  : public ParticleContainer
{
public:
  ParticleContainerSet();

  virtual ~ParticleContainerSet();

  IMP_PARTICLE_CONTAINER(internal::core_version_info);

  IMP_CONTAINER(ParticleContainer, particle_container,
                ParticleContainerIndex);
};


IMPCORE_END_NAMESPACE

#endif  /* IMPCORE_PARTICLE_CONTAINER_SET_H */
