/**
 *  \file FixedRefiner.h
 *  \brief A particle refiner which returns a fixed set of particles
 *
 *  Copyright 2007-2010 Sali Lab. All rights reserved.
 */

#ifndef IMPCORE_FIXED_REFINER_H
#define IMPCORE_FIXED_REFINER_H

#include "core_config.h"

#include <IMP/PairContainer.h>
#include <IMP/SingletonContainer.h>
#include <IMP/Refiner.h>
#include <IMP/container_macros.h>

IMPCORE_BEGIN_NAMESPACE

//! The refiner can refine any particle by returning a fixed set
/**
 */
class IMPCOREEXPORT FixedRefiner: public Refiner
{
  IMP_LIST(private, Particle, particle, Particle*, Particles);
public:
  //! Store the set of particles
  FixedRefiner(const Particles &ps);

  IMP_REFINER(FixedRefiner);
};


IMPCORE_END_NAMESPACE

#endif  /* IMPCORE_FIXED_REFINER_H */
