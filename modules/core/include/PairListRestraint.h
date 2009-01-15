/**
 *  \file PairListRestraint.h
 *  \brief Apply a PairScore to each particle pair in a list.
 *
 *  Copyright 2007-8 Sali Lab. All rights reserved.
 *
 */

#ifndef IMPCORE_PAIR_LIST_RESTRAINT_H
#define IMPCORE_PAIR_LIST_RESTRAINT_H

#include "config.h"
#include "internal/core_version_info.h"

#include <IMP/Restraint.h>
#include <IMP/Particle.h>
#include <IMP/Pointer.h>
#include <IMP/PairScore.h>

#include <iostream>

IMPCORE_BEGIN_NAMESPACE

//! Applies a PairScore to each pair of particles in a list.
/**    \deprecated Use a ParticlePairsRestraint instead.
 */
class IMPCOREEXPORT PairListRestraint : public Restraint
{
public:
  //! Create the list restraint.
  /** \param[in] ss The function to apply to each particle.
      \param[in] ps The list of particle pairs to use in the restraints
   */
  PairListRestraint(PairScore *ss, const ParticlePairs &ps=ParticlePairs());
  virtual ~PairListRestraint();

  IMP_RESTRAINT(internal::core_version_info)

  IMP_LIST(public, ParticlePair, particle_pair, ParticlePair)

  virtual ParticlesList get_interacting_particles() const;
protected:
  Pointer<PairScore> ss_;
};

IMPCORE_END_NAMESPACE

#endif  /* IMPCORE_PAIR_LIST_RESTRAINT_H */
